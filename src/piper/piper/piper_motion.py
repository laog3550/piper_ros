#!/usr/bin/env python3
"""Joint limits, motion profiles and gripper/reset planning."""

import math
import time
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

from piper.piper_feedback_decode import JOINT_COUNT

# Driver flash limits: query (0x472), feedback (0x473), set (0x474). The three
# frames share one layout: joint number, then the angle ceiling, the angle
# floor and the speed ceiling as big-endian 16-bit words.
LIMIT_QUERY_CAN_ID = 0x472
LIMIT_FEEDBACK_CAN_ID = 0x473
LIMIT_SET_CAN_ID = 0x474
LIMIT_FRAME_LENGTH = 8
# The set frame's third field is 0.001 rad/s per count and the driver accepts
# 0..3000.  There is a "leave this field alone" value (0x7FFF) but it only
# exists in firmware V1.5-2 and later; on older firmware it is read literally
# as 32.767 rad/s, so every field is always written explicitly instead.
MAX_JOINT_SPD_RAW = 3000
LEAVE_UNCHANGED_RAW = 0x7FFF
_ANGLE_LIMIT_SCALE = 0.1  # raw angle limits are 0.1 degree per count
_SPEED_SCALE = 0.001  # raw speed limits are 0.001 rad/s per count
QUERY_ANGLE_AND_SPEED = 0x01


@dataclass(frozen=True)
class JointLimit:
    """One joint's driver-flash limits, as read back or to be written."""

    joint: int
    max_angle_deg: float
    min_angle_deg: float
    max_joint_spd: int


def encode_limit_query(joint: int) -> bytes:
    """Build the frame that asks one driver to report its limits."""
    return bytes([joint, QUERY_ANGLE_AND_SPEED, 0, 0, 0, 0, 0, 0])


def encode_limit_set(limit: JointLimit) -> bytes:
    """Build the frame that writes one joint's limits to driver flash."""
    return bytes([limit.joint]) + int(
        round(limit.max_angle_deg / _ANGLE_LIMIT_SCALE)
    ).to_bytes(2, 'big', signed=True) + int(
        round(limit.min_angle_deg / _ANGLE_LIMIT_SCALE)
    ).to_bytes(2, 'big', signed=True) + int(
        limit.max_joint_spd
    ).to_bytes(2, 'big') + b'\x00'


def decode_joint_limit(can_id: int, data) -> Optional[JointLimit]:
    """Decode one limit feedback frame, or None if it is not one."""
    if can_id != LIMIT_FEEDBACK_CAN_ID or len(data) < LIMIT_FRAME_LENGTH:
        return None
    joint = data[0]
    if not 1 <= joint <= JOINT_COUNT:
        return None
    return JointLimit(
        joint=joint,
        max_angle_deg=int.from_bytes(data[1:3], 'big', signed=True)
        * _ANGLE_LIMIT_SCALE,
        min_angle_deg=int.from_bytes(data[3:5], 'big', signed=True)
        * _ANGLE_LIMIT_SCALE,
        max_joint_spd=int.from_bytes(data[5:7], 'big'),
    )


# Joint position command limits, in degrees, as they apply to JointCtrl
# (CAN 0x155/0x156/0x157).  A target outside its range is not rejected by the
# driver, it is clamped to the nearest bound, which turns "hold this joint
# still" into a real movement whenever the joint currently sits outside its
# range.
#
# Verified against the driver with GetAllMotorAngleLimitMaxSpd() (CAN 0x473).
# Two deviations from the JointCtrl docstring were found and the driver wins:
#   joint6  driver reports +-180 where the docstring says +-120;
#   joint2  lower bound widened 0 -> -2.0 on 2026-09-23
#           (MotorAngleLimitMaxSpdSet, CAN 0x474) so that a gravity sag onto
#           the mechanical stop no longer sits fully outside the commandable
#           range.  Applied to all four arms, whose factory limits were
#           identical (can_fl, can_mr, can_fr, can_ml all read back the same
#           table).
#   joint3  upper bound widened 0 -> +1.2, then +1.2 -> +2.0 the same day, for
#           the same reason: both masters settle at +2.05 and +2.10 degrees, so
#           +1.2 left the follower unable to match the master pose at all.  The
#           new bound sits 0.1 degree below the highest angle a joint was
#           actually observed to reach (2.102), so it cannot point at an
#           unreachable position.  Motor angle limits step in 0.1 degree, which
#           is why these are round values.
# These limits live in driver flash, so they are per-arm and can be changed:
# after any MotorAngleLimitMaxSpdSet, update this table or re-read the arm,
# otherwise the overshoot this module predicts will be wrong.
JOINT_COMMAND_LIMITS_DEG = {
    1: (-150.0, 150.0),
    2: (-2.0, 180.0),
    3: (-170.0, 2.0),
    4: (-100.0, 100.0),
    5: (-70.0, 70.0),
    6: (-180.0, 180.0),
}


def out_of_limits(angles_deg: Dict[int, float]) -> Dict[int, float]:
    """
    Report how far each joint angle exceeds its commandable range.

    The returned mapping holds one positive overshoot per offending joint, so
    an empty result means every angle can be commanded as-is.
    """
    overshoot = {}
    for joint, angle in angles_deg.items():
        bounds = JOINT_COMMAND_LIMITS_DEG.get(joint)
        if bounds is None:
            continue
        low, high = bounds
        if angle < low:
            overshoot[joint] = low - angle
        elif angle > high:
            overshoot[joint] = angle - high
    return overshoot


def minimum_jerk_step(progress):
    """Return an S-curve with zero endpoint velocity and acceleration."""
    if progress <= 0.0:
        return 0.0
    if progress >= 1.0:
        return 1.0
    return progress ** 3 * (
        10.0 + progress * (-15.0 + 6.0 * progress))


def align_targets(start_deg, goal_deg, progress):
    """
    Interpolate a whole pose along a quintic minimum-jerk S-curve.

    ``progress`` is the fraction of the alignment move already elapsed, so 0
    yields exactly ``start_deg`` and 1 yields exactly ``goal_deg``.
    """
    fraction = minimum_jerk_step(progress)
    return {
        joint: start_deg[joint]
        + fraction * (goal_deg[joint] - start_deg[joint])
        for joint in start_deg
    }


def clamp_targets(targets_deg):
    """
    Clamp every target into its commandable range.

    Returns the clamped targets plus the joints that had to be clamped, since
    a clamped joint means the follower cannot follow the master any further.
    """
    clamped = {}
    limited = {}
    for joint, angle in targets_deg.items():
        low, high = JOINT_COMMAND_LIMITS_DEG[joint]
        fixed = max(low, min(angle, high))
        clamped[joint] = fixed
        if abs(fixed - angle) > 1e-9:
            limited[joint] = angle - fixed
    return clamped, limited


def limit_step(targets_deg, previous_deg, max_step_deg):
    """
    Limit how far each target may move from the previous published one.

    This is a guard against implausible jumps in the incoming angles, not a
    speed limit: a real master can never move a joint this far in one control
    period, so only noise or a misread can trip it.
    """
    if previous_deg is None:
        return dict(targets_deg), {}
    limited = {}
    stepped = {}
    for joint, angle in targets_deg.items():
        delta = angle - previous_deg[joint]
        if abs(delta) > max_step_deg:
            fixed = previous_deg[joint] + math.copysign(max_step_deg, delta)
            limited[joint] = delta - math.copysign(max_step_deg, delta)
            angle = fixed
        stepped[joint] = angle
    return stepped, limited


# 夹爪不是关节：它的可指令量是"开口"，单位是**米**，范围 0~0.08 m。
# joint_states 的第 7 项就是米（节点把驱动的 1e-6 m 计数除以 1e6），主从两侧同
# 单位，所以遥操作只需限幅、不需要换算。驱动侧真正关心的两个量是开口和夹持力：
# 开口在节点的 joint_callback 里乘 1e6 交给 GripperCtrl，夹持力取自 effort[6]。
GRIPPER_OPEN_MIN_M = 0.0
GRIPPER_OPEN_MAX_M = 0.08
# 手捏主臂夹爪时反馈带着 0.1 mm 量级的抖动；小于这个幅度的变化不传递，否则从臂
# 夹爪会一直追着噪声跑（听起来是"嗡嗡"响）。
DEFAULT_GRIPPER_DEADBAND_M = 0.0005
# 从臂夹爪的夹持力，单位 N·m；节点的 joint_callback 会把它钳到 [0.5, 3]。
DEFAULT_GRIPPER_EFFORT_NM = 1.0
# 主从夹爪的行程倍数：master 的开口乘上它才是 follower 的开口。本机构上主臂夹爪
# 的有效行程只有从臂的约 1/1.3（从臂 0~80mm），所以默认 1.3——主臂走满行程正好
# 对应从臂走满行程，中间按比例跟随。两台夹爪完全相同时把它设成 1.0 就是纯镜像。
DEFAULT_GRIPPER_SCALE = 1.3

# 快速复位手势：一次手势是 master 做出一段短促移动后重新停下；两次
# 手势在窗口内完成就触发 follower 回 home。检测器只处理角度和时间，
# 不依赖 ROS，左右两臂各自实例化后不会共享状态。
DEFAULT_QUICK_RESET_WINDOW_S = 2.0
DEFAULT_QUICK_RESET_TRIGGER_SPEED_DEG_S = 20.0
DEFAULT_QUICK_RESET_RELEASE_SPEED_DEG_S = 5.0
DEFAULT_QUICK_RESET_MIN_TRAVEL_DEG = 3.0


def clamp_gripper(open_m: float) -> float:
    """Clamp a gripper opening into the commandable range, in metres."""
    return max(GRIPPER_OPEN_MIN_M, min(float(open_m), GRIPPER_OPEN_MAX_M))


def scale_gripper(open_m: float,
                  scale: float = DEFAULT_GRIPPER_SCALE) -> float:
    """Turn a master opening into the follower opening: multiply, then clamp."""
    return clamp_gripper(float(open_m) * float(scale))


class DoubleMotionResetDetector:
    """Detect two short master motion bursts inside a time window."""

    def __init__(self,
                 window_s: float = DEFAULT_QUICK_RESET_WINDOW_S,
                 trigger_speed_deg_s: float =
                 DEFAULT_QUICK_RESET_TRIGGER_SPEED_DEG_S,
                 release_speed_deg_s: float =
                 DEFAULT_QUICK_RESET_RELEASE_SPEED_DEG_S,
                 min_travel_deg: float = DEFAULT_QUICK_RESET_MIN_TRAVEL_DEG):
        if window_s <= 0.0:
            raise ValueError('window_s must be positive')
        if trigger_speed_deg_s <= 0.0:
            raise ValueError('trigger speed must be positive')
        if not 0.0 <= release_speed_deg_s < trigger_speed_deg_s:
            raise ValueError('release speed must be below trigger speed')
        if min_travel_deg <= 0.0:
            raise ValueError('minimum travel must be positive')
        self.window_s = float(window_s)
        self.trigger_speed_deg_s = float(trigger_speed_deg_s)
        self.release_speed_deg_s = float(release_speed_deg_s)
        self.min_travel_deg = float(min_travel_deg)
        self.reset()

    def reset(self) -> None:
        """Forget the previous burst and double-motion history."""
        self._last_angles = None
        self._last_time = None
        self._moving = False
        self._burst_start = None
        self._burst_travel = 0.0
        self._last_tap_time = None

    def update(self, angles_deg: Dict[int, float], now: float) -> bool:
        """Return true when two completed motion bursts are detected."""
        values = {
            joint: float(angles_deg[joint])
            for joint in range(1, JOINT_COUNT + 1)
            if joint in angles_deg and math.isfinite(float(angles_deg[joint]))
        }
        if len(values) != JOINT_COUNT:
            self._last_angles = None
            self._last_time = None
            return False

        speed = 0.0
        if self._last_angles is not None and self._last_time is not None:
            dt = float(now) - self._last_time
            if dt > 0.0:
                speed = max(abs(values[joint] - self._last_angles[joint])
                            for joint in values) / dt

        if (self._last_tap_time is not None
                and float(now) - self._last_tap_time > self.window_s):
            self._last_tap_time = None

        if not self._moving and speed >= self.trigger_speed_deg_s:
            self._moving = True
            self._burst_start = dict(values)
            self._burst_travel = 0.0
        elif self._moving:
            self._burst_travel = max(
                self._burst_travel,
                max(abs(values[joint] - self._burst_start[joint])
                    for joint in values))
            if speed <= self.release_speed_deg_s:
                self._moving = False
                if self._burst_travel >= self.min_travel_deg:
                    if (self._last_tap_time is not None
                            and float(now) - self._last_tap_time <= self.window_s):
                        self._last_tap_time = None
                        self._last_angles = values
                        self._last_time = float(now)
                        return True
                    self._last_tap_time = float(now)

        self._last_angles = values
        self._last_time = float(now)
        return False


# The quintic minimum-jerk S-curve peaks at 1.875 times the average speed
# (the derivative of 10t^3-15t^4+6t^5 reaches 1.875 at t=0.5).
MINIMUM_JERK_PEAK_FACTOR = 1.875
# Return-home is a known point-to-point move.  The old 10/15 deg/s values made
# a normal return visibly slow; 20/40 shortens it by roughly half while keeping
# the planned peak well below the measured 86 deg/s sustained capability.
DEFAULT_RETURN_SPEED_DEG_S = 20.0
DEFAULT_RETURN_MAX_PEAK_DEG_S = 40.0
# Samples taken along the return path when reporting which joints the driver
# would have to clamp.  Clamping is only possible where the pose or the home
# pose sits outside the commandable range, so a coarse sweep is enough.
RETURN_PATH_SAMPLES = 64
# 实测的持续跟随速度上限，单位 deg/s，用于提示回位计划是否超出机械臂能力。
#
# 2026-09-23 实测（左从臂 `follower_left`，当天其接口名叫 `can_fr`；
# `max_joint_spd` 保持出厂 300 未改）：遥操作把
# 速度百分比从 10% 提到 100% 后，follower 的持续速度达到 45.6~86.2 deg/s，并且
# 跟着拖动速度走（j1 45.6 / master 44.1，j3 82.7 / 83.7），因此**驱动器的
# `max_joint_spd` 在「CAN 流式位置指令 + MotionCtrl_2 百分比」这条控制路径上
# 不是运动速度的硬上限**——当时压住跟随的是遥操作默认的 `--speed 10`
# （10% × 3 rad/s ≈ 17.2 deg/s，与 300 的换算值恰好相同）。
#
# 所以这里用**实测**到的能力值，而不是固件的限位值：它只用于告警，不参与钳制。
MEASURED_MAX_JOINT_SPD_DEG_S = 86.0


def return_duration(max_delta_deg,
                    speed_deg_s=DEFAULT_RETURN_SPEED_DEG_S,
                    max_peak_deg_s=DEFAULT_RETURN_MAX_PEAK_DEG_S) -> float:
    """
    Seconds a minimum-jerk return move of ``max_delta_deg`` should take.

    The move has no fixed duration: it falls out of the distance and the two
    speed limits.  The average speed implies ``distance / speed`` seconds and
    the peak ceiling implies ``1.875 * distance / peak``; the longer of the
    two wins, so neither limit is exceeded.
    """
    if speed_deg_s <= 0.0 or max_peak_deg_s <= 0.0:
        raise ValueError('speed and peak limits must be positive')
    if max_delta_deg <= 0.0:
        return 0.0
    return max(max_delta_deg / speed_deg_s,
               MINIMUM_JERK_PEAK_FACTOR * max_delta_deg / max_peak_deg_s)


@dataclass(frozen=True)
class ReturnPlan:
    """A minimum-jerk move from the final pose back to its home pose."""

    deltas_deg: Dict[int, float]
    max_delta_deg: float
    duration: float
    peak_deg_s: float
    clamped_deg: Dict[int, float]


def plan_return(start_deg, home_deg,
                speed_deg_s=DEFAULT_RETURN_SPEED_DEG_S,
                max_peak_deg_s=DEFAULT_RETURN_MAX_PEAK_DEG_S,
                samples: int = RETURN_PATH_SAMPLES) -> ReturnPlan:
    """
    Describe the return move before any of it is executed.

    Reports the per-joint displacement, how long the move takes, the peak
    speed the fastest joint reaches, and which joints the path cannot command
    as-is.  ``clamped_deg`` holds the largest correction the driver would
    apply to that joint anywhere along the path, so an empty mapping means the
    whole path can be sent unchanged.
    """
    if samples < 1:
        raise ValueError('samples must be at least 1')
    deltas = {
        joint: home_deg[joint] - start_deg[joint] for joint in start_deg
    }
    max_delta = max((abs(delta) for delta in deltas.values()), default=0.0)
    duration = return_duration(max_delta, speed_deg_s, max_peak_deg_s)
    clamped: Dict[int, float] = {}
    for step in range(samples + 1):
        path = align_targets(start_deg, home_deg, step / samples)
        _, limited = clamp_targets(path)
        for joint, correction in limited.items():
            clamped[joint] = max(clamped.get(joint, 0.0), abs(correction))
    return ReturnPlan(
        deltas_deg=deltas,
        max_delta_deg=max_delta,
        duration=duration,
        peak_deg_s=(MINIMUM_JERK_PEAK_FACTOR * max_delta / duration
                    if duration > 0.0 else 0.0),
        clamped_deg=clamped,
    )


# Time constant of the follow-phase low-pass filter, in seconds.

__all__ = [
    'LIMIT_QUERY_CAN_ID',
    'LIMIT_FEEDBACK_CAN_ID',
    'LIMIT_SET_CAN_ID',
    'LIMIT_FRAME_LENGTH',
    'MAX_JOINT_SPD_RAW',
    'LEAVE_UNCHANGED_RAW',
    'QUERY_ANGLE_AND_SPEED',
    'JointLimit',
    'encode_limit_query',
    'encode_limit_set',
    'decode_joint_limit',
    'JOINT_COMMAND_LIMITS_DEG',
    'out_of_limits',
    'minimum_jerk_step',
    'align_targets',
    'clamp_targets',
    'limit_step',
    'GRIPPER_OPEN_MIN_M',
    'GRIPPER_OPEN_MAX_M',
    'DEFAULT_GRIPPER_DEADBAND_M',
    'DEFAULT_GRIPPER_EFFORT_NM',
    'DEFAULT_GRIPPER_SCALE',
    'DEFAULT_QUICK_RESET_WINDOW_S',
    'DEFAULT_QUICK_RESET_TRIGGER_SPEED_DEG_S',
    'DEFAULT_QUICK_RESET_RELEASE_SPEED_DEG_S',
    'DEFAULT_QUICK_RESET_MIN_TRAVEL_DEG',
    'clamp_gripper',
    'scale_gripper',
    'DoubleMotionResetDetector',
    'MINIMUM_JERK_PEAK_FACTOR',
    'DEFAULT_RETURN_SPEED_DEG_S',
    'DEFAULT_RETURN_MAX_PEAK_DEG_S',
    'RETURN_PATH_SAMPLES',
    'MEASURED_MAX_JOINT_SPD_DEG_S',
    'return_duration',
    'ReturnPlan',
    'plan_return',
]
