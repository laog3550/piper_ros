#!/usr/bin/env python3
"""Decode Piper driver feedback frames and track how fresh they are."""

# Frames are decoded straight from the CAN bus, so this module depends on
# neither piper_sdk nor ROS.  Callers feed it frames the arm already
# broadcasts; nothing here transmits.

import math
import time
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

JOINT_COUNT = 6
# Low-speed driver information feedback, one frame per joint.
FEEDBACK_FIRST_CAN_ID = 0x261
FEEDBACK_CAN_IDS = tuple(
    range(FEEDBACK_FIRST_CAN_ID, FEEDBACK_FIRST_CAN_ID + JOINT_COUNT)
)
FEEDBACK_DATA_LENGTH = 8
# Byte 5 of the frame is the driver status word; bit 6 is driver_enable_status.
STATUS_BYTE = 5
ENABLE_BIT = 0x40

# High-speed driver information feedback, one frame per joint.
HIGH_SPEED_CAN_IDS = tuple(range(0x251, 0x257))
# Joint angles are packed two per frame.
JOINT_ANGLE_IDS = {0x2A5: (1, 2), 0x2A6: (3, 4), 0x2A7: (5, 6)}
ARM_STATUS_CAN_ID = 0x2A1
END_POSE_CAN_IDS = (0x2A2, 0x2A3, 0x2A4)
GRIPPER_FEEDBACK_CAN_ID = 0x2A8
# Every frame a healthy Piper arm broadcasts on its own, in any arm mode.
CORE_FEEDBACK_CAN_IDS = (
    HIGH_SPEED_CAN_IDS
    + FEEDBACK_CAN_IDS
    + (ARM_STATUS_CAN_ID,)
    + END_POSE_CAN_IDS
    + tuple(JOINT_ANGLE_IDS)
    + (GRIPPER_FEEDBACK_CAN_ID,)
)
# Making an arm the teaching input arm shifts its feedback IDs by one of these
# amounts (CAN 0x470); see MasterSlaveConfig in the SDK.  An arm that never
# received that command stays a motion output arm and keeps the plain IDs.
TEACHING_INPUT_OFFSETS = (0x10, 0x20)
_ANGLE_SCALE = 0.001  # raw joint angles are 0.001 degree per count

# True = enabled, False = disabled, None = no usable recent frame.
JointObservations = Tuple[Optional[bool], ...]


def joint_for_can_id(can_id: int) -> Optional[int]:
    """Map a low-speed feedback CAN ID to its 1-based joint number."""
    if FEEDBACK_FIRST_CAN_ID <= can_id <= FEEDBACK_CAN_IDS[-1]:
        return can_id - FEEDBACK_FIRST_CAN_ID + 1
    return None


def detect_feedback_offset(observed_can_ids) -> Optional[int]:
    """Return the whole-set feedback offset, or None for the plain layout."""
    # The offset cannot be decided from a single ID: shifting a high-speed
    # frame by 0x20 lands on the same ID as shifting a low-speed frame by
    # 0x10, so only the whole set disambiguates the layout being used.  A
    # plain arm yields None because no offset maps all its core IDs back.
    unique = set(observed_can_ids)
    for offset in TEACHING_INPUT_OFFSETS:
        matched = sum(
            1 for can_id in unique
            if can_id - offset in CORE_FEEDBACK_CAN_IDS
        )
        if matched >= len(CORE_FEEDBACK_CAN_IDS):
            return offset
    return None


def decode_joint_angles(can_id: int, data) -> Dict[int, float]:
    """Decode one joint angle frame into degrees, keyed by joint number."""
    joints = JOINT_ANGLE_IDS.get(can_id)
    if joints is None or len(data) < 8:
        return {}
    return {
        joints[0]: int.from_bytes(data[0:4], 'big', signed=True) * _ANGLE_SCALE,
        joints[1]: int.from_bytes(data[4:8], 'big', signed=True) * _ANGLE_SCALE,
    }


def _status_byte(data) -> Optional[int]:
    if len(data) < FEEDBACK_DATA_LENGTH:
        return None
    return data[STATUS_BYTE]


@dataclass(frozen=True)
class DriverFeedback:
    """One decoded low-speed driver feedback frame."""

    joint: int
    enabled: bool
    status_code: int
    voltage: float
    foc_temperature: int


def decode(can_id: int, data) -> Optional[DriverFeedback]:
    """Decode one feedback frame, or None if it is not one."""
    joint = joint_for_can_id(can_id)
    if joint is None:
        return None
    status = _status_byte(data)
    if status is None:
        return None
    raw_voltage = int.from_bytes(data[:2], 'big')
    raw_temperature = int.from_bytes(data[2:4], 'big', signed=True)
    return DriverFeedback(
        joint=joint,
        enabled=bool(status & ENABLE_BIT),
        status_code=status,
        voltage=raw_voltage * 0.1,
        foc_temperature=raw_temperature,
    )


class FeedbackTracker:
    """Keep the newest feedback frame per joint and how old it is."""

    def __init__(self, timeout: float = 0.5):
        if timeout <= 0:
            raise ValueError('timeout must be positive')
        self.timeout = timeout
        self._feedback: Dict[int, DriverFeedback] = {}
        self._last_seen: Dict[int, float] = {}

    def update(self, can_id: int, data, now: Optional[float] = None):
        """Record one frame and return the decoded feedback it carried."""
        feedback = decode(can_id, data)
        if feedback is None:
            return None
        self._feedback[feedback.joint] = feedback
        self._last_seen[feedback.joint] = (
            time.monotonic() if now is None else now
        )
        return feedback

    def age(self, joint: int, now: Optional[float] = None):
        """Seconds since this joint's newest frame, or None if never seen."""
        seen = self._last_seen.get(joint)
        if seen is None:
            return None
        current = time.monotonic() if now is None else now
        return max(0.0, current - seen)

    def known_joints(self) -> Tuple[int, ...]:
        """Joints that have sent at least one feedback frame."""
        return tuple(sorted(self._feedback))

    def is_fresh(self, joint: int, now: Optional[float] = None) -> bool:
        """Whether this joint's newest frame is within the timeout."""
        age = self.age(joint, now)
        return age is not None and age <= self.timeout

    def last_known(self, joint: int):
        """Return the newest decoded frame for this joint, fresh or not."""
        return self._feedback.get(joint)

    def observations(self, now: Optional[float] = None) -> JointObservations:
        """Per-joint enable bits, with None for missing or stale joints."""
        result = []
        for joint in range(1, JOINT_COUNT + 1):
            if not self.is_fresh(joint, now):
                # A joint that stopped reporting must not keep contributing
                # its last enable bit as if it were current.
                result.append(None)
                continue
            result.append(self._feedback[joint].enabled)
        return tuple(result)

    def stale_joints(self, now: Optional[float] = None) -> Tuple[int, ...]:
        """Joints already seen at least once but silent past the timeout."""
        return tuple(
            joint for joint in self.known_joints()
            if not self.is_fresh(joint, now)
        )

    def missing_joints(self) -> Tuple[int, ...]:
        """Joints that have never sent a feedback frame."""
        return tuple(
            joint for joint in range(1, JOINT_COUNT + 1)
            if joint not in self._feedback
        )


class ArmAngleTracker:
    """Track one arm's six joint angles against a baseline pose."""

    def __init__(self):
        self._angles: Dict[int, float] = {}
        self._baseline: Dict[int, float] = {}
        self._max_offset: Dict[int, float] = {}
        self._last_angle: Dict[int, float] = {}
        self._updated: Optional[float] = None
        self._last_change: Optional[float] = None

    def update(self, can_id: int, data, now: float = None,
               change_threshold: float = 0.05) -> Tuple[int, ...]:
        """Store the angles carried by one frame; return the joints it moved."""
        decoded = decode_joint_angles(can_id, data)
        if not decoded:
            return ()
        current = time.monotonic() if now is None else now
        moved = []
        for joint, angle in decoded.items():
            previous = self._last_angle.get(joint)
            if previous is None or abs(angle - previous) >= change_threshold:
                moved.append(joint)
                self._last_change = current
            self._last_angle[joint] = angle
            self._angles[joint] = angle
            if joint not in self._baseline:
                self._baseline[joint] = angle
            offset = abs(angle - self._baseline[joint])
            if joint not in self._max_offset or offset > self._max_offset[joint]:
                self._max_offset[joint] = offset
        self._updated = current
        return tuple(moved)

    def set_baseline(self) -> None:
        """Re-anchor every joint to its current angle and forget the peaks."""
        self._baseline = dict(self._angles)
        # Seed a zero peak per known joint, so a joint that simply has not
        # moved is shown as 0.0 rather than as missing data.
        self._max_offset = {joint: 0.0 for joint in self._angles}

    def angles(self) -> Dict[int, float]:
        """Newest angle of each joint that has reported."""
        return dict(self._angles)

    def offsets(self) -> Dict[int, float]:
        """Signed displacement of each joint from its baseline."""
        return {
            joint: angle - self._baseline[joint]
            for joint, angle in self._angles.items()
            if joint in self._baseline
        }

    def max_offsets(self) -> Dict[int, float]:
        """Largest absolute displacement each joint reached since baseline."""
        return dict(self._max_offset)

    def age(self, now: Optional[float] = None) -> Optional[float]:
        """Seconds since the newest angle frame, or None if none arrived."""
        if self._updated is None:
            return None
        current = time.monotonic() if now is None else now
        return max(0.0, current - self._updated)

    def is_moving(self, window: float = 0.5,
                  now: Optional[float] = None) -> bool:
        """Whether a joint changed within the last ``window`` seconds."""
        if self._last_change is None:
            return False
        current = time.monotonic() if now is None else now
        return (current - self._last_change) <= window

    def moved_joints(self, threshold: float) -> Tuple[int, ...]:
        """Joints displaced from the baseline by at least ``threshold``."""
        return tuple(sorted(
            joint for joint, offset in self._max_offset.items()
            if offset >= threshold
        ))


# Driver flash limits: query (0x472), feedback (0x473), set (0x474).  The three
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


def cubic_step(progress):
    """
    Smoothstep easing: an S-curve with zero velocity at both ends.

    Used to move the follower from its own pose to the master's pose without
    a velocity jump at either end of the move.
    """
    if progress <= 0.0:
        return 0.0
    if progress >= 1.0:
        return 1.0
    return progress * progress * (3.0 - 2.0 * progress)


def align_targets(start_deg, goal_deg, progress):
    """
    Interpolate a whole pose from start to goal along the cubic S-curve.

    ``progress`` is the fraction of the alignment move already elapsed, so 0
    yields exactly ``start_deg`` and 1 yields exactly ``goal_deg``.
    """
    fraction = cubic_step(progress)
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


# The cubic S-curve used for the alignment move peaks at 1.5 times the average
# speed of the same displacement (the derivative of 3t^2-2t^3 reaches 1.5 at
# t=0.5), so a move of D degrees spread over T seconds never exceeds 1.5*D/T.
CUBIC_PEAK_FACTOR = 1.5
# Return-home defaults: 10 deg/s average and a 15 deg/s peak ceiling, which
# happen to imply the same duration because 1.5 * 10 == 15.  They are two
# independent knobs rather than one, so either can be tightened on its own.
DEFAULT_RETURN_SPEED_DEG_S = 10.0
DEFAULT_RETURN_MAX_PEAK_DEG_S = 15.0
# Samples taken along the return path when reporting which joints the driver
# would have to clamp.  Clamping is only possible where the pose or the home
# pose sits outside the commandable range, so a coarse sweep is enough.
RETURN_PATH_SAMPLES = 64
# 实测的持续跟随速度上限，单位 deg/s，用于提示回位计划是否超出机械臂能力。
#
# 2026-09-23 实测（左从臂 can_fr，`max_joint_spd` 保持出厂 300 未改）：遥操作把
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
    Seconds a cubic return move of ``max_delta_deg`` should take.

    The move has no fixed duration: it falls out of the distance and the two
    speed limits.  The average speed implies ``distance / speed`` seconds and
    the peak ceiling implies ``1.5 * distance / peak``; the longer of the two
    wins, so neither limit is exceeded.
    """
    if speed_deg_s <= 0.0 or max_peak_deg_s <= 0.0:
        raise ValueError('speed and peak limits must be positive')
    if max_delta_deg <= 0.0:
        return 0.0
    return max(max_delta_deg / speed_deg_s,
               CUBIC_PEAK_FACTOR * max_delta_deg / max_peak_deg_s)


@dataclass(frozen=True)
class ReturnPlan:
    """A cubic move from the pose a run ended in back to its home pose."""

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
        peak_deg_s=(CUBIC_PEAK_FACTOR * max_delta / duration
                    if duration > 0.0 else 0.0),
        clamped_deg=clamped,
    )


# Time constant of the follow-phase low-pass filter, in seconds.
DEFAULT_FILTER_TAU_S = 0.02
# Deadband, in degrees, applied to the follow-phase master reading before it
# reaches the filter.  Any move smaller than this is not passed on at all, so
# the follower holds perfectly still instead of echoing a hand that is
# nominally resting: One Euro already flattens the 5-15 Hz band, but a slow
# wobble below its 1 Hz cutoff still gets through.  The cost is that the
# target may sit up to this far from the master's true angle after a move.
DEFAULT_DEADBAND_DEG = 0.2
# 速度门限：输入速度低于它才认为"手停着"。实测手在静止时的慢晃（0.5 Hz、±0.15 度）
# 速度约 0.47 deg/s，而人刻意慢拖至少有 1 deg/s 量级，所以 0.8 能把两者分开。
DEFAULT_DEADBAND_SPEED_DEG_S = 0.8
# One Euro filter defaults (Casiez et al., CHI 2012).  The cutoff is
# ``min_cutoff + beta * speed`` in Hz and deg/s, so beta is the adaptive part:
# 0.3 Hz per deg/s.  Chosen from a measured comparison at 50 Hz against a
# 10 Hz +-0.3 degree hand tremor: at rest this leaves 0.036 degrees of wobble
# where the fixed low-pass (tau=0.02s) leaves 0.146, and while dragging at
# 100 deg/s it lags 0.43 degrees where the low-pass lags 2.00.
DEFAULT_ONE_EURO_MIN_CUTOFF_HZ = 1.0
DEFAULT_ONE_EURO_BETA = 0.3
DEFAULT_ONE_EURO_D_CUTOFF_HZ = 1.0


def _smoothing_factor(dt: float, cutoff_hz: float) -> float:
    """First-order low-pass coefficient for one interval and cutoff."""
    rate = 2.0 * math.pi * cutoff_hz * dt
    return rate / (rate + 1.0)


class OneEuroFilter:
    """
    Velocity-adaptive low-pass, one state per joint.

    A fixed cutoff cannot both kill hand tremor and avoid lag: suppressing the
    8-12 Hz band means a cutoff near 1 Hz, which lags by ``speed / (2*pi*fc)``
    -- over 1.5 degrees while dragging at 30 deg/s.  This filter moves its own
    cutoff with the signal instead: ``fc = min_cutoff + beta * |dx|``, where
    ``dx`` is the derivative smoothed by its own low-pass (``d_cutoff``).  The
    derivative smoother is what keeps tremor from raising the cutoff that is
    supposed to suppress it, so keep ``d_cutoff`` low.

    Parameters are per-joint and seeded by :meth:`reset`, and ``dt`` is the
    measured loop interval, exactly as in :class:`LowPassFilter`.

    Like any smoothing filter this shapes the signal without bounding it: a
    step still leaves the output short by roughly ``dx * dt / tau_effective``,
    so a speed ceiling is still the driver's business, not the filter's.
    """

    def __init__(self, min_cutoff: float = DEFAULT_ONE_EURO_MIN_CUTOFF_HZ,
                 beta: float = DEFAULT_ONE_EURO_BETA,
                 d_cutoff: float = DEFAULT_ONE_EURO_D_CUTOFF_HZ):
        if min_cutoff <= 0.0:
            raise ValueError('min_cutoff must be positive')
        if d_cutoff <= 0.0:
            raise ValueError('d_cutoff must be positive')
        if beta < 0.0:
            raise ValueError('beta must not be negative')
        self.min_cutoff = float(min_cutoff)
        self.beta = float(beta)
        self.d_cutoff = float(d_cutoff)
        self._state: Dict[int, Tuple[float, float]] = {}

    def reset(self, values):
        """Seed the filter with a reading so the first output has no jump."""
        self._state = {
            joint: (float(value), 0.0) for joint, value in values.items()
        }
        return dict(values)

    def velocities(self):
        """
        Return the filtered derivative per joint, in degrees per second.

        This is the smoothed speed estimate the filter already maintains; the
        follow-phase smoother uses it as velocity feed-forward so that a
        steady drag is tracked without lag.
        """
        return {joint: state[1] for joint, state in self._state.items()}

    def update(self, values, dt: float):
        """Advance the filter by ``dt`` seconds and return the smoothed values."""
        smoothed = {}
        for joint, sample in values.items():
            state = self._state.get(joint)
            if state is None:
                # A joint that appears for the first time is taken as-is:
                # there is no history to smooth against.
                self._state[joint] = (sample, 0.0)
                smoothed[joint] = sample
                continue
            x_prev, dx_prev = state
            if dt <= 0.0:
                # No time passed, so nothing is smoothed towards yet.
                smoothed[joint] = x_prev
                continue
            a_d = _smoothing_factor(dt, self.d_cutoff)
            dx_hat = a_d * (sample - x_prev) / dt + (1.0 - a_d) * dx_prev
            cutoff = self.min_cutoff + self.beta * abs(dx_hat)
            a = _smoothing_factor(dt, cutoff)
            x_hat = a * sample + (1.0 - a) * x_prev
            self._state[joint] = (x_hat, dx_hat)
            smoothed[joint] = x_hat
        return smoothed


class DeadbandGate:
    """
    Hold the reading still while the hand is at rest; follow it otherwise.

    A joint's output is held only when **both** conditions hold: the input has
    not moved further than ``threshold`` from its anchor, and the input's
    smoothed speed is below ``speed_threshold``.  The speed test is what keeps
    a *slow but deliberate* drag continuous: gating on amplitude alone chops
    such a drag into steps of one threshold, and between the steps the
    commanded velocity is zero -- the arm then stops and re-accelerates over
    and over (2026-09-23 实测：慢拖时指令速度反复归零，操作者描述为"每一小段
    速度都会归为零然后重新加速"）。As soon as the input is moving, the anchor
    follows it and nothing is held.

    The speed estimate is this gate's own: a first-order low-pass (1 Hz) of
    the per-cycle difference, so it does not depend on anything upstream.

    ``threshold_deg = 0`` passes everything through unchanged, and
    ``speed_threshold = 0`` reduces the gate to the amplitude-only behaviour.
    """

    # 速度估计的低通截止频率。3 Hz 让它在约 0.05 秒（3 个周期）内建立起来，
    # 于是"慢拖被切成台阶"只剩开头这几十毫秒、幅度远小于阈值；再慢就来不及
    # 区分"手停着晃"与"手在动"。
    SPEED_CUTOFF_HZ = 3.0

    def __init__(self, threshold_deg: float = DEFAULT_DEADBAND_DEG,
                 speed_threshold: float = DEFAULT_DEADBAND_SPEED_DEG_S):
        if threshold_deg < 0.0:
            raise ValueError('threshold must not be negative')
        if speed_threshold < 0.0:
            raise ValueError('speed threshold must not be negative')
        self.threshold = float(threshold_deg)
        self.speed_threshold = float(speed_threshold)
        self._anchor: Dict[int, float] = {}
        self._speed: Dict[int, float] = {}
        self._last: Dict[int, float] = {}

    def reset(self, values):
        """Anchor every joint to the given reading and forget its speed."""
        self._anchor = {joint: float(value) for joint, value in values.items()}
        self._speed = {joint: 0.0 for joint in values}
        self._last = dict(self._anchor)
        return dict(values)

    def update(self, values, dt: float):
        """Return the reading to use, holding joints that are simply resting."""
        alpha = _smoothing_factor(max(dt, 1e-9), self.SPEED_CUTOFF_HZ)
        gated = {}
        for joint, sample in values.items():
            anchor = self._anchor.get(joint, sample)
            # 速度取**相邻两帧输入**之差：若拿"与锚点之差"来算，锚点被保持期间
            # 这个差会一路虚高，反而把静止判成运动。
            previous = self._last.get(joint, sample)
            speed = self._speed.get(joint, 0.0)
            if dt > 0.0:
                speed = (alpha * abs(sample - previous) / dt
                         + (1.0 - alpha) * speed)
            # 速度门限为 0 表示不做速度判断，退化成只按幅度保持。
            moving = (self.speed_threshold > 0.0
                      and speed >= self.speed_threshold)
            if moving or abs(sample - anchor) >= self.threshold:
                anchor = sample
                gated[joint] = sample
            else:
                gated[joint] = anchor
            self._anchor[joint] = anchor
            self._speed[joint] = speed
            self._last[joint] = sample
        return gated


# 五次插值（jerk 受限平滑）的默认参数。
#
# 这一级要解决的是"大幅度移动时抖动严重"：One Euro 的截止频率随速度线性打开
# （100 deg/s 时约 31 Hz），手抖和结构振动都会跟过去。用一条**带宽固定**的
# 三阶级联环（位置→速度→加速度→jerk）把 5~15 Hz 整段压掉，同时靠**速度
# 前馈**保住跟手性：2026-09-23 在真实拖动数据上实测，5~15 Hz 衰减 95~98%，
# 而滞后中位数只有 0.0~0.24 度（比不加这一级时的 1.5 度更紧）。
#
# 级联环按三阶 Butterworth 极点配置：特征多项式 s³ + 2ωs² + 2ω²s + ω³，
# 于是 k_p = ω/2、k_v = ω、k_a = 2ω，只有一个可调参数 ω（带宽）。ω 越大越
# 跟手、但急停时的过冲越大（ω=15 rad/s 时 150 deg/s 急停过冲约 7.6 度）。
DEFAULT_SMOOTH_BANDWIDTH_RAD_S = 15.0
# 速度、加速度、jerk 的硬上限：由实测加速度分布定的安全网（90 分位约 4700、
# 峰值约 8500 deg/s²），正常情况下由 ω 决定形状，这几个上限只在猛拉时兜底。
DEFAULT_SMOOTH_MAX_VELOCITY_DEG_S = 172.0
DEFAULT_SMOOTH_MAX_ACCELERATION_DEG_S2 = 6000.0
DEFAULT_SMOOTH_MAX_JERK_DEG_S3 = 60000.0


def _clamp(value, low, high):
    """Clamp ``value`` into ``[low, high]``."""
    return low if value < low else (high if value > high else value)


def smoother_gains(bandwidth_rad_s: float):
    """Return the (k_p, k_v, k_a) gains placed on Butterworth poles."""
    return (bandwidth_rad_s / 2.0, bandwidth_rad_s, 2.0 * bandwidth_rad_s)


class MotionSmoother:
    """
    Jerk-limited smoothing stage: the stable form of a quintic interpolation.

    Every joint keeps (position, velocity, acceleration) and each cycle walks
    one step of a cascaded loop whose poles are placed at ``-bandwidth``:
    the position error becomes a desired velocity, that becomes a desired
    acceleration, that becomes a jerk.  Because jerk is the inner-most
    command, the **acceleration is continuous** -- there are no acceleration
    jumps, which is what a quintic segment buys and what this stage is for.

    Velocity feed-forward (``velocities``, normally the One Euro derivative
    estimate) keeps a steady drag tracked without lag, so smoothing does not
    cost tracking.  What it does cost is the braking distance: when the
    reference stops abruptly the tracker overshoots by about ``v / bandwidth``
    (7.6 degrees for a 150 deg/s stop at the default 15 rad/s).

    Limits are hard: acceleration and jerk never exceed the values passed in.
    The velocity ceiling can be exceeded by a fraction of a percent (0.35% was
    measured) because the jerk limit cannot reduce an established acceleration
    within a single cycle -- a safety-net-level deviation, not a runaway.
    """

    def __init__(self,
                 bandwidth: float = DEFAULT_SMOOTH_BANDWIDTH_RAD_S,
                 max_velocity: float = DEFAULT_SMOOTH_MAX_VELOCITY_DEG_S,
                 max_acceleration: float =
                 DEFAULT_SMOOTH_MAX_ACCELERATION_DEG_S2,
                 max_jerk: float = DEFAULT_SMOOTH_MAX_JERK_DEG_S3):
        for name, value in (('bandwidth', bandwidth),
                            ('max_velocity', max_velocity),
                            ('max_acceleration', max_acceleration),
                            ('max_jerk', max_jerk)):
            if value <= 0.0:
                raise ValueError(f'{name} must be positive')
        self.bandwidth = float(bandwidth)
        self.max_velocity = float(max_velocity)
        self.max_acceleration = float(max_acceleration)
        self.max_jerk = float(max_jerk)
        self._state: Dict[int, Tuple[float, float, float]] = {}

    def reset(self, values):
        """Start every joint at rest on the given pose."""
        self._state = {
            joint: (float(value), 0.0, 0.0) for joint, value in values.items()
        }
        return dict(values)

    def update(self, positions, velocities, dt: float):
        """Advance every joint by ``dt`` seconds toward ``positions``."""
        k_p, k_v, k_a = smoother_gains(self.bandwidth)
        smoothed = {}
        for joint, goal in positions.items():
            state = self._state.get(joint)
            if state is None:
                self._state[joint] = (goal, 0.0, 0.0)
                smoothed[joint] = goal
                continue
            position, velocity, acceleration = state
            if dt <= 0.0:
                smoothed[joint] = position
                continue
            feed_forward = velocities.get(joint, 0.0)
            v_des = _clamp(k_p * (goal - position) + feed_forward,
                           -self.max_velocity, self.max_velocity)
            a_des = _clamp(k_v * (v_des - velocity),
                           -self.max_acceleration, self.max_acceleration)
            # 收紧（不是赋值）：这一步的加速不许把速度顶出上限
            a_des = min(a_des, (self.max_velocity - velocity) / dt)
            a_des = max(a_des, (-self.max_velocity - velocity) / dt)
            jerk = _clamp(k_a * (a_des - acceleration),
                          -self.max_jerk, self.max_jerk)
            acceleration += jerk * dt
            velocity += acceleration * dt
            position += velocity * dt
            self._state[joint] = (position, velocity, acceleration)
            smoothed[joint] = position
        return smoothed


class LowPassFilter:
    """
    First-order IIR smoothing of a per-joint reading: y = a*x + (1-a)*y.

    Kept as the non-default comparison path (``--filter lowpass``): its cutoff
    is fixed, so the follow phase either lags on fast drags or lets hand tremor
    through -- see :class:`OneEuroFilter`, which is the default for exactly
    that reason.

    ``a`` is derived from the measured interval rather than the configured
    period, because the control loop's real interval fluctuates and a fixed
    ``a`` would change the filter's cutoff with the loop's load:
    ``a = dt / (tau + dt)``.
    """

    def __init__(self, tau: float = DEFAULT_FILTER_TAU_S):
        if tau < 0.0:
            raise ValueError('tau must not be negative')
        self.tau = float(tau)
        self._state: Dict[int, float] = {}

    def reset(self, values):
        """Seed the filter with a reading so the first output has no jump."""
        self._state = dict(values)
        return dict(self._state)

    def update(self, values, dt: float):
        """Advance the filter by ``dt`` seconds and return the smoothed values."""
        if self.tau <= 0.0:
            alpha = 1.0
        elif dt > 0.0:
            alpha = dt / (self.tau + dt)
        else:
            # No time passed, so nothing is smoothed towards yet: hold the
            # last output and seed joints that appear for the first time.
            alpha = 0.0
        smoothed = {}
        for joint, sample in values.items():
            previous = self._state.get(joint)
            value = (sample if previous is None
                     else alpha * sample + (1.0 - alpha) * previous)
            self._state[joint] = value
            smoothed[joint] = value
        return smoothed
