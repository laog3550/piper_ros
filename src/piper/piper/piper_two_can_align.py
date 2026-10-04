#!/usr/bin/env python3
"""Align one follower to its offset master on a shared CAN bus."""

import math
import time
from argparse import ArgumentParser
from collections import Counter, defaultdict, deque
from dataclasses import dataclass
from typing import Dict, Mapping, Optional, Tuple

import can

from piper.piper_enable_status import EnableState, aggregate
from piper.piper_feedback import (
    DEFAULT_GRIPPER_EFFORT_NM,
    FEEDBACK_CAN_IDS,
    JOINT_ANGLE_IDS,
    JOINT_COUNT,
    FeedbackTracker,
    align_targets,
    clamp_gripper,
    clamp_targets,
    decode,
    decode_gripper,
    decode_joint_angles,
)
from piper.piper_two_can_identity import (
    CONFIG_CAN_ID,
    DEFAULT_ERROR_POLICY,
    DEFAULT_MAX_ERROR_RATE_HZ,
    ENABLE_CAN_ID,
    ERROR_POLICIES,
    ERROR_RATE_WINDOW_S,
    error_frame_class,
    error_frame_is_degradation,
    peak_error_rate,
    FOLLOWER_CONTROL_IDS,
    FOLLOWER_FEEDBACK_IDS,
    MASTER_CONTROL_IDS,
    MASTER_FEEDBACK_IDS,
    IdentityResult,
    evaluate_identity,
)


FOLLOWER_ANGLE_IDS = tuple(JOINT_ANGLE_IDS)
MASTER_OFFSET = 0x20
MASTER_ANGLE_IDS = tuple(can_id + MASTER_OFFSET for can_id in JOINT_ANGLE_IDS)
MOTION_CTRL_1_ID = 0x150
MOTION_CTRL_2_ID = 0x151
EMERGENCY_STOP = 0x01
EMERGENCY_STOP_RECOVER = 0x02
JOINT_CTRL_IDS = (0x155, 0x156, 0x157)
GRIPPER_CTRL_ID = 0x159
GRIPPER_FEEDBACK_ID = 0x2A8
MASTER_GRIPPER_FEEDBACK_ID = GRIPPER_FEEDBACK_ID + MASTER_OFFSET

DEFAULT_PREFLIGHT_SECONDS = 2.0
DEFAULT_MIN_HZ = 150.0
DEFAULT_CONFLICT_HZ = 300.0
DEFAULT_MAX_DELTA_DEG = 30.0
DEFAULT_MAX_GOAL_CLAMP_DEG = 1.0
DEFAULT_MIN_DURATION_S = 2.0
DEFAULT_MAX_PEAK_DEG_S = 5.0
DEFAULT_RATE_HZ = 50.0
DEFAULT_SPEED_PERCENT = 5
DEFAULT_STALE_S = 0.03
DEFAULT_MAX_SKEW_S = 0.01
DEFAULT_MASTER_DRIFT_DEG = 0.5
DEFAULT_MAX_FOLLOW_ERROR_DEG = 3.0
DEFAULT_STATIONARY_PROBE_S = 0.5
DEFAULT_STATIONARY_PROBE_QUIET_S = 0.1
DEFAULT_SETTLE_S = 1.0
DEFAULT_GOAL_TOLERANCE_DEG = 0.5
DEFAULT_GRIPPER_STALE_S = 0.05
READY_TIMEOUT_S = 1.0
MINIMUM_JERK_PEAK_FACTOR = 1.875

EXIT_OK = 0
EXIT_REFUSED = 2
EXIT_FAILED = 3


class SafetyError(RuntimeError):
    """A condition that forbids starting or continuing motion."""


@dataclass(frozen=True)
class AlignmentPlan:
    """Frozen start/goal poses and the bounded minimum-jerk duration."""

    start_deg: Mapping[int, float]
    requested_goal_deg: Mapping[int, float]
    goal_deg: Mapping[int, float]
    goal_corrections_deg: Mapping[int, float]
    deltas_deg: Mapping[int, float]
    max_delta_deg: float
    duration_s: float
    peak_deg_s: float


@dataclass(frozen=True)
class PreflightReport:
    """All observations required before an alignment may transmit."""

    identity: IdentityResult
    follower_deg: Mapping[int, float]
    master_deg: Mapping[int, float]
    enable_samples: Mapping[int, Tuple[bool, ...]]
    error_frames: int = 0
    max_error_rate_hz: float = 0.0
    peak_error_rate_hz: float = 0.0
    follower_gripper_m: Optional[float] = None
    master_gripper_m: Optional[float] = None

    @property
    def all_samples_enabled(self) -> bool:
        """Whether every joint reported and every observed sample was on."""
        return all(
            self.enable_samples.get(joint)
            and all(self.enable_samples[joint])
            for joint in range(1, JOINT_COUNT + 1)
        )

    @property
    def all_samples_disabled(self) -> bool:
        """Whether every joint reported and every observed sample was off."""
        return all(
            self.enable_samples.get(joint)
            and not any(self.enable_samples[joint])
            for joint in range(1, JOINT_COUNT + 1)
        )


class PoseTracker:
    """Build coherent six-joint snapshots from three angle frames."""

    def __init__(self, offset: int = 0):
        self.offset = offset
        self._angles: Dict[int, float] = {}
        self._frame_times: Dict[int, float] = {}
        self._versions: Dict[int, int] = {}

    def update(self, can_id: int, data, now: Optional[float] = None) -> bool:
        base_id = can_id - self.offset
        decoded = decode_joint_angles(base_id, data)
        if not decoded:
            return False
        observed = time.monotonic() if now is None else float(now)
        self._angles.update(decoded)
        self._frame_times[base_id] = observed
        self._versions[base_id] = self._versions.get(base_id, 0) + 1
        return True

    def versions(self) -> Tuple[int, ...]:
        """Return per-pair generations, independent of equal timestamps."""
        return tuple(self._versions.get(can_id, 0)
                     for can_id in JOINT_ANGLE_IDS)

    def snapshot(self, now: Optional[float] = None, *,
                 stale_s: float = DEFAULT_STALE_S,
                 max_skew_s: float = DEFAULT_MAX_SKEW_S
                 ) -> Dict[int, float]:
        current = time.monotonic() if now is None else float(now)
        missing = tuple(
            can_id for can_id in JOINT_ANGLE_IDS
            if can_id not in self._frame_times
        )
        if missing:
            ids = '、'.join(
                f'0x{can_id + self.offset:03X}' for can_id in missing)
            raise SafetyError(f'关节反馈不完整：{ids}')
        times = tuple(self._frame_times[can_id] for can_id in JOINT_ANGLE_IDS)
        oldest = min(times)
        newest = max(times)
        if current - oldest > stale_s:
            raise SafetyError(
                f'关节反馈过期：最旧帧龄 {current - oldest:.3f}s'
            )
        if newest - oldest > max_skew_s:
            raise SafetyError(
                f'六关节快照不同步：跨度 {newest - oldest:.3f}s'
            )
        if len(self._angles) != JOINT_COUNT:
            raise SafetyError('没有完整的六关节角度')
        return dict(self._angles)


def plan_alignment(start_deg: Mapping[int, float],
                   goal_deg: Mapping[int, float], *,
                   max_delta_deg: float = DEFAULT_MAX_DELTA_DEG,
                   max_goal_clamp_deg: float = DEFAULT_MAX_GOAL_CLAMP_DEG,
                   min_duration_s: float = DEFAULT_MIN_DURATION_S,
                   max_peak_deg_s: float = DEFAULT_MAX_PEAK_DEG_S,
                   ) -> AlignmentPlan:
    """Validate two poses and plan a speed-bounded minimum-jerk move."""
    expected = set(range(1, JOINT_COUNT + 1))
    if set(start_deg) != expected or set(goal_deg) != expected:
        raise ValueError('start and goal must contain joints 1..6 exactly')
    if (max_delta_deg <= 0 or max_goal_clamp_deg < 0
            or min_duration_s <= 0 or max_peak_deg_s <= 0):
        raise ValueError('alignment limits must be positive')
    if not all(math.isfinite(float(value))
               for value in (*start_deg.values(), *goal_deg.values())):
        raise ValueError('joint angles must be finite')

    _, start_limited = clamp_targets(dict(start_deg))
    if start_limited:
        details = '、'.join(
            f'j{joint}={start_deg[joint]:.3f}°'
            for joint in sorted(start_limited)
        )
        raise SafetyError(f'从臂起点超出指令限位：{details}')

    command_goal, goal_corrections = clamp_targets(dict(goal_deg))
    excessive_clamps = {
        joint: correction
        for joint, correction in goal_corrections.items()
        if abs(correction) > max_goal_clamp_deg
    }
    if excessive_clamps:
        details = '、'.join(
            f'j{joint}={goal_deg[joint]:.3f}°→{command_goal[joint]:.3f}°'
            for joint in sorted(excessive_clamps)
        )
        raise SafetyError(
            f'主臂目标超限过多（允许钳制 {max_goal_clamp_deg:.3f}°）：'
            f'{details}'
        )

    deltas = {
        joint: float(command_goal[joint]) - float(start_deg[joint])
        for joint in range(1, JOINT_COUNT + 1)
    }
    largest = max(abs(delta) for delta in deltas.values())
    if largest > max_delta_deg:
        raise SafetyError(
            f'最大姿态差 {largest:.3f}° 超过门禁 {max_delta_deg:.3f}°'
        )
    duration = max(
        min_duration_s,
        MINIMUM_JERK_PEAK_FACTOR * largest / max_peak_deg_s,
    )
    peak = (MINIMUM_JERK_PEAK_FACTOR * largest / duration
            if duration > 0 else 0.0)
    return AlignmentPlan(
        start_deg=dict(start_deg),
        requested_goal_deg=dict(goal_deg),
        goal_deg=command_goal,
        goal_corrections_deg=goal_corrections,
        deltas_deg=deltas,
        max_delta_deg=largest,
        duration_s=duration,
        peak_deg_s=peak,
    )


def encode_motion_ctrl_1(emergency_stop: int = EMERGENCY_STOP) -> bytes:
    """
    Encode 0x150: emergency stop 0x01, recovery 0x02 (SDK MotionCtrl_1).

    现场只验收过 0x471 失能，这两个载荷的实测行为尚未单独验收，所以默认不发送。
    """
    if emergency_stop not in (EMERGENCY_STOP, EMERGENCY_STOP_RECOVER):
        raise ValueError('emergency_stop must be 0x01 or 0x02')
    return bytes((emergency_stop, 0, 0, 0, 0, 0, 0, 0))


def encode_motion_ctrl_2(speed_percent: int, standby: bool = False) -> bytes:
    """Encode 0x151 without depending on the four-CAN SDK interface."""
    if not 1 <= int(speed_percent) <= 100:
        raise ValueError('speed_percent must be in 1..100')
    ctrl_mode = 0x00 if standby else 0x01
    return bytes((ctrl_mode, 0x01, int(speed_percent), 0, 0, 0, 0, 0))


def encode_joint_targets(
        targets_deg: Mapping[int, float]
        ) -> Tuple[Tuple[int, bytes], ...]:
    """Encode 0x155..0x157 as signed big-endian millidegrees."""
    expected = set(range(1, JOINT_COUNT + 1))
    if set(targets_deg) != expected:
        raise ValueError('targets must contain joints 1..6 exactly')
    clamped, limited = clamp_targets(dict(targets_deg))
    if limited:
        joints = '、'.join(f'j{joint}' for joint in sorted(limited))
        raise SafetyError(f'目标超出指令限位：{joints}')
    frames = []
    for can_id, pair in zip(JOINT_CTRL_IDS, ((1, 2), (3, 4), (5, 6))):
        payload = b''.join(
            int(round(clamped[joint] * 1000.0)).to_bytes(
                4, 'big', signed=True)
            for joint in pair
        )
        frames.append((can_id, payload))
    return tuple(frames)


def socket_filters():
    """Return the exact standard-ID filters this tool listens on."""
    ids = set(
        FOLLOWER_FEEDBACK_IDS + MASTER_FEEDBACK_IDS
        + FOLLOWER_CONTROL_IDS + MASTER_CONTROL_IDS
        + FEEDBACK_CAN_IDS + (CONFIG_CAN_ID, ENABLE_CAN_ID)
    )
    return [
        {'can_id': can_id, 'can_mask': 0x7FF, 'extended': False}
        for can_id in sorted(ids)
    ]


def _open_bus(port: str):
    return can.Bus(
        interface='socketcan', channel=port, receive_own_messages=False,
        can_filters=socket_filters(),
    )


def capture_preflight(port: str, duration_s: float, *,
                      min_hz: float = DEFAULT_MIN_HZ,
                      conflict_hz: float = DEFAULT_CONFLICT_HZ,
                      stale_s: float = DEFAULT_STALE_S,
                      max_skew_s: float = DEFAULT_MAX_SKEW_S,
                      side: str = 'unknown',
                      error_policy: str = DEFAULT_ERROR_POLICY,
                      max_error_rate_hz: float = DEFAULT_MAX_ERROR_RATE_HZ,
                      gripper_stale_s: float = DEFAULT_GRIPPER_STALE_S
                      ) -> PreflightReport:
    """Observe identity, poses and every enable sample without transmitting."""
    if duration_s <= 0:
        raise ValueError('duration_s must be positive')
    if error_policy not in ERROR_POLICIES:
        raise ValueError(
            'error_policy must be one of ' + '/'.join(ERROR_POLICIES))
    counts: Counter = Counter()
    master_modes: Counter = Counter()
    error_times = []
    degradation_frames = 0
    follower = PoseTracker()
    master = PoseTracker(MASTER_OFFSET)
    enable_samples = defaultdict(list)
    follower_gripper: Optional[Tuple[float, float]] = None
    master_gripper: Optional[Tuple[float, float]] = None
    bus = _open_bus(port)
    started = time.monotonic()
    deadline = started + duration_s
    try:
        while time.monotonic() < deadline:
            frame = bus.recv(timeout=max(
                0.0, min(0.1, deadline - time.monotonic())))
            if frame is None:
                continue
            now = time.monotonic()
            if frame.is_error_frame:
                error_times.append(now)
                if error_frame_is_degradation(frame):
                    degradation_frames += 1
                continue
            if frame.is_extended_id:
                continue
            can_id = frame.arbitration_id
            counts[can_id] += 1
            follower.update(can_id, frame.data, now)
            master.update(can_id, frame.data, now)
            feedback = decode(can_id, frame.data)
            if feedback is not None:
                enable_samples[feedback.joint].append(feedback.enabled)
            if can_id == MASTER_FEEDBACK_IDS[0] and frame.data:
                master_modes[frame.data[0]] += 1
            opening = decode_gripper(can_id, frame.data)
            if opening is not None:
                follower_gripper = (opening, now)
            else:
                opening = decode_gripper(can_id - MASTER_OFFSET, frame.data)
                if opening is not None:
                    master_gripper = (opening, now)
    finally:
        elapsed = time.monotonic() - started
        bus.shutdown()

    average_rate = (len(error_times) / elapsed) if elapsed > 0 else 0.0
    peak_rate = peak_error_rate(error_times, 1.0)
    # 在 recoverable 策略下，孤立错误帧不参与身份判定，改由持续速率上限单独判定；
    # 平均速率超过上限时仍把真实计数交给 evaluate_identity，让它判定 CAN_ERROR。
    gated_errors = len(error_times)
    if (error_policy == 'recoverable' and not degradation_frames
            and average_rate <= max_error_rate_hz):
        gated_errors = 0
    identity = evaluate_identity(
        counts, elapsed, side=side, port=port,
        min_hz=min_hz, conflict_hz=conflict_hz,
        error_frames=gated_errors, master_modes=master_modes,
    )
    now = time.monotonic()

    def _fresh_gripper(stored):
        if stored is None:
            return None
        value, seen = stored
        if now - seen > gripper_stale_s:
            return None
        return value

    def _pose(tracker) -> Dict[int, float]:
        # 某一侧没有可用姿态时不给报告，而不是抛异常：身份门禁已经把
        # “反馈窗口缺失或频率过低”作为拒绝理由，报告能打印出来更有用。
        try:
            return tracker.snapshot(
                now, stale_s=stale_s, max_skew_s=max_skew_s)
        except SafetyError:
            return {}

    return PreflightReport(
        identity=identity,
        follower_deg=_pose(follower),
        master_deg=_pose(master),
        enable_samples={
            joint: tuple(samples)
            for joint, samples in enable_samples.items()
        },
        error_frames=len(error_times),
        max_error_rate_hz=average_rate,
        peak_error_rate_hz=peak_rate,
        follower_gripper_m=_fresh_gripper(follower_gripper),
        master_gripper_m=_fresh_gripper(master_gripper),
    )


class RuntimeMonitor:
    """Reject stale feedback, disable events and competing controllers."""

    def __init__(self, *, stale_s: float, max_skew_s: float,
                 error_policy: str = DEFAULT_ERROR_POLICY,
                 max_error_rate_hz: float = DEFAULT_MAX_ERROR_RATE_HZ,
                 gripper_stale_s: float = DEFAULT_GRIPPER_STALE_S):
        if error_policy not in ERROR_POLICIES:
            raise ValueError(
                'error_policy must be one of ' + '/'.join(ERROR_POLICIES))
        if max_error_rate_hz <= 0:
            raise ValueError('max_error_rate_hz must be positive')
        self.stale_s = stale_s
        self.max_skew_s = max_skew_s
        self.error_policy = error_policy
        self.max_error_rate_hz = max_error_rate_hz
        self.gripper_stale_s = gripper_stale_s
        self.follower = PoseTracker()
        self.master = PoseTracker(MASTER_OFFSET)
        self.enable = FeedbackTracker(timeout=max(stale_s, 0.1))
        self.error_frames = 0
        self.error_classes: Counter = Counter()
        self._error_times = deque()
        self._follower_gripper: Optional[Tuple[float, float]] = None
        self._master_gripper: Optional[Tuple[float, float]] = None

    def error_rate_hz(self, now: Optional[float] = None) -> float:
        """Return the sustained error-frame rate over the rolling window."""
        observed = time.monotonic() if now is None else float(now)
        self._prune_errors(observed)
        if not self._error_times:
            return 0.0
        # Fixed rolling-window budget: bursts must not reset the denominator.
        return len(self._error_times) / ERROR_RATE_WINDOW_S

    def peak_error_rate_hz(self) -> float:
        """Return the highest error-frame count inside any one second."""
        return peak_error_rate(self._error_times, 1.0)

    def _prune_errors(self, now: float) -> None:
        limit = now - ERROR_RATE_WINDOW_S
        while self._error_times and self._error_times[0] < limit:
            self._error_times.popleft()

    def _observe_error(self, frame, now: float) -> None:
        """Count one error frame and decide whether it forbids motion."""
        self.error_frames += 1
        self._error_times.append(now)
        self._prune_errors(now)
        description = error_frame_class(frame)
        if error_frame_is_degradation(frame):
            raise SafetyError(f'运行中 CAN 状态退化：{description}')
        if self.error_policy == 'strict':
            raise SafetyError(f'运行中收到 CAN 错误帧：{description}')
        rate = self.error_rate_hz(now)
        if rate > self.max_error_rate_hz:
            raise SafetyError(
                f'CAN 错误帧持续速率 {rate:.1f} Hz 超过上限 '
                f'{self.max_error_rate_hz:.1f} Hz（{description}）'
            )

    def process(self, frame, now: Optional[float] = None,
                expect_enabled: bool = True) -> None:
        """
        Check one frame; ``expect_enabled=False`` allows disabled samples.

        The disable read-back window expects every joint to report disabled,
        so a disabled sample is data there rather than a fault.  Everything
        else — error frames, external control frames, 0x470, 0x471 — stays
        fatal in both modes.
        """
        observed = time.monotonic() if now is None else float(now)
        if frame.is_error_frame:
            self._observe_error(frame, observed)
            return
        if frame.is_extended_id:
            return
        can_id = frame.arbitration_id
        if can_id == CONFIG_CAN_ID:
            raise SafetyError('运行中观察到 0x470 配置帧')
        if can_id == ENABLE_CAN_ID:
            raise SafetyError('运行中观察到外部 0x471 使能/失能帧')
        if can_id in FOLLOWER_CONTROL_IDS or can_id in MASTER_CONTROL_IDS:
            raise SafetyError(f'检测到外部控制帧 0x{can_id:03X}')
        self.follower.update(can_id, frame.data, observed)
        self.master.update(can_id, frame.data, observed)
        feedback = self.enable.update(can_id, frame.data, observed)
        if (feedback is not None and not feedback.enabled
                and expect_enabled):
            raise SafetyError(f'j{feedback.joint} 报告 disabled')
        self._observe_gripper(can_id, frame.data, observed)

    def _observe_gripper(self, can_id: int, data, now: float) -> None:
        """Track both gripper openings; a stale one is reported as None."""
        opening = decode_gripper(can_id, data)
        if opening is not None:
            self._follower_gripper = (opening, now)
            return
        opening = decode_gripper(can_id - MASTER_OFFSET, data)
        if opening is not None:
            self._master_gripper = (opening, now)

    def _fresh_gripper(self, stored, now: float) -> Optional[float]:
        if stored is None:
            return None
        value, seen = stored
        if now - seen > self.gripper_stale_s:
            return None
        return value

    def grippers(self, now: Optional[float] = None
                 ) -> Tuple[Optional[float], Optional[float]]:
        """Return (follower, master) openings in metres, or None when stale."""
        observed = time.monotonic() if now is None else float(now)
        return (
            self._fresh_gripper(self._follower_gripper, observed),
            self._fresh_gripper(self._master_gripper, observed),
        )

    def snapshots(self, now: Optional[float] = None):
        observed = time.monotonic() if now is None else float(now)
        enable_state = aggregate(self.enable.observations(observed))
        if enable_state is not EnableState.ENABLED:
            raise SafetyError('六关节没有持续报告 enabled')
        follower = self.follower.snapshot(
            observed, stale_s=self.stale_s, max_skew_s=self.max_skew_s)
        master = self.master.snapshot(
            observed, stale_s=self.stale_s, max_skew_s=self.max_skew_s)
        return follower, master


def _send(bus, can_id: int, payload: bytes) -> None:
    bus.send(can.Message(
        arbitration_id=can_id, data=payload, is_extended_id=False,
    ))


def encode_gripper_target(open_m: float,
                          effort_nm: float = DEFAULT_GRIPPER_EFFORT_NM
                          ) -> bytes:
    """Encode 0x159: opening in 0.001 mm, effort in 0.001 N·m, enabled."""
    counts = int(round(clamp_gripper(open_m) * 1e6))
    effort = int(round(max(0.0, min(float(effort_nm), 5.0)) * 1000.0))
    return (counts.to_bytes(4, 'big', signed=True)
            + effort.to_bytes(2, 'big')
            + bytes((0x01, 0x00)))


def send_target(bus, targets_deg: Mapping[int, float], speed_percent: int,
                gripper_open_m: Optional[float] = None,
                gripper_effort_nm: float = DEFAULT_GRIPPER_EFFORT_NM) -> None:
    """Send one standard-address follower command set and optional gripper."""
    _send(bus, MOTION_CTRL_2_ID, encode_motion_ctrl_2(speed_percent))
    for can_id, payload in encode_joint_targets(targets_deg):
        _send(bus, can_id, payload)
    if gripper_open_m is not None:
        _send(bus, GRIPPER_CTRL_ID,
              encode_gripper_target(gripper_open_m, gripper_effort_nm))


def send_standby(bus, speed_percent: int) -> None:
    """Stop follower target streaming without touching master control IDs."""
    _send(bus, MOTION_CTRL_2_ID,
          encode_motion_ctrl_2(speed_percent, standby=True))


def _wait_runtime_ready(bus, monitor: RuntimeMonitor):
    deadline = time.monotonic() + READY_TIMEOUT_S
    while time.monotonic() < deadline:
        frame = bus.recv(timeout=max(
            0.0, min(0.02, deadline - time.monotonic())))
        if frame is not None:
            monitor.process(frame)
        try:
            return monitor.snapshots()
        except SafetyError:
            continue
    return monitor.snapshots()


def _drain_until(bus, monitor: RuntimeMonitor, deadline: float) -> None:
    """Process frames until a deadline while retaining safety checks."""
    while time.monotonic() < deadline:
        frame = bus.recv(timeout=max(
            0.0, min(0.005, deadline - time.monotonic())))
        if frame is not None:
            monitor.process(frame)


@dataclass(frozen=True)
class GripperRamp:
    """Gripper openings (metres) at the two ends of one alignment move."""

    start_m: float
    goal_m: float
    effort_nm: float = DEFAULT_GRIPPER_EFFORT_NM


def _validate_run_params(rate_hz: float, stationary_probe_s: float,
                         master_drift_deg: float,
                         max_follow_error_deg: float, settle_s: float,
                         goal_tolerance_deg: float,
                         speed_percent: int) -> None:
    if rate_hz <= 0 or master_drift_deg <= 0:
        raise ValueError('rate and master drift threshold must be positive')
    if (max_follow_error_deg <= 0 or stationary_probe_s <= 0
            or settle_s < 0 or goal_tolerance_deg <= 0):
        raise ValueError('tracking thresholds must be positive')
    encode_motion_ctrl_2(speed_percent)


def run_alignment(bus, monitor: RuntimeMonitor, plan: AlignmentPlan, *,
                  rate_hz: float, speed_percent: int,
                  master_drift_deg: float, max_follow_error_deg: float,
                  settle_s: float, goal_tolerance_deg: float,
                  stationary_probe_s: float = DEFAULT_STATIONARY_PROBE_S,
                  gripper: Optional[GripperRamp] = None,
                  standby_on_finish: bool = True,
                  watch_master: bool = True,
                  check_stop=None) -> AlignmentPlan:
    """
    Run one guarded alignment on an already open bus and live monitor.

    ``standby_on_finish`` is what the standalone align tool wants: stop the
    follower target stream as soon as the move ends or aborts.  A teleop
    session passes False so it can keep commanding the aligned pose.
    """
    _validate_run_params(rate_hz, stationary_probe_s, master_drift_deg,
                         max_follow_error_deg, settle_s, goal_tolerance_deg,
                         speed_percent)

    def gripper_at(progress: float) -> Optional[float]:
        if gripper is None:
            return None
        return clamp_gripper(
            align_targets({7: gripper.start_m}, {7: gripper.goal_m},
                          progress)[7])

    sent_any = False
    try:
        period = 1.0 / rate_hz

        # Hold the follower at its measured start pose while exercising the
        # exact command rate used by the trajectory.  A shared-bus collision
        # that only appears once command traffic is added is therefore caught
        # before the target begins to move.
        probe_started = time.monotonic()
        probe_deadline = probe_started + stationary_probe_s
        next_command = probe_started
        while time.monotonic() < probe_deadline:
            if check_stop is not None:
                check_stop()
            _drain_until(bus, monitor, min(next_command, probe_deadline))
            now = time.monotonic()
            if now >= probe_deadline:
                break
            if now < next_command:
                continue
            follower, current_master = monitor.snapshots(now)
            drift = max(
                abs(current_master[joint] - plan.requested_goal_deg[joint])
                for joint in range(1, JOINT_COUNT + 1)
            )
            if watch_master and drift > master_drift_deg:
                raise SafetyError(
                    f'静止探针期间主臂移动 {drift:.3f}°，超过 '
                    f'{master_drift_deg:.3f}°'
                )
            hold_error = max(
                abs(follower[joint] - plan.start_deg[joint])
                for joint in range(1, JOINT_COUNT + 1)
            )
            if hold_error > max_follow_error_deg:
                raise SafetyError(
                    f'静止探针期间从臂偏移 {hold_error:.3f}°，超过 '
                    f'{max_follow_error_deg:.3f}°'
                )
            sent_any = True
            send_target(bus, plan.start_deg, speed_percent,
                        gripper_at(0.0), gripper.effort_nm if gripper else 0.0)
            next_command = max(next_command + period, now + period * 0.25)
        _drain_until(
            bus, monitor,
            time.monotonic() + DEFAULT_STATIONARY_PROBE_QUIET_S,
        )

        started = time.monotonic()
        final_deadline = started + plan.duration_s + settle_s
        next_command = started
        while True:
            if check_stop is not None:
                check_stop()
            now = time.monotonic()
            if now >= final_deadline:
                break
            _drain_until(bus, monitor, min(next_command, final_deadline))
            now = time.monotonic()
            if now < next_command:
                continue
            follower, current_master = monitor.snapshots(now)
            drift = max(
                abs(current_master[joint] - plan.requested_goal_deg[joint])
                for joint in range(1, JOINT_COUNT + 1)
            )
            if watch_master and drift > master_drift_deg:
                raise SafetyError(
                    f'主臂在对齐期间移动 {drift:.3f}°，超过 '
                    f'{master_drift_deg:.3f}°'
                )
            progress = min(1.0, (now - started) / plan.duration_s)
            targets = align_targets(
                plan.start_deg, plan.goal_deg, progress)
            follow_error = max(
                abs(follower[joint] - targets[joint])
                for joint in range(1, JOINT_COUNT + 1)
            )
            if follow_error > max_follow_error_deg:
                raise SafetyError(
                    f'从臂跟踪误差 {follow_error:.3f}°，超过 '
                    f'{max_follow_error_deg:.3f}°'
                )
            sent_any = True
            send_target(bus, targets, speed_percent, gripper_at(progress),
                        gripper.effort_nm if gripper else 0.0)
            next_command = max(next_command + period, now + period * 0.25)

        _drain_until(bus, monitor, time.monotonic() + min(0.1, period * 2))
        follower, current_master = monitor.snapshots()
        drift = max(
            abs(current_master[joint] - plan.requested_goal_deg[joint])
            for joint in range(1, JOINT_COUNT + 1)
        )
        error = max(
            abs(follower[joint] - plan.goal_deg[joint])
            for joint in range(1, JOINT_COUNT + 1)
        )
        if watch_master and drift > master_drift_deg:
            raise SafetyError(f'结束时主臂漂移 {drift:.3f}°')
        if error > goal_tolerance_deg:
            raise SafetyError(
                f'结束时从臂误差 {error:.3f}°，超过 '
                f'{goal_tolerance_deg:.3f}°'
            )
        return plan
    finally:
        if sent_any and standby_on_finish:
            try:
                send_standby(bus, speed_percent)
            except can.CanError:
                pass


def execute_alignment(port: str, *, max_delta_deg: float,
                      max_goal_clamp_deg: float,
                      min_duration_s: float, max_peak_deg_s: float,
                      rate_hz: float, speed_percent: int,
                      stale_s: float, max_skew_s: float,
                      master_drift_deg: float,
                      max_follow_error_deg: float,
                      settle_s: float, goal_tolerance_deg: float,
                      stationary_probe_s: float = DEFAULT_STATIONARY_PROBE_S,
                      error_policy: str = DEFAULT_ERROR_POLICY,
                      max_error_rate_hz: float = DEFAULT_MAX_ERROR_RATE_HZ,
                      gripper: Optional[GripperRamp] = None,
                      target_deg: Optional[Mapping[int, float]] = None,
                      ) -> AlignmentPlan:
    """Run one guarded alignment and always return the follower to standby."""
    _validate_run_params(rate_hz, stationary_probe_s, master_drift_deg,
                         max_follow_error_deg, settle_s, goal_tolerance_deg,
                         speed_percent)

    bus = _open_bus(port)
    monitor = RuntimeMonitor(
        stale_s=stale_s, max_skew_s=max_skew_s,
        error_policy=error_policy, max_error_rate_hz=max_error_rate_hz,
    )
    try:
        follower, master = _wait_runtime_ready(bus, monitor)
        plan = plan_alignment(
            follower, master if target_deg is None else target_deg,
            max_delta_deg=max_delta_deg,
            max_goal_clamp_deg=max_goal_clamp_deg,
            min_duration_s=min_duration_s,
            max_peak_deg_s=max_peak_deg_s,
        )
        return run_alignment(
            bus, monitor, plan,
            rate_hz=rate_hz, speed_percent=speed_percent,
            master_drift_deg=master_drift_deg,
            max_follow_error_deg=max_follow_error_deg,
            settle_s=settle_s, goal_tolerance_deg=goal_tolerance_deg,
            stationary_probe_s=stationary_probe_s,
            gripper=gripper,
            watch_master=target_deg is None,
        )
    finally:
        bus.shutdown()


def print_plan(plan: AlignmentPlan, goal_label: str = '主臂读数') -> None:
    print('两 CAN 自动对齐计划（单位：度）：')
    print(f'  {"关节":<6}{"从臂起点":>12}{goal_label:>12}'
          f'{"对齐目标":>12}{"位移":>12}')
    for joint in range(1, JOINT_COUNT + 1):
        print(
            f'  j{joint:<5}{plan.start_deg[joint]:>12.3f}'
            f'{plan.requested_goal_deg[joint]:>12.3f}'
            f'{plan.goal_deg[joint]:>12.3f}'
            f'{plan.deltas_deg[joint]:>+12.3f}'
        )
    if plan.goal_corrections_deg:
        print('  目标限幅：' + '、'.join(
            f'j{joint} {plan.requested_goal_deg[joint]:.3f}°→'
            f'{plan.goal_deg[joint]:.3f}°'
            for joint in sorted(plan.goal_corrections_deg)
        ))
    print(
        f'  最大位移 {plan.max_delta_deg:.3f}°；'
        f'时长 {plan.duration_s:.3f}s；'
        f'minimum-jerk 峰值 {plan.peak_deg_s:.3f}°/s'
    )


def _parser() -> ArgumentParser:
    parser = ArgumentParser(description=__doc__)
    parser.add_argument('--side', choices=('left', 'right'), default='left')
    parser.add_argument('--port', help='SocketCAN 接口（默认 can_<side>）')
    parser.add_argument('--return-to-deg', type=float, nargs=6,
                        metavar=('J1', 'J2', 'J3', 'J4', 'J5', 'J6'),
                        help='返回已记录的从臂姿态（度）；目标固定，不跟随主臂')
    parser.add_argument('--preflight-seconds', type=float,
                        default=DEFAULT_PREFLIGHT_SECONDS)
    parser.add_argument('--max-delta-deg', type=float,
                        default=DEFAULT_MAX_DELTA_DEG)
    parser.add_argument('--max-goal-clamp-deg', type=float,
                        default=DEFAULT_MAX_GOAL_CLAMP_DEG)
    parser.add_argument('--min-duration', type=float,
                        default=DEFAULT_MIN_DURATION_S)
    parser.add_argument('--max-peak-speed', type=float,
                        default=DEFAULT_MAX_PEAK_DEG_S)
    parser.add_argument('--rate', type=float, default=DEFAULT_RATE_HZ)
    parser.add_argument('--speed-percent', type=int,
                        default=DEFAULT_SPEED_PERCENT)
    parser.add_argument('--stale-seconds', type=float, default=DEFAULT_STALE_S)
    parser.add_argument('--max-skew-seconds', type=float,
                        default=DEFAULT_MAX_SKEW_S)
    parser.add_argument('--master-drift-deg', type=float,
                        default=DEFAULT_MASTER_DRIFT_DEG)
    parser.add_argument('--max-follow-error-deg', type=float,
                        default=DEFAULT_MAX_FOLLOW_ERROR_DEG)
    parser.add_argument('--settle-seconds', type=float,
                        default=DEFAULT_SETTLE_S)
    parser.add_argument('--goal-tolerance-deg', type=float,
                        default=DEFAULT_GOAL_TOLERANCE_DEG)
    parser.add_argument('--error-policy', choices=ERROR_POLICIES,
                        default=DEFAULT_ERROR_POLICY,
                        help='recoverable：容忍孤立错误帧，只在状态退化或持续'
                             '速率超限时中止（默认）；strict：任何错误帧都中止')
    parser.add_argument('--max-error-rate-hz', type=float,
                        default=DEFAULT_MAX_ERROR_RATE_HZ,
                        help='recoverable 策略下的错误帧速率上限（默认 %(default)s）')
    parser.add_argument('--send', action='store_true',
                        help='真正发送从臂标准地址控制帧；默认只干跑')
    parser.add_argument('--workspace-clear', action='store_true',
                        help='确认人员和线缆已退出运动空间；--send 时必需')
    return parser


def print_preflight(report: PreflightReport) -> None:
    """Print the read-only observations that gate a start."""
    if report.all_samples_enabled:
        enable_note = '窗口内全部 samples 都是 enabled'
    elif report.all_samples_disabled:
        enable_note = '窗口内全部 samples 都是 disabled'
    else:
        enable_note = '窗口内 samples 不一致或缺失（PARTIAL）'
    print(f'  六关节使能样本：{enable_note}')
    print(f'  CAN 错误帧：{report.error_frames} 个，平均 '
          f'{report.max_error_rate_hz:.2f} Hz，单秒峰值 '
          f'{report.peak_error_rate_hz:.1f} Hz')
    if report.follower_gripper_m is not None:
        follower_mm = report.follower_gripper_m * 1000.0
        if report.master_gripper_m is None:
            print(f'  夹爪开口：从臂 {follower_mm:.2f}mm')
        else:
            print(f'  夹爪开口：从臂 {follower_mm:.2f}mm，'
                  f'主臂 {report.master_gripper_m * 1000.0:.2f}mm')


def main(args=None) -> int:
    options = _parser().parse_args(args)
    port = options.port or f'can_{options.side}'
    target = (dict(enumerate(options.return_to_deg, 1))
              if options.return_to_deg is not None else None)
    print(f'监听 {options.side} / {port}，执行 '
          f'{options.preflight_seconds:.1f}s 只读门禁'
          f'（错误帧策略 {options.error_policy}）。')
    try:
        report = capture_preflight(
            port, options.preflight_seconds, side=options.side,
            stale_s=options.stale_seconds,
            max_skew_s=options.max_skew_seconds,
            error_policy=options.error_policy,
            max_error_rate_hz=options.max_error_rate_hz,
        )
        print_preflight(report)
        if not report.identity.confirmed:
            raise SafetyError(
                f'身份门禁未通过：{report.identity.status.value}'
            )
        plan = plan_alignment(
            report.follower_deg,
            report.master_deg if target is None else target,
            max_delta_deg=options.max_delta_deg,
            max_goal_clamp_deg=options.max_goal_clamp_deg,
            min_duration_s=options.min_duration,
            max_peak_deg_s=options.max_peak_speed,
        )
        print_plan(plan, goal_label='回位目标' if target else '主臂读数')
        if target is not None:
            print('返回模式：上表目标来自已记录姿态；仅从臂运动，不跟随主臂。')
        if not options.send:
            print('干跑完成：没有发送任何 CAN 帧。')
            if not report.all_samples_enabled:
                print('提示：当前不是“所有低速样本均 enabled”；真正发送会拒绝。')
            return EXIT_OK
        if not options.workspace_clear:
            raise SafetyError('--send 必须同时给出 --workspace-clear')
        if not report.all_samples_enabled:
            raise SafetyError('使能门禁未通过：并非所有低速样本均为 enabled')
        print(
            '开始发送：只使用从臂标准 0x151/0x155～0x157；先执行 '
            f'{DEFAULT_STATIONARY_PROBE_S:.1f}s 静止目标发帧探针。'
        )
        completed = execute_alignment(
            port,
            max_delta_deg=options.max_delta_deg,
            max_goal_clamp_deg=options.max_goal_clamp_deg,
            min_duration_s=options.min_duration,
            max_peak_deg_s=options.max_peak_speed,
            rate_hz=options.rate,
            speed_percent=options.speed_percent,
            stale_s=options.stale_seconds,
            max_skew_s=options.max_skew_seconds,
            master_drift_deg=options.master_drift_deg,
            max_follow_error_deg=options.max_follow_error_deg,
            settle_s=options.settle_seconds,
            goal_tolerance_deg=options.goal_tolerance_deg,
            error_policy=options.error_policy,
            max_error_rate_hz=options.max_error_rate_hz,
            target_deg=target,
        )
        action = '回位' if target is not None else '对齐'
        print(
            f'{action}完成：最大位移 {completed.max_delta_deg:.3f}°，'
            '从臂已切回待机；程序未发送 0x170、0x171 或 0x471。'
        )
        return EXIT_OK
    except (ValueError, SafetyError, OSError, can.CanError) as exc:
        print(f'拒绝/中止：{exc}')
        return EXIT_REFUSED if isinstance(exc, (ValueError, SafetyError)) \
            else EXIT_FAILED


if __name__ == '__main__':
    raise SystemExit(main())
