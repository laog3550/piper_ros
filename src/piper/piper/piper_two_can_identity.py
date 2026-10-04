#!/usr/bin/env python3
"""Confirm master/follower CAN address identities without transmitting."""

import time
from argparse import ArgumentParser
from collections import Counter
from dataclasses import dataclass
from enum import Enum
from typing import Mapping, Optional, Tuple

import can


FOLLOWER_FEEDBACK_IDS = tuple(range(0x2A1, 0x2A9))
MASTER_FEEDBACK_IDS = tuple(range(0x2C1, 0x2C9))
FOLLOWER_CONTROL_IDS = tuple(range(0x150, 0x160))
MASTER_CONTROL_IDS = tuple(range(0x170, 0x180))
CONFIG_CAN_ID = 0x470
ENABLE_CAN_ID = 0x471
LINKAGE_TEACHING_MODE = 0x06
OBSERVED_CAN_IDS = (
    FOLLOWER_FEEDBACK_IDS
    + MASTER_FEEDBACK_IDS
    + FOLLOWER_CONTROL_IDS
    + MASTER_CONTROL_IDS
    + (CONFIG_CAN_ID, ENABLE_CAN_ID)
)

DEFAULT_DURATION_S = 2.0
DEFAULT_MIN_HZ = 150.0

# CAN 错误帧策略（2026-09-30 现场决策：默认放开为 recoverable）。
#   recoverable —— 容忍孤立错误帧，只在“控制器状态退化”或“持续速率超限”时判失败；
#   strict      —— 任何错误帧都判失败。
# 现场实测：两台臂全部失能、上位机不发任何帧时，can_left 仍有 0.4～10 Hz 的孤立
# 错误帧（单秒峰值见过 47 个），因此 strict 无法稳定通过；而 bus-off、错误被动、
# 总线错误在任何策略下都直接判失败。
ERROR_POLICIES = ('strict', 'recoverable')
DEFAULT_ERROR_POLICY = 'recoverable'
DEFAULT_MAX_ERROR_RATE_HZ = 20.0
ERROR_RATE_WINDOW_S = 5.0

# SocketCAN 的 CAN_ERR_* 错误类别位；python-can 已经把 CAN_ERR_FLAG 剥掉，
# 所以 frame.arbitration_id 就是这些类别位的组合（现场实测为 0x4 和 0xC）。
CAN_ERR_CRTL = 0x04
CAN_ERR_PROT = 0x08
CAN_ERR_BUSOFF = 0x40
CAN_ERR_BUSERROR = 0x80
CTRL_RX_PASSIVE = 0x10
CTRL_TX_PASSIVE = 0x20
CTRL_ACTIVE = 0x40  # linux/can/error.h: recovered to error-active, not bus-off

ERROR_CLASS_NAMES = (
    (CAN_ERR_BUSOFF, 'bus-off'),
    (CAN_ERR_BUSERROR, '总线错误'),
    (CAN_ERR_PROT, '协议违规'),
    (CAN_ERR_CRTL, '控制器问题'),
)


def error_frame_is_degradation(frame) -> bool:
    """
    Report whether one CAN error frame means controller state degradation.

    Only these frames justify treating the bus as unusable on their own:
    bus-off, bus error and the error-passive/error-active transitions that
    SocketCAN reports through the controller-status byte.  Isolated
    bit-stuffing or form errors are retransmitted by the CAN controller and
    therefore do not belong in this category.
    """
    classes = int(frame.arbitration_id)
    if classes & (CAN_ERR_BUSOFF | CAN_ERR_BUSERROR):
        return True
    if classes & CAN_ERR_CRTL and len(frame.data) > 1:
        status = int(frame.data[1])
        degraded = CTRL_RX_PASSIVE | CTRL_TX_PASSIVE
        return bool(status & degraded)
    return False


def error_frame_class(frame) -> str:
    """Return a short Chinese description of one error frame's class bits."""
    classes = int(frame.arbitration_id)
    names = [name for bit, name in ERROR_CLASS_NAMES if classes & bit]
    return '、'.join(names) or f'错误类别 0x{classes:X}'


def peak_error_rate(error_times, window_s: float = 1.0) -> float:
    """Return the highest error-frame rate inside any window of a capture."""
    if not error_times:
        return 0.0
    peak = 0
    start = 0
    for index, moment in enumerate(error_times):
        while moment - error_times[start] > window_s:
            start += 1
        peak = max(peak, index - start + 1)
    return peak / window_s


# A single arm nominally reports each high-speed feedback ID at about 200 Hz.
# 300 Hz cleanly separates it from two arms colliding at about 400 Hz while
# leaving margin for scheduler and measurement jitter.
DEFAULT_CONFLICT_HZ = 300.0

EXIT_OK = 0
EXIT_NOT_CONFIRMED = 2
EXIT_FAILED = 3


class IdentityStatus(str, Enum):
    """Safety-gate result for one side's shared CAN bus."""

    CONFIRMED = 'CONFIRMED'
    FEEDBACK_INCOMPLETE = 'FEEDBACK_INCOMPLETE'
    ADDRESS_CONFLICT = 'ADDRESS_CONFLICT'
    CONTROL_PRESENT = 'CONTROL_PRESENT'
    CONFIG_CHANGED = 'CONFIG_CHANGED'
    UNSAFE_MASTER_ROLE = 'UNSAFE_MASTER_ROLE'
    CAN_ERROR = 'CAN_ERROR'


@dataclass(frozen=True)
class FeedbackWindow:
    """Per-ID rates and anomalies for one arm feedback address window."""

    role: str
    can_ids: Tuple[int, ...]
    rates_hz: Tuple[Tuple[int, float], ...]
    missing_or_slow: Tuple[int, ...]
    conflicts: Tuple[int, ...]

    @property
    def complete(self) -> bool:
        """Whether every ID is present at a plausible single-arm rate."""
        return not self.missing_or_slow and not self.conflicts

    @property
    def minimum_hz(self) -> float:
        """Return the slowest expected ID's observed rate."""
        return min(rate for _, rate in self.rates_hz)

    @property
    def maximum_hz(self) -> float:
        """Return the fastest expected ID's observed rate."""
        return max(rate for _, rate in self.rates_hz)


@dataclass(frozen=True)
class IdentityResult:
    """Read-only identity verdict for a master/follower pair."""

    side: str
    port: str
    duration_s: float
    follower: FeedbackWindow
    master: FeedbackWindow
    command_ids: Tuple[int, ...]
    master_modes: Tuple[Tuple[int, int], ...]
    config_seen: bool
    enable_seen: bool
    error_frames: int
    status: IdentityStatus
    reasons: Tuple[str, ...]

    @property
    def confirmed(self) -> bool:
        """Whether this bus is safe to pass the identity preflight gate."""
        return self.status is IdentityStatus.CONFIRMED


def socket_filters():
    """Return exact standard-ID filters used by the read-only capture."""
    return [
        {'can_id': can_id, 'can_mask': 0x7FF, 'extended': False}
        for can_id in OBSERVED_CAN_IDS
    ]


def _feedback_window(role: str, can_ids: Tuple[int, ...],
                     counts: Mapping[int, int], duration_s: float,
                     min_hz: float, conflict_hz: float) -> FeedbackWindow:
    rates = tuple(
        (can_id, counts.get(can_id, 0) / duration_s)
        for can_id in can_ids
    )
    return FeedbackWindow(
        role=role,
        can_ids=can_ids,
        rates_hz=rates,
        missing_or_slow=tuple(
            can_id for can_id, rate in rates if rate < min_hz
        ),
        conflicts=tuple(
            can_id for can_id, rate in rates if rate >= conflict_hz
        ),
    )


def evaluate_identity(counts: Mapping[int, int], duration_s: float, *,
                      side: str = 'unknown', port: str = 'unknown',
                      min_hz: float = DEFAULT_MIN_HZ,
                      conflict_hz: float = DEFAULT_CONFLICT_HZ,
                      error_frames: int = 0,
                      master_modes: Optional[Mapping[int, int]] = None
                      ) -> IdentityResult:
    """Evaluate one capture against the two-CAN master/follower contract."""
    if duration_s <= 0:
        raise ValueError('duration_s must be positive')
    if min_hz <= 0:
        raise ValueError('min_hz must be positive')
    if conflict_hz <= min_hz:
        raise ValueError('conflict_hz must be greater than min_hz')
    if error_frames < 0:
        raise ValueError('error_frames must not be negative')

    follower = _feedback_window(
        'follower', FOLLOWER_FEEDBACK_IDS, counts, duration_s,
        min_hz, conflict_hz,
    )
    master = _feedback_window(
        'master', MASTER_FEEDBACK_IDS, counts, duration_s,
        min_hz, conflict_hz,
    )
    command_ids = tuple(
        can_id for can_id in (*FOLLOWER_CONTROL_IDS, *MASTER_CONTROL_IDS)
        if counts.get(can_id, 0) > 0
    )
    config_seen = counts.get(CONFIG_CAN_ID, 0) > 0
    enable_seen = counts.get(ENABLE_CAN_ID, 0) > 0
    observed_master_modes = tuple(sorted((master_modes or {}).items()))
    unsafe_master_role = any(
        mode == LINKAGE_TEACHING_MODE and count > 0
        for mode, count in observed_master_modes
    )

    reasons = []
    if error_frames:
        reasons.append(f'观察到 {error_frames} 个 CAN 错误帧')
    if config_seen:
        reasons.append('观察到 0x470，角色或地址配置可能已被修改')
    if unsafe_master_role:
        reasons.append(
            '主臂 0x2C1 报告 0x06 联动示教输入模式，疑似仍处于 0xFA 角色'
        )
    if follower.conflicts:
        ids = ', '.join(f'0x{can_id:03X}' for can_id in follower.conflicts)
        reasons.append(f'从臂反馈窗口疑似地址冲突：{ids}')
    if master.conflicts:
        ids = ', '.join(f'0x{can_id:03X}' for can_id in master.conflicts)
        reasons.append(f'主臂反馈窗口疑似地址冲突：{ids}')
    if command_ids:
        ids = ', '.join(f'0x{can_id:03X}' for can_id in command_ids)
        reasons.append(f'观察到运动控制流：{ids}')
    if enable_seen:
        reasons.append('观察到 0x471 使能/失能广播')
    if follower.missing_or_slow:
        ids = ', '.join(
            f'0x{can_id:03X}' for can_id in follower.missing_or_slow
        )
        reasons.append(f'从臂反馈缺失或频率过低：{ids}')
    if master.missing_or_slow:
        ids = ', '.join(
            f'0x{can_id:03X}' for can_id in master.missing_or_slow
        )
        reasons.append(f'主臂反馈缺失或频率过低：{ids}')

    if error_frames:
        status = IdentityStatus.CAN_ERROR
    elif config_seen:
        status = IdentityStatus.CONFIG_CHANGED
    elif unsafe_master_role:
        status = IdentityStatus.UNSAFE_MASTER_ROLE
    elif follower.conflicts or master.conflicts:
        status = IdentityStatus.ADDRESS_CONFLICT
    elif command_ids or enable_seen:
        status = IdentityStatus.CONTROL_PRESENT
    elif follower.missing_or_slow or master.missing_or_slow:
        status = IdentityStatus.FEEDBACK_INCOMPLETE
    else:
        status = IdentityStatus.CONFIRMED
        reasons.append(
            '0x2A1～0x2A8 确认为从臂地址，'
            '0x2C1～0x2C8 确认为主臂地址'
        )

    return IdentityResult(
        side=side,
        port=port,
        duration_s=duration_s,
        follower=follower,
        master=master,
        command_ids=command_ids,
        master_modes=observed_master_modes,
        config_seen=config_seen,
        enable_seen=enable_seen,
        error_frames=error_frames,
        status=status,
        reasons=tuple(reasons),
    )


@dataclass(frozen=True)
class Capture:
    """One read-only capture of a shared master/follower bus."""

    counts: Mapping[int, int]
    elapsed: float
    master_modes: Mapping[int, int]
    error_times: Tuple[float, ...]
    degradation_frames: int

    @property
    def error_frames(self) -> int:
        """Return the number of CAN error frames observed."""
        return len(self.error_times)

    @property
    def average_error_rate_hz(self) -> float:
        """Error frames per second over the whole capture."""
        return self.error_frames / self.elapsed if self.elapsed > 0 else 0.0

    @property
    def peak_error_rate_hz(self) -> float:
        """Highest error-frame count inside any one second of the capture."""
        return peak_error_rate(self.error_times, 1.0)


def capture(port: str, duration_s: float) -> Capture:
    """Return counts, rates and error statistics for one read-only window."""
    bus = can.Bus(
        interface='socketcan',
        channel=port,
        receive_own_messages=False,
        can_filters=socket_filters(),
    )
    counts: Counter = Counter()
    master_modes: Counter = Counter()
    error_times = []
    degradation_frames = 0
    started = time.monotonic()
    deadline = started + duration_s
    try:
        while time.monotonic() < deadline:
            timeout = min(0.2, max(0.0, deadline - time.monotonic()))
            frame = bus.recv(timeout=timeout)
            if frame is None:
                continue
            if frame.is_error_frame:
                error_times.append(time.monotonic())
                if error_frame_is_degradation(frame):
                    degradation_frames += 1
                continue
            if frame.is_extended_id:
                continue
            counts[frame.arbitration_id] += 1
            if frame.arbitration_id == MASTER_FEEDBACK_IDS[0] and frame.data:
                master_modes[frame.data[0]] += 1
    finally:
        elapsed = time.monotonic() - started
        bus.shutdown()
    return Capture(
        counts=dict(counts),
        elapsed=elapsed,
        master_modes=dict(master_modes),
        error_times=tuple(error_times),
        degradation_frames=degradation_frames,
    )


def gated_error_count(capture_result: Capture, *, error_policy: str,
                      max_error_rate_hz: float) -> int:
    """
    Return how many error frames the identity gate should see.

    Under ``recoverable`` an isolated error is not an identity failure, but
    controller state degradation always is.  Under ``strict`` every error
    frame is passed through so that ``evaluate_identity`` refuses.
    """
    if error_policy == 'strict':
        return capture_result.error_frames
    if capture_result.degradation_frames:
        return capture_result.error_frames
    if capture_result.average_error_rate_hz > max_error_rate_hz:
        return capture_result.error_frames
    return 0


def _window_line(label: str, window: FeedbackWindow) -> str:
    first = window.can_ids[0]
    last = window.can_ids[-1]
    return (
        f'{label} 0x{first:03X}～0x{last:03X}：'
        f'{window.minimum_hz:.1f}～{window.maximum_hz:.1f} Hz'
    )


def print_error_summary(capture_result: Capture, *,
                        error_policy: str) -> None:
    """Print the CAN error-frame statistics of one capture."""
    print(f'CAN 错误帧：{capture_result.error_frames} 个，'
          f'平均 {capture_result.average_error_rate_hz:.2f} Hz，'
          f'单秒峰值 {capture_result.peak_error_rate_hz:.1f} Hz，'
          f'状态退化帧 {capture_result.degradation_frames} 个'
          f'（策略 {error_policy}）')


def print_result(result: IdentityResult) -> None:
    """Print a concise Chinese operator report."""
    print(f'--- {result.side} / {result.port} ---')
    print(_window_line('从臂', result.follower))
    print(_window_line('主臂', result.master))
    print(f'身份确认：{result.status.value}')
    for reason in result.reasons:
        print(f'  - {reason}')
    if result.confirmed:
        print('结论：主从臂地址身份确认通过，可以继续执行后续安全门禁。')
    else:
        print('结论：主从臂地址身份未确认，禁止进入摇操。')


def _parser() -> ArgumentParser:
    parser = ArgumentParser(description=__doc__)
    parser.add_argument(
        '--side', required=True, choices=('left', 'right'),
        help='要确认的侧别；用于报告和默认接口名',
    )
    parser.add_argument(
        '--port', metavar='IFACE',
        help='SocketCAN 接口（默认 can_<side>）',
    )
    parser.add_argument(
        '--duration', type=float, default=DEFAULT_DURATION_S,
        help='只读抓取时长，秒（默认 %(default)s）',
    )
    parser.add_argument(
        '--min-hz', type=float, default=DEFAULT_MIN_HZ,
        help='每个反馈 ID 的最低频率（默认 %(default)s）',
    )
    parser.add_argument(
        '--conflict-hz', type=float, default=DEFAULT_CONFLICT_HZ,
        help='达到此频率即判定地址冲突（默认 %(default)s）',
    )
    parser.add_argument(
        '--error-policy', choices=ERROR_POLICIES,
        default=DEFAULT_ERROR_POLICY,
        help='recoverable：容忍孤立错误帧，只在状态退化或持续速率超限时判失败'
             '（默认）；strict：任何错误帧都判失败',
    )
    parser.add_argument(
        '--max-error-rate-hz', type=float,
        default=DEFAULT_MAX_ERROR_RATE_HZ,
        help='recoverable 策略下允许的平均错误帧速率（默认 %(default)s）',
    )
    return parser


def main(args=None) -> int:
    """Run the read-only identity preflight check."""
    options = _parser().parse_args(args)
    port = options.port or f'can_{options.side}'
    try:
        # Validate thresholds before trying to open a CAN interface.
        evaluate_identity(
            {}, options.duration, side=options.side, port=port,
            min_hz=options.min_hz, conflict_hz=options.conflict_hz,
        )
    except ValueError as exc:
        print(f'错误：{exc}')
        return EXIT_FAILED

    print(f'只读确认 {options.side} 侧主从臂身份，监听 {port} '
          f'{options.duration:.1f}s（错误帧策略 {options.error_policy}）')
    print('本命令不会发送配置、使能、失能或运动帧。')
    try:
        measured = capture(port, options.duration)
    except (OSError, can.CanError) as exc:
        print(f'错误：无法监听 {port}: {exc}')
        return EXIT_FAILED
    print_error_summary(measured, error_policy=options.error_policy)
    gated = gated_error_count(
        measured, error_policy=options.error_policy,
        max_error_rate_hz=options.max_error_rate_hz)
    if gated == 0 and measured.error_frames:
        print(f'  （recoverable 策略：这 {measured.error_frames} 个错误帧不参与'
              '身份判定；速率超过上限或出现状态退化帧时仍会判失败）')
    result = evaluate_identity(
        measured.counts, measured.elapsed, side=options.side, port=port,
        min_hz=options.min_hz, conflict_hz=options.conflict_hz,
        error_frames=gated, master_modes=measured.master_modes,
    )
    print_result(result)
    return EXIT_OK if result.confirmed else EXIT_NOT_CONFIRMED


if __name__ == '__main__':
    raise SystemExit(main())
