#!/usr/bin/env python3
"""Read a SocketCAN bus and report its load, shared IDs and errors."""

# 用途：为“物理层诊断”提供可重复、可对比的只读测量。它不发送任何帧，也不改变任何
# 配置，只统计总线上的现有流量，因此可以在四台臂通电的任何时刻运行。
#
# 报出的四类信息对应四个待查项：
#   1. 接口状态与内核计数（bus-off／错误被动／丢帧）——是否已经退化；
#   2. 帧率与估算负载——两 CAN 拓扑把两台臂塞进一条总线，负载接近饱和时，
#      任何电气余量不足都会被放大；
#   3. 共享 ID（同一条总线上有 ≥2 台设备在发同一个 CAN ID）——数据段冲突的根源，
#      目前固件不随 0x20 偏移搬移 0x251～0x266，所以它们无法通过配置消除；
#   4. 错误帧：总数、速率、突发结构、类别位，以及“错误帧前后 ±300µs 内出现的
#      帧 ID 相对其基线占比的过代表倍数”——用来判断是哪些流量在冲突。
#
# 注意（实测 2026-09-30）：错误帧只交付给显式打开 CAN_RAW_ERR_FILTER 的套接字。
# 本机 can-utils 2020.11 的 `candump -e` 在同样条件下报 0 个错误帧，而本工具
# （python-can 会自动打开该过滤器）与裸套接字都能看到，所以基线统计请用本工具。

import time
from argparse import ArgumentParser
from collections import Counter, defaultdict, deque
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import can

from piper.piper_feedback import JOINT_COUNT, decode
from piper.piper_two_can_identity import (
    error_frame_class,
    error_frame_is_degradation,
    peak_error_rate,
)

DEFAULT_SECONDS = 20.0
DEFAULT_NEAR_WINDOW_S = 300e-6
DEFAULT_BITRATE = 1_000_000.0

EXIT_OK = 0
EXIT_ERRORS = 2
EXIT_DEGRADED = 3

# 反馈族分组，用于打印可读的帧率表。
FAMILIES = (
    ('末端/手（0x1C0～0x1C3、0x212）', (0x1C0, 0x1C1, 0x1C2, 0x1C3, 0x212)),
    ('高速关节反馈 0x251～0x256', tuple(range(0x251, 0x257))),
    ('低速驱动反馈 0x261～0x266', tuple(range(0x261, 0x267))),
    ('从臂高速反馈 0x2A1～0x2A8', tuple(range(0x2A1, 0x2A9))),
    ('主臂（+0x20）反馈 0x2C1～0x2C8', tuple(range(0x2C1, 0x2C9))),
)
# 单台臂的标称帧率（见 docs/PI05_CAN_MAPPING.md）：高速反馈每个 ID 约 200 Hz，
# 低速驱动反馈每个 ID 约 40 Hz。超过下面的门限就说明同一条总线上有 ≥2 台设备在发
# 同一个 CAN ID——数据段冲突的根源。
SHARED_HIGH_SPEED_HZ = 300.0
SHARED_LOW_SPEED_HZ = 60.0


@dataclass(frozen=True)
class BusQuality:
    """One read-only measurement of a CAN bus."""

    port: str
    seconds: float
    frames: int
    dlc_histogram: Dict[int, int]
    per_id_rate: Dict[int, float]
    family_rates: Tuple[Tuple[str, float], ...]
    shared_ids: Tuple[int, ...]
    error_frames: int
    error_classes: Tuple[Tuple[str, int], ...]
    error_rate_hz: float
    peak_error_rate_hz: float
    degradation_frames: int
    near_ids: Tuple[Tuple[int, float, int], ...]
    nominal_load_kbit_s: float
    worst_case_load_kbit_s: float
    bitrate: float
    joint_enable_samples: Tuple[bool, ...] = ()

    @property
    def load_percent(self) -> float:
        """Worst-case bus load as a percentage of the configured bitrate."""
        if self.bitrate <= 0:
            return 0.0
        return self.worst_case_load_kbit_s / (self.bitrate / 1000.0) * 100.0

    @property
    def verdict(self) -> str:
        """Return one of CLEAN / ERRORS / DEGRADED / OVERLOAD."""
        if self.degradation_frames:
            return 'DEGRADED'
        if self.load_percent >= 80.0:
            return 'OVERLOAD'
        if self.error_frames:
            return 'ERRORS'
        return 'CLEAN'


def frame_bits(dlc: int, *, worst_case_stuffing: bool = True) -> int:
    """
    Estimate the bits one classic CAN frame occupies on the wire.

    Nominal overhead is 44 bits plus 3 bits inter-frame space; bit stuffing
    adds at most one bit per four bits of the SOF..CRC region, and roughly
    half of that in practice.  The worst-case number is what decides whether
    a bus still has headroom.
    """
    overhead = 44 + 3
    stuffing_region = 34 + 8 * dlc
    stuffing = (stuffing_region - 1) // 4 if worst_case_stuffing else 0
    return overhead + 8 * dlc + stuffing


def _family_of(can_id: int) -> Optional[str]:
    for name, ids in FAMILIES:
        if can_id in ids:
            return name
    return None


def measure(port: str, seconds: float, *,
            near_window_s: float = DEFAULT_NEAR_WINDOW_S,
            bitrate: float = DEFAULT_BITRATE) -> BusQuality:
    """Watch one bus and summarise its traffic, load and error frames."""
    bus = can.Bus(interface='socketcan', channel=port,
                  receive_own_messages=False)
    counts: Counter = Counter()
    dlc_histogram: Counter = Counter()
    error_classes: Counter = Counter()
    error_times = []
    degradation = 0
    near_before: Counter = Counter()
    near_after: Counter = Counter()
    enable_samples: Dict[int, set] = {joint: set()
                                      for joint in range(1, JOINT_COUNT + 1)}
    near_total = 0
    recent = deque(maxlen=16)
    after_deadline = None
    started = time.monotonic()
    deadline = started + seconds
    try:
        while time.monotonic() < deadline:
            frame = bus.recv(
                timeout=min(0.2, max(0.0, deadline - time.monotonic())))
            if frame is None:
                continue
            now = frame.timestamp or time.monotonic()
            if frame.is_error_frame:
                error_times.append(now)
                error_classes[(error_frame_class(frame),
                               bytes(frame.data).hex(' '))] += 1
                if error_frame_is_degradation(frame):
                    degradation += 1
                for moment, can_id in recent:
                    if now - moment <= near_window_s:
                        near_before[can_id] += 1
                        near_total += 1
                after_deadline = now + near_window_s
                continue
            counts[frame.arbitration_id] += 1
            dlc_histogram[frame.dlc] += 1
            feedback = decode(frame.arbitration_id, frame.data)
            if feedback is not None:
                enable_samples[feedback.joint].add(feedback.enabled)
            if after_deadline is not None:
                if now <= after_deadline:
                    near_after[frame.arbitration_id] += 1
                    near_total += 1
                else:
                    after_deadline = None
            recent.append((now, frame.arbitration_id))
    finally:
        elapsed = time.monotonic() - started
        bus.shutdown()

    per_id_rate = {
        can_id: count / elapsed for can_id, count in counts.items()
    }
    family_totals = defaultdict(int)
    for can_id, count in counts.items():
        name = _family_of(can_id)
        family_totals[name or '其他'] += count
    family_rates = tuple(
        (name, family_totals[name] / elapsed)
        for name, _ids in FAMILIES if family_totals.get(name)
    ) + ((('其他', family_totals['其他'] / elapsed),)
         if family_totals.get('其他') else ())
    shared_ids = tuple(sorted(
        can_id for can_id, rate in per_id_rate.items()
        if rate > _shared_threshold(can_id)
    ))
    total_frames = sum(counts.values())
    near_ids = _near_id_ranking(near_before, near_after, counts, near_total,
                                total_frames)
    nominal = sum(
        count * frame_bits(dlc, worst_case_stuffing=False)
        for dlc, count in dlc_histogram.items()) / elapsed / 1000.0
    worst = sum(count * frame_bits(dlc)
                for dlc, count in dlc_histogram.items()) / 1000.0
    return BusQuality(
        port=port,
        seconds=elapsed,
        frames=total_frames,
        dlc_histogram=dict(dlc_histogram),
        per_id_rate=per_id_rate,
        family_rates=family_rates,
        shared_ids=shared_ids,
        error_frames=len(error_times),
        error_classes=tuple(error_classes.most_common(6)),
        error_rate_hz=len(error_times) / elapsed if elapsed else 0.0,
        peak_error_rate_hz=peak_error_rate(error_times, 1.0),
        degradation_frames=degradation,
        near_ids=near_ids,
        nominal_load_kbit_s=nominal,
        worst_case_load_kbit_s=worst / elapsed,
        bitrate=bitrate,
        joint_enable_samples=_enable_summary(enable_samples),
    )


def _enable_summary(samples: Dict[int, set]) -> Tuple[bool, ...]:
    """
    Collapse the six joints' observed enable bits into one tuple.

    A joint that reported both values inside the window is reported as
    enabled, because that is the unsafe direction: the follower must never be
    assumed disabled when some frame claimed it was on.
    """
    summary = []
    for joint in range(1, JOINT_COUNT + 1):
        values = samples.get(joint) or set()
        summary.append(True in values)
    return tuple(summary)


def _shared_threshold(can_id: int) -> float:
    """Return the rate above which an ID must have more than one sender."""
    if 0x261 <= can_id <= 0x266:
        return SHARED_LOW_SPEED_HZ
    return SHARED_HIGH_SPEED_HZ


def _near_id_ranking(near_before: Counter, near_after: Counter,
                     counts: Counter, near_total: int,
                     total_frames: int) -> Tuple[Tuple[int, float, int], ...]:
    """Rank IDs by how much more often they sit next to an error frame."""
    if not near_total or not total_frames:
        return ()
    ranked = []
    for can_id, baseline in counts.items():
        observed = near_before.get(can_id, 0) + near_after.get(can_id, 0)
        if observed < 20:
            continue
        expected = near_total * baseline / total_frames
        if expected <= 0:
            continue
        ranked.append((can_id, observed / expected, observed))
    ranked.sort(key=lambda item: item[1], reverse=True)
    return tuple(ranked[:6])


def render(quality: BusQuality) -> str:
    """Build the full Chinese report text."""
    lines = []
    lines.append(f'--- {quality.port}（{quality.seconds:.1f}s 只读）---')
    rate = quality.frames / quality.seconds
    lines.append(f'  普通帧 {quality.frames} 个（{rate:.0f}/s），'
                 f'估算负载 {quality.nominal_load_kbit_s:.0f}～'
                 f'{quality.worst_case_load_kbit_s:.0f} kbit/s'
                 f'（最坏情况 {quality.load_percent:.0f}% of '
                 f'{quality.bitrate / 1000.0:.0f} kbit/s）')
    lines.append('  分组帧率：')
    for name, rate in quality.family_rates:
        lines.append(f'    {name}: {rate:.0f}/s')
    if quality.shared_ids:
        ids = ', '.join(f'0x{can_id:03X}' for can_id in quality.shared_ids)
        lines.append('  共享 ID（同一条总线上 ≥2 台设备在发）：'
                     f'{len(quality.shared_ids)} 个 → {ids}')
    else:
        lines.append('  共享 ID：没有发现')
    if quality.joint_enable_samples:
        observed = ('全部 disabled' if not any(quality.joint_enable_samples)
                    else '全部 enabled'
                    if all(quality.joint_enable_samples)
                    else 'enabled／disabled 混合')
        lines.append(f'  六关节使能（低速反馈）：{observed}')
    lines.append(f'  CAN 错误帧：{quality.error_frames} 个，平均 '
                 f'{quality.error_rate_hz:.2f}/s，单秒峰值 '
                 f'{quality.peak_error_rate_hz:.1f}/s，状态退化帧 '
                 f'{quality.degradation_frames} 个')
    for name, payload in quality.error_classes:
        lines.append(f'    类别 {name}｜data={payload or "空"}')
    if quality.near_ids:
        lines.append('  错误帧近邻 ID（过代表倍数，≥20 次样本）：')
        for can_id, ratio, observed in quality.near_ids:
            lines.append(f'    0x{can_id:03X}: ×{ratio:.2f}（{observed} 次）')
    lines.append(f'  结论：{quality.verdict}')
    return '\n'.join(lines)


def summary_line(quality: BusQuality) -> str:
    """Return one paste-able line for before/after comparisons."""
    shared = ','.join(
        f'0x{can_id:03X}' for can_id in quality.shared_ids) or '无'
    rate = quality.frames / quality.seconds
    return (f'[{quality.port}] 帧 {rate:.0f}/s，'
            f'负载 {quality.load_percent:.0f}%，错误帧 {quality.error_frames}'
            f'（{quality.error_rate_hz:.2f}/s，峰值 '
            f'{quality.peak_error_rate_hz:.0f}/s），退化 '
            f'{quality.degradation_frames}，共享 ID {shared}，'
            f'结论 {quality.verdict}')


def interface_state(port: str) -> str:
    """Read the kernel's view of one interface, read-only."""
    lines = []
    try:
        path = f'/sys/class/net/{port}/operstate'
        with open(path, encoding='utf-8') as handle:
            lines.append(f'  {port} operstate: {handle.read().strip()}')
    except OSError as exc:
        lines.append(f'  {port} operstate 读取失败：{exc}')
    return '\n'.join(lines)


def _parser() -> ArgumentParser:
    parser = ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='can_left',
                        help='要体检的 SocketCAN 接口（默认 %(default)s）')
    parser.add_argument('--seconds', type=float, default=DEFAULT_SECONDS,
                        help='测量时长，秒（默认 %(default)s）')
    parser.add_argument('--near-window-us', type=float,
                        default=DEFAULT_NEAR_WINDOW_S * 1e6,
                        help='错误帧近邻统计的时间窗，微秒（默认 %(default)s）')
    parser.add_argument('--bitrate', type=float, default=DEFAULT_BITRATE,
                        help='总线波特率，用于换算负载百分比（默认 %(default)s）')
    parser.add_argument('--repeat', type=int, default=1,
                        help='连续测量几轮；每轮打印一行摘要，用来观察间歇性'
                             '（默认 %(default)s）')
    return parser


def main(args=None) -> int:
    """Run one read-only bus quality measurement."""
    options = _parser().parse_args(args)
    if options.seconds <= 0:
        print('错误：--seconds 必须为正')
        return EXIT_DEGRADED
    print(f'只读体检 {options.port}，{options.seconds:.1f}s × '
          f'{max(options.repeat, 1)} 轮；本命令不发送任何帧。')
    print(interface_state(options.port))
    worst = None
    for round_index in range(1, max(options.repeat, 1) + 1):
        try:
            quality = measure(
                options.port, options.seconds,
                near_window_s=options.near_window_us / 1e6,
                bitrate=options.bitrate,
            )
        except (OSError, can.CanError) as exc:
            print(f'错误：无法监听 {options.port}: {exc}')
            return EXIT_DEGRADED
        if options.repeat > 1:
            print(f'第 {round_index} 轮摘要：' + summary_line(quality))
        else:
            print(render(quality))
            print('对比用摘要：' + summary_line(quality))
        if worst is None or _verdict_rank(quality) > _verdict_rank(worst):
            worst = quality
    print(interface_state(options.port))
    if worst is None:
        return EXIT_OK
    if worst.verdict == 'DEGRADED':
        return EXIT_DEGRADED
    if worst.verdict == 'CLEAN':
        return EXIT_OK
    return EXIT_ERRORS


def _verdict_rank(quality: BusQuality) -> int:
    """Order verdicts by severity so a repeat run reports the worst one."""
    return {'CLEAN': 0, 'ERRORS': 1, 'OVERLOAD': 2, 'DEGRADED': 3}[
        quality.verdict]


if __name__ == '__main__':
    raise SystemExit(main())
