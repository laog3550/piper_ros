#!/usr/bin/env python3
"""Record a teleop run read-only and report how well the follower tracked."""

# 用途：遥操作验证。两种模式：
#
#   --record（默认）：以接收-only 套接字记录两臂的关节角度帧（约 1200 帧/秒），
#                     不发送任何帧；同时打印遥操作该用的命令。
#   --analyze PATH ：对记录文件算指标——按拖动速度分档的滞后、静止段的抖动、
#                     拖动中 follower 的停顿占比、以及两臂帧率是否正常。
#
# 为什么不是直接看遥操作自己的输出：它每秒只打印一行（1 Hz），而滞后、抖动、
# 停顿都发生在 20~50 Hz 的尺度上；另外 1200 帧/秒的记录才能保留 5~15 Hz 抖动
# （50 Hz 采样会把它混叠掉）。
#
# 指标口径（都是"同一拖动速度档内"比，跨运行直接比会得出错误结论）：
#   * 滞后：|master − follower| 的中位数与 90 分位，按该关节 master 的速度分档；
#   * 抖动：master 连续静止 ≥1 秒的段里，follower 的 5~15 Hz 带内 RMS（零相位
#     带通，避免边界伪影）；
#   * 停顿：拖动中 follower 速度接近零的采样占比。
#
# 判据（本项目的经验值）：
#   * 帧率：两臂都应约 1200 帧/秒，每秒之间波动 < 5%；
#   * 滞后：1~5 deg/s 档应在 1 度上下；20~80 档在 7~8 度上下；再高就该考虑
#     驱动器侧的加速度配置了（见操作手册「跟随速度」一节）；
#   * 静止段抖动：应在 0.01~0.04 度（机械臂自身本底）。

import csv
import threading
import time
from argparse import ArgumentParser
from collections import defaultdict
from typing import Dict, List, Optional, Tuple

import can
import numpy as np

from piper.piper_feedback import JOINT_ANGLE_IDS, JOINT_COUNT, decode_joint_angles

# 记录的是哪两台臂：默认 master_left(can_fl) 与 follower_left(can_fr)。
DEFAULT_PORTS = (('master', 'can_fl'), ('follower', 'can_fr'))
DEFAULT_SECONDS = 70.0
SAMPLE_HZ = 50.0
WOBBLE_BAND = (5.0, 15.0)
STILL_SPEED_DEG_S = 0.5
STILL_MIN_SECONDS = 1.0
SPEED_BANDS = ((1.0, 5.0), (5.0, 20.0), (20.0, 80.0), (80.0, 400.0))
NEAR_ZERO_DEG_S = 0.2
DEFAULT_CSV = '/tmp/piper_teleop_verify.csv'

EXIT_OK = 0
EXIT_FAILED = 3


def _open_readonly(port: str) -> can.BusABC:
    """Open one interface receiving only the joint angle frames."""
    bus = can.Bus(interface='socketcan', channel=port,
                  receive_own_messages=False)
    bus.set_filters([
        {'can_id': can_id, 'can_mask': 0x7FF, 'extended': False}
        for can_id in JOINT_ANGLE_IDS
    ])
    return bus


class Recorder(threading.Thread):
    """Read one interface's joint angle frames; never transmits."""

    def __init__(self, role: str, port: str, sink: list, lock,
                 started: float):
        super().__init__(daemon=True)
        self.role = role
        self.port = port
        self.sink = sink
        self.lock = lock
        self.started = started
        self.stop_event = threading.Event()
        self.error: Optional[str] = None

    def run(self) -> None:
        """Append (role, joint, t, deg) rows until asked to stop."""
        try:
            bus = _open_readonly(self.port)
        except Exception as exc:  # noqa: BLE001 - 打开失败要如实报告
            self.error = str(exc)
            return
        try:
            while not self.stop_event.is_set():
                frame = bus.recv(timeout=0.2)
                if frame is None:
                    continue
                angles = decode_joint_angles(frame.arbitration_id, frame.data)
                if not angles:
                    continue
                now = time.monotonic() - self.started
                with self.lock:
                    for joint, angle in sorted(angles.items()):
                        self.sink.append((self.role, joint, now, angle))
        finally:
            bus.shutdown()


def record(ports, seconds: float, path: str) -> int:
    """Capture both arms for ``seconds`` and write one CSV."""
    started = time.monotonic()
    rows: List[Tuple[str, int, float, float]] = []
    lock = threading.Lock()
    readers = [Recorder(role, port, rows, lock, started)
               for role, port in ports]
    for reader in readers:
        reader.start()
    for reader in readers:
        if reader.error:
            print(f'错误：无法打开 {reader.port}: {reader.error}')
            return EXIT_FAILED
    print(f'只读记录中（{seconds:.0f} 秒，接收-only，不发送任何帧）…')
    print('现在请在另一个终端启动遥操作，例如：')
    print('  ros2 run piper piper_teleop --side left --enable --duration 50')
    while time.monotonic() - started < seconds:
        time.sleep(0.2)
    for reader in readers:
        reader.stop_event.set()
    time.sleep(0.3)

    with open(path, 'w', newline='') as handle:
        writer = csv.writer(handle)
        writer.writerow(['arm', 'joint', 't', 'deg'])
        writer.writerows(sorted(rows, key=lambda row: (row[0], row[2])))
    for role, port in ports:
        count = sum(1 for row in rows if row[0] == role)
        print(f'  {role:<9}（{port}）：{count / seconds:.0f} 帧/秒')
    print(f'记录 {len(rows)} 帧到 {path}')
    print(f'结束后运行：ros2 run piper piper_teleop_verify --analyze {path}')
    return EXIT_OK


def _load(path: str, rate: float) -> Dict[str, Dict[int, np.ndarray]]:
    """Resample both arms' recordings onto a uniform grid."""
    raw: Dict[Tuple[str, int], List[Tuple[float, float]]] = defaultdict(list)
    with open(path) as handle:
        for row in csv.reader(handle):
            if row[0] == 'arm':
                continue
            raw[(row[0], int(row[1]))].append((float(row[2]), float(row[3])))
    out: Dict[str, Dict[int, np.ndarray]] = defaultdict(dict)
    for (role, joint), samples in raw.items():
        times = np.array([t for t, _ in samples])
        values = np.array([v for _, v in samples])
        grid = np.arange(times[0], times[-1], 1.0 / rate)
        out[role][joint] = np.interp(grid, times, values)
    return out


def _band_rms(values, band=WOBBLE_BAND, rate: float = SAMPLE_HZ) -> float:
    """Zero-phase band-pass RMS, so window edges cannot fake energy."""
    from scipy.signal import butter, sosfiltfilt
    signal = np.asarray(values, dtype=float)
    sos = butter(4, band, btype='bandpass', fs=rate, output='sos')
    return float(np.sqrt(np.mean(sosfiltfilt(sos, signal - signal.mean()) ** 2)))


def _stretches(mask, min_samples: int):
    """Contiguous True stretches of at least ``min_samples``."""
    out, start = [], None
    for index, flag in enumerate(mask):
        if flag and start is None:
            start = index
        elif not flag and start is not None:
            if index - start >= min_samples:
                out.append((start, index))
            start = None
    if start is not None and len(mask) - start >= min_samples:
        out.append((start, len(mask)))
    return out


def analyze(path: str, rate: float = SAMPLE_HZ) -> int:
    """Print the tracking metrics of one recording."""
    data = _load(path, rate)
    if 'master' not in data or 'follower' not in data:
        print(f'错误：{path} 里没有 master/follower 两臂的数据')
        return EXIT_FAILED
    count = min(min(len(v) for v in data[role].values())
                for role in ('master', 'follower'))
    seconds = count / rate
    master = {j: data['master'][j][:count] for j in range(1, JOINT_COUNT + 1)}
    follower = {j: data['follower'][j][:count] for j in
                range(1, JOINT_COUNT + 1)}
    speed = {j: np.abs(np.gradient(master[j])) * rate
             for j in range(1, JOINT_COUNT + 1)}
    follower_speed = {j: np.abs(np.gradient(follower[j])) * rate
                      for j in range(1, JOINT_COUNT + 1)}

    print(f'{path}：{seconds:.1f} 秒')
    print()
    print('按拖动速度分档的滞后（|master − follower|，度）')
    print(f'  {"速度档":<16}{"滞后中位":>10}{"滞后90分位":>12}{"采样占比":>10}')
    for low, high in SPEED_BANDS:
        lags = []
        for joint in range(1, JOINT_COUNT + 1):
            mask = (speed[joint] >= low) & (speed[joint] < high)
            if mask.sum() >= 30:
                lags.extend(np.abs(master[joint][mask] - follower[joint][mask]))
        if lags:
            print(f'  {f"{low:g}~{high:g} deg/s":<16}'
                  f'{np.median(lags):>10.3f}'
                  f'{np.percentile(lags, 90):>12.3f}{len(lags):>10}')

    moving = np.max([speed[j] for j in range(1, JOINT_COUNT + 1)], axis=0)
    still = moving < STILL_SPEED_DEG_S
    segments = _stretches(still, int(STILL_MIN_SECONDS * rate))
    print()
    if segments:
        master_wobble = max(_band_rms(master[j][a:b], rate=rate)
                            for a, b in segments
                            for j in range(1, JOINT_COUNT + 1))
        follower_wobble = max(_band_rms(follower[j][a:b], rate=rate)
                              for a, b in segments
                              for j in range(1, JOINT_COUNT + 1))
        total = sum(b - a for a, b in segments) / rate
        print(f'静止段 {len(segments)} 段、共 {total:.1f} 秒：'
              f'5~15 Hz 抖动 master {master_wobble:.4f} / '
              f'follower {follower_wobble:.4f} 度')
    else:
        print('没有找到 master 连续静止 ≥1 秒的段，算不出静止抖动')

    print()
    print('拖动中 follower 的停顿占比（该关节 master 在动、follower 速度≈0）')
    print(f'  {"速度档":<16}{"停顿占比":>10}{"速度标准差":>12}')
    for low, high in SPEED_BANDS:
        parts, totals = [], []
        for joint in range(1, JOINT_COUNT + 1):
            mask = (speed[joint] >= low) & (speed[joint] < high)
            if mask.sum() < 30:
                continue
            parts.append(100 * float(np.mean(
                follower_speed[joint][mask] < NEAR_ZERO_DEG_S)))
            totals.extend(follower_speed[joint][mask])
        if parts:
            print(f'  {f"{low:g}~{high:g} deg/s":<16}'
                  f'{np.mean(parts):>9.0f}%{np.std(totals):>12.2f}')
    print()
    print('提示：跨运行比较必须按同一速度档，否则拖动内容的差异会盖过真实差别。')
    return EXIT_OK


def _parser() -> ArgumentParser:
    parser = ArgumentParser(description=__doc__)
    parser.add_argument('--analyze', metavar='CSV',
                        help='分析一份记录，而不是记录新的')
    parser.add_argument('--csv', default=DEFAULT_CSV,
                        help='记录写到哪个文件（默认 %(default)s）')
    parser.add_argument('--duration', type=float, default=DEFAULT_SECONDS,
                        help='记录多少秒（默认 %(default)s；要比遥操作长）')
    parser.add_argument('--arm', action='append', dest='arms',
                        metavar='ROLE:IFACE',
                        help='记录哪两臂，默认 '
                             + ' '.join(f'{r}:{p}' for r, p in DEFAULT_PORTS))
    return parser


def main(args=None) -> int:
    """Record a run, or analyze a recording."""
    options = _parser().parse_args(args)
    if options.analyze:
        return analyze(options.analyze)
    ports = tuple(
        tuple(value.split(':', 1)) for value in options.arms
    ) if options.arms else DEFAULT_PORTS
    if options.duration <= 0:
        print('错误：--duration 必须为正')
        return EXIT_FAILED
    return record(ports, options.duration, options.csv)


if __name__ == '__main__':
    raise SystemExit(main())
