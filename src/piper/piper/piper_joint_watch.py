#!/usr/bin/env python3
"""Watch both Piper buses' joint angles live, read-only."""

# 用途：人工逐个拖动机械臂，程序实时显示两条 CAN 总线各自的关节角度。哪一条
# 总线的读数跟着动，就说明那条接口对应你手上的那台臂，从而确证映射关系。
#
# 注意：一条总线上现在挂着一侧的主臂与从臂。两台都用常规布局时，同一批角度帧
# 由两台同时发出，这里显示的是混合结果，分不出是哪一台——此时只能确认「哪条
# 总线在动」。要分开两台，需要主臂处于偏移主臂布局（反馈 ID 整体偏移），
# 而本工具默认只收常规布局的角度帧，读不到偏移后的主臂。
#
# 本工具只调用 recv()，代码中没有任何发送路径，因此在人手接触机械臂时运行是
# 安全的：它不会使能、失能或运动任何关节，也不会下发任何指令。

import argparse
import select
import sys
import threading
import time
from typing import Dict, List, Optional, Tuple

import can

from piper.piper_feedback import JOINT_ANGLE_IDS, ArmAngleTracker, parse_offset

DEFAULT_ARMS = (
    ('can_left', 'left'),
    ('can_right', 'right'),
)
JOINTS = tuple(sorted({j for pair in JOINT_ANGLE_IDS.values() for j in pair}))
REFRESH_SECONDS = 0.15
STALE_SECONDS = 1.0
MOTION_WINDOW = 0.5
MOVED_THRESHOLD = 5.0

EXIT_OK = 0
EXIT_FAILED = 3


def open_readonly_bus(port: str, offset: int = 0):
    """Open one interface receiving only the joint angle frames."""
    bus = can.Bus(
        interface='socketcan', channel=port, receive_own_messages=False,
    )
    bus.set_filters([
        {'can_id': can_id + offset, 'can_mask': 0x7FF, 'extended': False}
        for can_id in JOINT_ANGLE_IDS
    ])
    return bus


class ArmSampler(threading.Thread):
    """Read one interface's joint angle frames into its own tracker."""

    def __init__(self, port: str, role: str, offset: int = 0,
                 bus_factory=None):
        super().__init__(daemon=True)
        self.port = port
        self.role = role
        self.offset = offset
        self.tracker = ArmAngleTracker(offset=offset)
        self.error: Optional[str] = None
        self._bus_factory = bus_factory or (
            lambda port: open_readonly_bus(port, offset))
        self._stop_event = threading.Event()
        self._lock = threading.Lock()

    def run(self) -> None:
        try:
            bus = self._bus_factory(self.port)
        except Exception as exc:  # noqa: BLE001 - 任何打开失败都要报告给操作者
            self.error = str(exc)
            return
        try:
            while not self._stop_event.is_set():
                frame = bus.recv(timeout=0.2)
                if frame is None:
                    continue
                with self._lock:
                    self.tracker.update(frame.arbitration_id, frame.data)
        finally:
            bus.shutdown()

    def stop(self) -> None:
        """Ask the reader thread to finish."""
        self._stop_event.set()

    def snapshot(self, now: float):
        """Return a consistent view of this arm for one display refresh."""
        with self._lock:
            return (
                self.tracker.angles(),
                self.tracker.max_offsets(),
                self.tracker.age(now),
                self.tracker.is_moving(MOTION_WINDOW, now),
            )

    def set_baseline(self) -> None:
        """Re-anchor the tracker, under the same lock the reader uses."""
        with self._lock:
            self.tracker.set_baseline()


class TerminalKeys:
    """Read single keypresses without blocking, when stdin is a terminal."""

    def __init__(self) -> None:
        self.enabled = sys.stdin.isatty()
        self._saved = None

    def __enter__(self) -> 'TerminalKeys':
        if self.enabled:
            import termios
            import tty
            self._saved = termios.tcgetattr(sys.stdin.fileno())
            tty.setcbreak(sys.stdin.fileno())
        return self

    def __exit__(self, *exc_info) -> None:
        if self._saved is not None:
            import termios
            termios.tcsetattr(
                sys.stdin.fileno(), termios.TCSADRAIN, self._saved)

    def read(self) -> Optional[str]:
        """Return one pending character, or None if there is none."""
        if not self.enabled:
            return None
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1)
        return None


def _angle_row(port: str, role: str, values: Dict[int, float]) -> str:
    cells = ''.join(
        f'{values[joint]:9.3f}' if joint in values else f'{"--":>9}'
        for joint in JOINTS
    )
    return f'{port:<11}{role:<16}{cells}'


def _header() -> str:
    return f'{"port":<11}{"role":<16}' + ''.join(
        f'{"j" + str(joint):>9}' for joint in JOINTS)


def _render(arms: List[ArmSampler], elapsed: float, baseline_at: float,
            threshold: float, now: float, interactive: bool) -> List[str]:
    lines = [
        '=== 两条总线关节角度实时监视（只读，不发送任何帧）===',
        f'运行 {elapsed:.0f}s　基准 T+{baseline_at:.0f}s　'
        f'按 r 重置基准　按 q 退出',
        '',
        '当前角度（度）',
        _header(),
    ]
    for arm in arms:
        angles, _, _, _ = arm.snapshot(now)
        lines.append(_angle_row(arm.port, _label(arm), angles))

    lines += ['', f'相对基准的最大位移（度），阈值 {threshold:.1f}', _header()]
    moving: List[str] = []
    moved_any: List[str] = []
    for arm in arms:
        _, peaks, age, is_moving = arm.snapshot(now)
        note = ''
        if age is None:
            note = '  <== 无角度帧'
        elif age > STALE_SECONDS:
            note = f'  <== 数据陈旧 {age:.1f}s'
        elif is_moving:
            note = '  <== 正在移动'
        lines.append(_angle_row(arm.port, arm.role, peaks) + note)
        if is_moving:
            moving.append(f'{arm.port} ({arm.role})')
        joints = [j for j in JOINTS if peaks.get(j, 0.0) >= threshold]
        if joints:
            moved_any.append(
                f'{arm.port} ({arm.role}) 的 ' +
                '、'.join(f'j{j}' for j in joints))
    lines.append('')
    lines.append('正在移动：' + ('、'.join(moving) if moving else '（无）'))
    lines.append('被移动过：' + ('；'.join(moved_any) if moved_any else '（无）'))
    if not interactive:
        lines.append('（stdout 非终端，仅输出快照，不做清屏刷新）')
    return lines


def _print_summary(arms: List[ArmSampler], threshold: float) -> None:
    print()
    print('=== 本次监视总结：每个关节相对基准的最大位移（度）===')
    print(_header())
    for arm in arms:
        _, peaks, age, _ = arm.snapshot(time.monotonic())
        suffix = '  <== 始终无角度帧' if age is None else ''
        print(_angle_row(arm.port, arm.role, peaks) + suffix)
    print()
    print(f'位移达到 {threshold:.1f} 度的接口（即被拖动过的臂）：')
    found = False
    for arm in arms:
        _, peaks, _, _ = arm.snapshot(time.monotonic())
        joints = [j for j in JOINTS if peaks.get(j, 0.0) >= threshold]
        if joints:
            found = True
            print(f'  {arm.port:<11}{_label(arm):<16}' +
                  '、'.join(f'j{j}({peaks[j]:.1f}°)' for j in joints))
    if not found:
        print('  （无）')


def _parse_arms(values: Optional[List[str]]
                ) -> Tuple[Tuple[str, str, int], ...]:
    """Parse IFACE[:ROLE][@OFFSET] specs; the offset reads a shifted arm."""
    if not values:
        return tuple((port, role, 0) for port, role in DEFAULT_ARMS)
    parsed = []
    for value in values:
        spec, _, offset_text = value.partition('@')
        port, _, role = spec.partition(':')
        try:
            offset = parse_offset(offset_text) if offset_text else 0
        except ValueError as exc:
            raise ValueError(f'{value}：{exc}') from None
        parsed.append((port, role or '-', offset))
    return tuple(parsed)


def _label(arm) -> str:
    """Label one row, showing the offset when it reads a shifted arm."""
    return f'{arm.role}@{arm.offset:#04x}' if arm.offset else arm.role


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--arm', action='append', dest='arms',
        metavar='IFACE[:ROLE][@OFFSET]',
        help='要监视的臂，可重复；默认两条总线（不带偏移）。主臂被 0x470 设为'
             '偏移主臂后其反馈 ID 偏移，用 @0x20 读它，例如 '
             '--arm can_left:主@0x20 --arm can_left:从',
    )
    parser.add_argument(
        '--moved-threshold', type=float, default=MOVED_THRESHOLD,
        help='判定「被移动过」的位移阈值，度（默认 %(default)s）',
    )
    parser.add_argument(
        '--refresh', type=float, default=REFRESH_SECONDS,
        help='刷新周期，秒（默认 %(default)s）',
    )
    parser.add_argument(
        '--duration', type=float, default=None,
        help='可选：运行指定秒数后自动退出（默认手动退出）',
    )
    return parser


def main(args=None) -> int:
    """Display joint angles until interrupted; transmits nothing."""
    options = _parser().parse_args(args)
    if options.refresh <= 0:
        print('错误：--refresh 必须为正')
        return EXIT_FAILED
    try:
        specs = _parse_arms(options.arms)
    except ValueError as exc:
        print(f'错误：{exc}')
        return EXIT_FAILED
    arms = [
        ArmSampler(port, role, offset)
        for port, role, offset in specs
    ]
    for arm in arms:
        arm.start()

    interactive = sys.stdout.isatty()
    started = time.monotonic()
    baseline_at = 0.0
    # 给采样线程一点时间拿到首帧，再固定基准，避免开机抖动被判成位移。
    time.sleep(0.5)
    for arm in arms:
        arm.set_baseline()
    baseline_at = time.monotonic() - started

    try:
        with TerminalKeys() as keys:
            while True:
                now = time.monotonic()
                elapsed = now - started
                if interactive:
                    sys.stdout.write('\033[H')
                    for line in _render(arms, elapsed, baseline_at,
                                        options.moved_threshold, now,
                                        interactive):
                        sys.stdout.write('\033[K' + line + '\n')
                    sys.stdout.write('\033[J')
                    sys.stdout.flush()
                else:
                    if int(elapsed / 1.0) != int((elapsed - options.refresh)
                                                 / 1.0):
                        for line in _render(arms, elapsed, baseline_at,
                                            options.moved_threshold, now,
                                            interactive):
                            print(line)
                    sys.stdout.flush()

                key = keys.read()
                if key in ('q', 'Q', '\x03', '\x04'):
                    break
                if key in ('r', 'R'):
                    for arm in arms:
                        arm.set_baseline()
                    baseline_at = time.monotonic() - started

                if options.duration is not None and elapsed >= options.duration:
                    break
                time.sleep(options.refresh)
    except KeyboardInterrupt:
        pass
    finally:
        for arm in arms:
            arm.stop()

    for arm in arms:
        if arm.error is not None:
            print(f'错误：{arm.port} 无法监听：{arm.error}')
    _print_summary(arms, options.moved_threshold)
    print()
    print('提示：只有被你拖动的那台臂对应的接口位移应该变化；'
          '两台臂共总线时，用 @0x20 读被设为偏移主臂的那台，'
          '不带偏移读到的才是常规臂。')
    return EXIT_OK


if __name__ == '__main__':
    raise SystemExit(main())
