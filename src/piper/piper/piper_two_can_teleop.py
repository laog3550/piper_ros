#!/usr/bin/env python3
"""Configure and observe the official PiPER FA/FC native teleoperation link."""

from argparse import ArgumentParser
from collections import Counter
from dataclasses import dataclass, field
import math
import sys
import time

import can

from piper.piper_can_writer import CanWriterLease
from piper.piper_feedback import CORE_FEEDBACK_CAN_IDS, HIGH_SPEED_CAN_IDS


MASTER_ROLE = 0xFA
FOLLOWER_ROLE = 0xFC
ROLE_CONFIG_ID = 0x470
NATIVE_CONTROL_IDS = (0x151, 0x155, 0x156, 0x157, 0x159)
SHIFTED_MASTER_IDS = tuple(can_id + 0x20 for can_id in NATIVE_CONTROL_IDS)
SHIFTED_STARTUP_MIN_COUNTS = {can_id: (1 if can_id == 0x171 else 10)
                              for can_id in SHIFTED_MASTER_IDS}
MIN_FRAMES = 10
NOMINAL_FEEDBACK_HZ = 200.0
EXIT_OK = 0
EXIT_REFUSED = 2
EXIT_FAILED = 3


@dataclass
class Capture:
    """One fresh CAN observation window, including error and status frames."""

    seconds: float
    counts: Counter = field(default_factory=Counter)
    follower_modes: set = field(default_factory=set)
    enable_payloads: Counter = field(default_factory=Counter)
    errors: int = 0


def capture(bus, seconds: float, ready_counts=None) -> Capture:
    """Receive only; never send a frame or alter either arm's mode."""
    result = Capture(seconds)
    started = time.monotonic()
    deadline = started + seconds
    ready_at = None
    while True:
        now = time.monotonic()
        remaining = deadline - now
        if remaining <= 0:
            break
        if ready_at is not None and now - ready_at >= 2.0:
            break
        frame = bus.recv(timeout=min(0.2, remaining))
        if frame is None:
            continue
        if frame.is_error_frame:
            result.errors += 1
            continue
        if frame.is_extended_id or frame.is_remote_frame:
            continue
        can_id = int(frame.arbitration_id)
        result.counts[can_id] += 1
        if can_id == 0x2A1 and frame.data:
            result.follower_modes.add(frame.data[0])
        elif can_id == 0x471:
            result.enable_payloads[bytes(frame.data)] += 1
        if ready_counts and ready_at is None and all(
                result.counts[expected_id] >= minimum
                for expected_id, minimum in ready_counts.items()):
            ready_at = time.monotonic()
    result.seconds = time.monotonic() - started
    return result


def verify_native(window: Capture) -> list[str]:
    """Require the official five master commands and one follower feedback set."""
    counts = window.counts
    problems = []
    missing_feedback = [can_id for can_id in CORE_FEEDBACK_CAN_IDS
                        if counts[can_id] < MIN_FRAMES]
    missing_control = [can_id for can_id in NATIVE_CONTROL_IDS
                       if not counts[can_id]]
    if missing_feedback:
        problems.append('从臂反馈缺失：' + _ids(missing_feedback))
    if missing_control:
        problems.append('主臂联动控制帧缺失：' + _ids(missing_control))
    shifted_feedback = [can_id for can_id in range(0x2C1, 0x2C9)
                        if counts[can_id]]
    if shifted_feedback:
        problems.append('出现旧方案偏移反馈：' + _ids(shifted_feedback))
    shifted_control = [can_id for can_id in range(0x160, 0x180)
                       if counts[can_id]]
    if shifted_control:
        problems.append('出现偏移控制帧：' + _ids(shifted_control))
    extra_control = [can_id for can_id in range(0x150, 0x160)
                     if counts[can_id] and can_id not in NATIVE_CONTROL_IDS]
    if extra_control:
        problems.append('出现官方五类以外的控制帧：' + _ids(extra_control))
    if counts[ROLE_CONFIG_ID]:
        problems.append('运行中出现角色配置帧 0x470')
    observed = sum(counts[can_id] for can_id in HIGH_SPEED_CAN_IDS)
    nominal = len(HIGH_SPEED_CAN_IDS) * NOMINAL_FEEDBACK_HZ * window.seconds
    if observed > nominal * 1.6:
        problems.append('反馈帧率超过单台从臂范围，可能有同 ID 冲突')
    if 0x06 in window.follower_modes:
        problems.append('反馈侧处于 0x06 示教输入模式，应为 0xFC 从臂')
    if window.errors:
        problems.append(f'CAN 错误帧 {window.errors} 个')
    return problems


def verify_isolated_arm(window: Capture, role: str) -> list[str]:
    """Check one standard arm or the deployed FC+0x20 master before 0x470."""
    counts = window.counts
    problems = []
    standard_status = tuple(range(0x2A1, 0x2A9))
    shifted_status = tuple(range(0x2C1, 0x2C9))
    standard = all(counts[can_id] >= MIN_FRAMES
                   for can_id in standard_status)
    shifted = all(counts[can_id] >= MIN_FRAMES
                  for can_id in shifted_status)
    old_master = role == 'master' and shifted and not any(
        counts[can_id] for can_id in standard_status)
    expected = tuple(can_id for can_id in CORE_FEEDBACK_CAN_IDS
                     if can_id not in standard_status)
    expected += shifted_status if old_master else standard_status
    missing = [can_id for can_id in expected
               if counts[can_id] < MIN_FRAMES]
    if missing:
        problems.append('单臂反馈缺失：' + _ids(missing))
    if standard and shifted:
        problems.append('同时存在标准和偏移状态反馈，未隔离目标臂')
    observed = sum(counts[can_id] for can_id in HIGH_SPEED_CAN_IDS)
    nominal = len(HIGH_SPEED_CAN_IDS) * NOMINAL_FEEDBACK_HZ * window.seconds
    if not 0.65 <= observed / nominal <= 1.4:
        problems.append('高速反馈帧率不符合单臂范围')
    control = [can_id for can_id in range(0x150, 0x180) if counts[can_id]]
    if control or counts[ROLE_CONFIG_ID] or counts[0x471]:
        problems.append('配置前总线上存在控制或配置帧')
    if shifted and not old_master:
        problems.append('偏移反馈只允许用于转换旧 0xFC + 0x20 主臂')
    if window.errors:
        problems.append(f'CAN 错误帧 {window.errors} 个')
    return problems


def verify_shifted_master_startup(window: Capture) -> list[str]:
    """Recognize an isolated old-offset master during its brief boot stream."""
    counts = window.counts
    problems = []
    missing = [can_id for can_id, minimum in SHIFTED_STARTUP_MIN_COUNTS.items()
               if counts[can_id] < minimum]
    if missing:
        problems.append('未见完整的偏移主臂启动控制帧：' + _ids(missing))
    if any(counts[can_id] for can_id in range(0x2A1, 0x2A9)):
        problems.append('发现默认地址反馈，可能有从臂仍在线')
    if any(counts[can_id] for can_id in NATIVE_CONTROL_IDS):
        problems.append('发现默认地址控制帧，不能识别为旧偏移主臂')
    if counts[ROLE_CONFIG_ID]:
        problems.append('观察期出现另一条 0x470 配置帧')
    return problems


def verify_recovery_bus(window: Capture) -> list[str]:
    """Allow the known master stream, but reject a newly visible peer."""
    counts = window.counts
    problems = []
    if any(counts[can_id] for can_id in range(0x2A1, 0x2A9)):
        problems.append('确认后出现默认地址反馈，不能保证主臂单独在线')
    other_control = [can_id for can_id in range(0x150, 0x180)
                     if counts[can_id] and can_id not in SHIFTED_MASTER_IDS]
    if other_control:
        problems.append('确认后出现非目标控制帧：' + _ids(other_control))
    unexpected_enable = [payload for payload in window.enable_payloads
                         if payload[:2] != bytes((7, 2))]
    if unexpected_enable:
        problems.append('确认后出现非预期 0x471 载荷')
    if counts[ROLE_CONFIG_ID]:
        problems.append('确认后出现另一条 0x470 配置帧')
    if window.errors:
        problems.append(f'确认后 CAN 错误帧 {window.errors} 个')
    return problems


def _ids(can_ids) -> str:
    return ', '.join(f'0x{can_id:03X}' for can_id in can_ids)


def configure_role(port: str, role: str, seconds: float,
                   recover_shifted_master: bool = False) -> int:
    """Send one official role frame to one physically isolated arm."""
    if recover_shifted_master and role != 'master':
        print('启动时静默的旧偏移恢复只适用于 master。')
        return EXIT_REFUSED
    lease = CanWriterLease(port)
    bus = None
    role_byte = MASTER_ROLE if role == 'master' else FOLLOWER_ROLE
    payload = bytes((role_byte, 0, 0, 0, 0, 0, 0, 0))
    print(f'{port}：即将广播 0x470 {payload.hex(" ")}。'
          '另一台臂必须断电或断开 CAN。')
    try:
        lease.acquire()
        bus = can.Bus(interface='socketcan', channel=port,
                      receive_own_messages=False)
        if recover_shifted_master:
            print(f'{port}：开始最长 {seconds:g}s 启动观察；'
                  '请只重启已隔离的主臂，抓齐偏移帧后会提前结束。')
            problems = verify_shifted_master_startup(
                capture(bus, seconds, SHIFTED_STARTUP_MIN_COUNTS))
        else:
            problems = verify_isolated_arm(capture(bus, seconds), role)
        if problems:
            print('拒绝配置：' + '；'.join(problems))
            return EXIT_REFUSED
        if input(f'输入 CONFIGURE-{role.upper()} 确认隔离：').strip() != f'CONFIGURE-{role.upper()}':
            print('未确认，未发送。')
            return EXIT_REFUSED
        # The physical connection may have changed while the operator read
        # the prompt. Check a fresh window immediately before broadcasting.
        fresh = capture(bus, min(seconds, 3.0))
        problems = (verify_recovery_bus(fresh) if recover_shifted_master
                    else verify_isolated_arm(fresh, role))
        if problems:
            print('确认后总线状态已变化，拒绝配置：' + '；'.join(problems))
            return EXIT_REFUSED
        bus.send(can.Message(arbitration_id=ROLE_CONFIG_ID, data=payload,
                             is_extended_id=False))
        print('已发送一次角色配置。请断电重启，并重新抓取 CAN 验证；'
              '发送成功不代表角色已保存。')
        return EXIT_OK
    except (OSError, can.CanError) as exc:
        print(f'{port} 配置失败：{exc}')
        return EXIT_FAILED
    except EOFError:
        print('未取得操作员确认，未发送。')
        return EXIT_REFUSED
    finally:
        if bus is not None:
            bus.shutdown()
        lease.release()


def monitor(port: str, window_seconds: float, duration: float | None) -> int:
    """Audit each window while firmware directly controls the follower."""
    bus = None
    ready = False
    observed_problem = False
    print(f'{port}：只读监视官方 0xFA/0xFC 联动。先从臂上电，后主臂上电，'
          '等待固件联动输出。Ctrl-C 只退出监视，不会停臂或失能。')
    try:
        bus = can.Bus(interface='socketcan', channel=port,
                      receive_own_messages=False)
        deadline = None if duration is None else time.monotonic() + duration
        while True:
            remaining = None if deadline is None else deadline - time.monotonic()
            if remaining is not None and remaining <= 0:
                return EXIT_REFUSED if observed_problem or not ready else EXIT_OK
            seconds = min(window_seconds, remaining) if remaining is not None else window_seconds
            # A short final window cannot satisfy the minimum frame count.
            if seconds < 2.0:
                return EXIT_REFUSED if observed_problem or not ready else EXIT_OK
            window = capture(bus, seconds)
            problems = verify_native(window)
            control_missing = [problem for problem in problems
                               if problem.startswith('主臂联动控制帧缺失：')]
            other_problems = [problem for problem in problems
                              if problem not in control_missing]
            enable_frames = ', '.join(
                f'{payload.hex(" ")} x{count}'
                for payload, count in window.enable_payloads.most_common(3))
            if problems:
                observed_problem = observed_problem or (ready and bool(other_problems))
                if ready and control_missing and not other_problems:
                    print(f'{port} 曾确认联动；本窗口主臂控制帧不完整或未出现。'
                          '主臂静止与输出中断无法仅靠这一窗口区分。')
                    continue
                if not window.counts and not window.errors:
                    print(f'{port} 本窗口未收到任何帧；只读观察无法判定设备角色或原因。')
                else:
                    print(f'{port} 联动未就绪：' + '；'.join(problems))
                if enable_frames:
                    print(f'{port} 另见 0x471：{enable_frames}；只读无法判断来源。')
                continue
            ready = True
            rates = ' '.join(
                f'0x{can_id:03X}={window.counts[can_id] / seconds:.0f}Hz'
                for can_id in NATIVE_CONTROL_IDS)
            print(f'{port} 联动帧齐全：{rates}，CAN 错误帧 0；'
                  f'另见 0x471：{enable_frames or "无"}（只读无法判断来源）')
    except KeyboardInterrupt:
        print('\n监视结束；固件联动可能仍在运行，请用实体流程停臂。')
        return EXIT_OK
    except (OSError, can.CanError) as exc:
        print(f'{port} 监视失败：{exc}；固件联动可能仍在运行。')
        return EXIT_FAILED
    finally:
        if bus is not None:
            bus.shutdown()


def argument_parser() -> ArgumentParser:
    parser = ArgumentParser(description=__doc__)
    parser.add_argument('--side', choices=('left', 'right'), default='left')
    parser.add_argument('--port', help='SocketCAN 接口；默认 can_<side>')
    parser.add_argument('--window-seconds', type=float, default=3.0,
                        help='身份核验窗口，秒（默认 3）')
    parser.add_argument('--duration', type=float,
                        help='监视时长，秒；默认持续监视至 Ctrl-C')
    parser.add_argument('--configure-role', choices=('master', 'follower'),
                        help='离线配置官方角色，须单臂隔离')
    parser.add_argument('--isolated-arm', action='store_true',
                        help='确认总线上仅连接被配置的那台臂')
    parser.add_argument('--send', action='store_true',
                        help='仅与 --configure-role 同用，允许发送一次 0x470')
    parser.add_argument('--recover-shifted-master', action='store_true',
                        help='右主臂等旧偏移主臂仅在启动时发帧时使用；'
                             '观察窗口内须单独重启该主臂')
    return parser


def main(args=None) -> int:
    options = argument_parser().parse_args(args)
    if not math.isfinite(options.window_seconds) or options.window_seconds < 2.0:
        print('观察窗口必须为至少 2 秒的有限数值。')
        return EXIT_REFUSED
    if options.duration is not None and (not math.isfinite(options.duration)
                                         or options.duration < options.window_seconds):
        print('监视时长必须为有限数值，且至少覆盖一个完整观察窗口。')
        return EXIT_REFUSED
    port = options.port or f'can_{options.side}'
    if options.configure_role:
        if not options.send or not options.isolated_arm or options.duration is not None:
            print('角色配置需要 --send --isolated-arm，且不能设置 --duration。')
            return EXIT_REFUSED
        if options.recover_shifted_master and options.configure_role != 'master':
            print('启动时静默的旧偏移恢复只适用于 master。')
            return EXIT_REFUSED
        if options.recover_shifted_master and options.window_seconds < 20.0:
            print('启动恢复需要 --window-seconds 至少 20，以覆盖单臂重启。')
            return EXIT_REFUSED
        return configure_role(port, options.configure_role, options.window_seconds,
                              options.recover_shifted_master)
    if options.send or options.isolated_arm or options.recover_shifted_master:
        print('--send、--isolated-arm 和 --recover-shifted-master '
              '仅用于离线角色配置；监视始终只读。')
        return EXIT_REFUSED
    return monitor(port, options.window_seconds, options.duration)


if __name__ == '__main__':
    sys.exit(main())
