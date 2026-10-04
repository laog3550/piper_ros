#!/usr/bin/env python3
"""Enable or disable the motors on whole CAN buses (CAN 0x471)."""

# 用途：退出示教模式（0x470）后臂会回到**使能**状态，伺服把住当前姿态、手拖不动。
# 这个工具用官方协议的 0x471（电机使能指令）把总线上的臂显式失能，失能后臂才回到
# 「靠人力搬得动」的状态。
#
# **0x471 是广播帧、帧里没有臂地址**：在一条总线上发一次，那条总线上的**两台臂
# 都会失能**（现在左右各一条总线，每条挂该侧的主臂与从臂）。所以「给所有机械臂
# 失能」= 每条总线发一帧，两条就够，与总线上挂了几台臂无关。
#
# 失能后臂靠重力：原本被伺服撑住的关节会下坠，需要用手托住——这是失能固有的后果，
# 不是故障。反过来，使能会让臂刚性把住当前位置。
#
# 默认干跑：只打印将要发送的帧与影响范围，**不打开任何套接字**。加 --send 才真正
# 发送，发送后用只读套接字读回 0x261~0x266 的使能位确认结果。

import time
from argparse import ArgumentParser
from typing import List

import can

from piper.piper_enable_check import collect
from piper.piper_enable_check import report as report_enable_state
from piper.piper_enable_status import EnableState, aggregate
from piper.piper_feedback import JOINT_COUNT

ENABLE_DISABLE_CAN_ID = 0x471
MOTOR_NUM_ALL = JOINT_COUNT + 1  # 7 = 所有电机
DISABLE_FLAG = 0x01
ENABLE_FLAG = 0x02

DEFAULT_PORTS = ('can_left', 'can_right')
DEFAULT_VERIFY_SECONDS = 2.0
DEFAULT_TIMEOUT = 0.5
FRAME_GAP_S = 0.2

EXIT_OK = 0
EXIT_UNCONFIRMED = 1
EXIT_FAILED = 3


def build_payload(enable: bool) -> bytes:
    """Build the 0x471 payload: motor number, flag, then padding."""
    flag = ENABLE_FLAG if enable else DISABLE_FLAG
    return bytes([MOTOR_NUM_ALL, flag] + [0] * 6)


def send_frame(port: str, payload: bytes) -> None:
    """Send one 0x471 frame on one bus; the only transmit path in this tool."""
    bus = can.Bus(
        interface='socketcan', channel=port, receive_own_messages=False,
    )
    try:
        bus.send(can.Message(
            arbitration_id=ENABLE_DISABLE_CAN_ID,
            data=payload,
            is_extended_id=False,
        ))
    finally:
        bus.shutdown()


def _describe_action(enable: bool) -> str:
    return '使能' if enable else '失能'


def _dry_run(ports: List[str], enable: bool) -> int:
    """Print exactly what would be sent, without touching any bus."""
    payload = build_payload(enable)
    action = _describe_action(enable)
    print(f'干跑：将要发送 {action} 帧（CAN '
          f'0x{ENABLE_DISABLE_CAN_ID:03X}），共 {len(ports)} 条总线')
    print(f'  载荷: {" ".join(f"{byte:02X}" for byte in payload)}'
          f'（电机号 {MOTOR_NUM_ALL}=全部，标志 0x{payload[1]:02X}）')
    for port in ports:
        print(f'  {port}: 这条总线上的臂都会{action}'
              f'（0x471 是广播，无臂地址）')
    if not enable:
        print('注意：失能后臂会因重力下坠，原来被伺服撑住的关节需要托住。')
    print('没有发送任何帧；确认无误后加 --send 执行。')
    return EXIT_OK


def _verify(port: str, seconds: float, timeout: float) -> EnableState:
    """Read the enable bits back and return the aggregate verdict."""
    tracker = collect(port, seconds, timeout)
    report_enable_state(port, tracker)
    return aggregate(tracker.observations(time.monotonic()))


def _execute(ports: List[str], enable: bool, verify_seconds: float,
             timeout: float) -> int:
    """Send the frame on every bus, then read the enable bits back."""
    payload = build_payload(enable)
    action = _describe_action(enable)
    print(f'发送 {action} 帧（CAN 0x{ENABLE_DISABLE_CAN_ID:03X}）：'
          f'{" ".join(f"{byte:02X}" for byte in payload)}')
    if not enable:
        print('注意：失能后臂会因重力下坠，原来被伺服撑住的关节需要托住。')
    for index, port in enumerate(ports):
        if index:
            time.sleep(FRAME_GAP_S)
        try:
            send_frame(port, payload)
        except (OSError, can.CanError) as exc:
            print(f'错误：{port} 上发送失败：{exc}')
            return EXIT_FAILED
        print(f'  已发送 → {port}')

    if verify_seconds <= 0:
        print('已按 --verify-seconds 0 跳过读回验证')
        return EXIT_OK

    expected = EnableState.ENABLED if enable else EnableState.DISABLED
    time.sleep(FRAME_GAP_S)
    print(f'读回验证（每条约 {verify_seconds:.1f}s）：')
    print('  注意：现场固件的 0x261~0x266 不随 0x20 偏移；同侧两台臂的样本'
          '混在一起，下面结果不能区分实体来源')
    confirmed = True
    for port in ports:
        try:
            verdict = _verify(port, verify_seconds, timeout)
        except (OSError, can.CanError) as exc:
            print(f'错误：{port} 上读回失败：{exc}')
            return EXIT_FAILED
        if verdict is not expected:
            confirmed = False
    print('=' * 60)
    if confirmed:
        print(f'结果：所有总线都读到 {expected.name}，{action}成功')
        return EXIT_OK
    print(f'结果：至少一条总线没有读到 {expected.name}')
    print('      两种常见原因：帧被丢掉（重跑一次本命令即可，重复发送无害），'
          '或总线上两台臂状态不一致（读回会是 PARTIAL）')
    return EXIT_UNCONFIRMED


def _parser() -> ArgumentParser:
    parser = ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group()
    group.add_argument(
        '--enable', action='store_true',
        help='使能（让臂刚性把住当前位置）',
    )
    group.add_argument(
        '--disable', action='store_true',
        help='失能（默认动作：不写任何标志就是失能）',
    )
    parser.add_argument(
        '--port', action='append', dest='ports', metavar='IFACE',
        help=f'要操作的 CAN 接口，可重复；默认 {" ".join(DEFAULT_PORTS)}',
    )
    parser.add_argument(
        '--send', action='store_true',
        help='真正发送帧（不加则只干跑打印）',
    )
    parser.add_argument(
        '--verify-seconds', type=float, default=DEFAULT_VERIFY_SECONDS,
        help='发送后读回验证的时长，秒；0 表示跳过验证（默认 %(default)s）',
    )
    parser.add_argument(
        '--timeout', type=float, default=DEFAULT_TIMEOUT,
        help='读回时接受的最大反馈帧龄，秒（默认 %(default)s）',
    )
    return parser


def main(args=None) -> int:
    """Run the enable/disable tool, dry by default."""
    options = _parser().parse_args(args)
    ports: List[str] = options.ports or list(DEFAULT_PORTS)
    if options.verify_seconds < 0:
        print('错误：--verify-seconds 不能为负')
        return EXIT_FAILED
    if options.timeout <= 0:
        print('错误：--timeout 必须为正')
        return EXIT_FAILED
    if not options.send:
        return _dry_run(ports, options.enable)
    return _execute(ports, options.enable, options.verify_seconds,
                    options.timeout)


if __name__ == '__main__':
    raise SystemExit(main())
