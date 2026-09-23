#!/usr/bin/env python3
"""Survey one CAN bus read-only to confirm a healthy Piper arm is on it."""

# 本工具为纯观测：以接收-only 方式打开套接字，按反馈帧过滤，代码中没有任何
# 发送路径。它自行解码原始帧，不依赖 piper_sdk，因此可以用来佐证控制节点，
# 而不是复述节点的话。
#
# 它回答的问题是「这条接口上到底连着什么」：Piper 的核心反馈帧是否齐全、
# 帧率是否正常、关节角度与供电是否合理、以及反馈 ID 是否已被偏移成示教输入
# 臂的布局。

import time
from argparse import ArgumentParser
from collections import Counter
from typing import Dict, Optional, Tuple

import can

from piper.piper_feedback import (
    CORE_FEEDBACK_CAN_IDS,
    END_POSE_CAN_IDS,
    FEEDBACK_CAN_IDS,
    GRIPPER_FEEDBACK_CAN_ID,
    HIGH_SPEED_CAN_IDS,
    JOINT_ANGLE_IDS,
    JOINT_COUNT,
    decode,
    decode_joint_angles,
    detect_feedback_offset,
)

DEFAULT_PORTS = ('can_fl', 'can_mr', 'can_fr', 'can_ml')
DEFAULT_DURATION = 2.0

EXIT_OK = 0
EXIT_INCOMPLETE = 1
EXIT_FAILED = 3

# 核心帧的标称帧率，仅用于判断「是否在流」，不作为严格断言。
MIN_FRAMES = 10


def _listen(port: str) -> can.BusABC:
    """Open a receive-only view of one CAN interface."""
    return can.Bus(
        interface='socketcan',
        channel=port,
        receive_own_messages=False,
    )


def survey(port: str, duration: float):
    """Watch one interface and return (frame counts, newest payloads)."""
    counter: Counter = Counter()
    last: Dict[int, bytes] = {}
    bus = _listen(port)
    try:
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            frame = bus.recv(
                timeout=min(0.2, max(0.0, deadline - time.monotonic())))
            if frame is None:
                continue
            counter[frame.arbitration_id] += 1
            last[frame.arbitration_id] = bytes(frame.data)
    finally:
        bus.shutdown()
    return counter, last


def _angle_summary(last: Dict[int, bytes]) -> str:
    angles: Dict[int, float] = {}
    for can_id in JOINT_ANGLE_IDS:
        if can_id in last:
            angles.update(decode_joint_angles(can_id, last[can_id]))
    if not angles:
        return '无角度帧'
    return ' '.join(
        f'j{joint}={angles[joint]:+.3f}°' for joint in sorted(angles)
    )


def _enable_summary(last: Dict[int, bytes]) -> str:
    parts = []
    for joint in range(1, JOINT_COUNT + 1):
        can_id = FEEDBACK_CAN_IDS[joint - 1]
        feedback = decode(can_id, last[can_id]) if can_id in last else None
        parts.append(
            f'j{joint}={"ON" if feedback.enabled else "off"}'
            if feedback else f'j{joint}=?'
        )
    return ' '.join(parts)


def _voltage_summary(last: Dict[int, bytes]) -> str:
    parts = []
    for joint in range(1, JOINT_COUNT + 1):
        can_id = FEEDBACK_CAN_IDS[joint - 1]
        feedback = decode(can_id, last[can_id]) if can_id in last else None
        if feedback:
            parts.append(f'j{joint}={feedback.voltage:.1f}V')
    return ' '.join(parts) if parts else '无低压帧'


def report(port: str, counter: Counter, last: Dict[int, bytes],
           duration: float) -> Tuple[bool, Optional[int]]:
    """Print one bus survey and return (core frames complete, offset)."""
    print(f'--- {port} ---')
    if not counter:
        print('  总线上没有任何帧')
        return False, None

    missing = [c for c in CORE_FEEDBACK_CAN_IDS if counter.get(c, 0) < MIN_FRAMES]
    offset = detect_feedback_offset(counter)
    unknown = sorted(
        c for c in counter
        if c not in CORE_FEEDBACK_CAN_IDS
        and (offset is None or c - offset not in CORE_FEEDBACK_CAN_IDS)
    )

    print(f'  帧总数 {sum(counter.values())} / {duration:.1f}s，'
          f'不同 ID {len(counter)} 个')
    if offset is None:
        print('  反馈 ID 未偏移 → 常规（运动输出臂）布局')
    else:
        shifted = sorted(c for c in counter if c - offset in CORE_FEEDBACK_CAN_IDS)
        print(f'  *** 反馈 ID 整体偏移 0x{offset:02X}，该臂处于示教输入臂布局 ***')
        print(f'      {len(shifted)} 个核心帧以 0x{offset:02X} 偏移出现，'
              f'例如 0x{shifted[0]:03X} 对应 0x{shifted[0] - offset:03X}')
        print('      常规布局的核心帧在这个偏移下不可能同时对上，故判定无歧义')

    print(f'  核心帧 {len(CORE_FEEDBACK_CAN_IDS) - len(missing)}/'
          f'{len(CORE_FEEDBACK_CAN_IDS)} 齐全'
          + (f'，缺失 {[hex(c) for c in missing]}' if missing else ''))
    if unknown:
        print(f'  未在 piper_sdk 定义中的额外 ID：{[hex(c) for c in unknown]}')
        for can_id in unknown:
            payload = last[can_id]
            print(f'      0x{can_id:03X} x{counter[can_id]} '
                  f'({counter[can_id] / duration:.0f} Hz) 载荷 {payload.hex(" ")}')

    print(f'  关节角度  : {_angle_summary(last)}')
    print(f'  母线电压  : {_voltage_summary(last)}')
    print(f'  使能位    : {_enable_summary(last)}')
    print(f'  高速反馈  : {sum(counter.get(c, 0) for c in HIGH_SPEED_CAN_IDS)} 帧'
          f'，末端位姿 {sum(counter.get(c, 0) for c in END_POSE_CAN_IDS)} 帧'
          f'，夹爪 {counter.get(GRIPPER_FEEDBACK_CAN_ID, 0)} 帧')
    return not missing, offset


def _parser() -> ArgumentParser:
    parser = ArgumentParser(description=__doc__)
    parser.add_argument(
        '--port', action='append', dest='ports', metavar='IFACE',
        help=f'要普查的 CAN 接口（默认 {" ".join(DEFAULT_PORTS)}）',
    )
    parser.add_argument(
        '--duration', type=float, default=DEFAULT_DURATION,
        help='每条接口抓取时长，秒（默认 %(default)s）',
    )
    return parser


def main(args=None) -> int:
    """Survey each interface without transmitting on any of them."""
    options = _parser().parse_args(args)
    ports = options.ports or list(DEFAULT_PORTS)
    if options.duration <= 0:
        print('错误：--duration 必须为正')
        return EXIT_FAILED

    print('只读普查：不会发送任何帧（无使能、失能或运动指令）')
    incomplete = []
    for port in ports:
        try:
            counter, last = survey(port, options.duration)
        except (OSError, can.CanError) as exc:
            print(f'错误：无法监听 {port}: {exc}')
            return EXIT_FAILED
        complete, _ = report(port, counter, last, options.duration)
        if not complete:
            incomplete.append(port)

    print('=' * 60)
    if incomplete:
        print(f'结果：以下接口的核心反馈帧不齐全：{incomplete}')
        return EXIT_INCOMPLETE
    print(f'结果：{len(ports)} 条接口的 Piper 核心反馈帧全部齐全')
    return EXIT_OK


if __name__ == '__main__':
    raise SystemExit(main())
