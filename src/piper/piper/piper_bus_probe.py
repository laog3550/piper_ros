#!/usr/bin/env python3
"""Survey one CAN bus read-only to see which Piper arms report on it."""

# 本工具为纯观测：以接收-only 方式打开套接字，接收所有 CAN 帧，代码中没有任何
# 发送路径。它自行解码原始帧，不依赖 piper_sdk，因此可以用来佐证控制节点，
# 而不是复述节点的话。
#
# 它回答的问题是「这条接口上到底连着什么」：Piper 的核心反馈帧是否齐全、
# 帧率是否正常、关节角度与供电是否合理、以及反馈 ID 是否已被偏移成示教输入
# 臂的布局。
#
# 现在一条总线上同时挂着该侧的主臂与从臂，判读方式随之改变：
#   * 核心帧齐全不再等于「只有一台臂」——两台都用常规布局时，同一批 ID 上
#     会有两台在报，高速反馈帧率约为单台的两倍，本工具会把这种情况标出来；
#   * 反馈 ID 偏移只能证明地址布局，不能据此判定 0xFA/0xFC 角色；
#   * 官方 0xFA 输入臂可能只发送控制帧，没有反馈。接口静默不能直接判为断线；
#   * 状态帧（0x2A1）第 0 字节只表示采到的反馈控制模式，不等于角色读回。

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
    LINKAGE_TEACHING_MODE,
    commanding_can_ids,
    decode,
    decode_ctrl_mode,
    decode_joint_angles,
    detect_feedback_offset,
)

DEFAULT_PORTS = ('can_left', 'can_right')
DEFAULT_DURATION = 2.0

EXIT_OK = 0
EXIT_INCOMPLETE = 1
EXIT_FAILED = 3

# 核心帧的标称帧率，仅用于判断「是否在流」，不作为严格断言。
MIN_FRAMES = 10

# 单台臂的高速反馈标称帧率（见 docs/PI05_CAN_MAPPING.md）。两台常规布局的臂
# 挂在同一条总线上时，同一批 ID 的帧率会接近它的两倍。
NOMINAL_HIGH_SPEED_HZ = 200.0
TWO_ARM_RATIO = 1.6

# 控制模式（0x2A1 第 0 字节）的显示名，取自 SDK 的 CtrlMode 枚举。
CTRL_MODE_NAMES = {
    0x00: '待机',
    0x01: 'CAN 指令控制',
    0x02: '示教',
    0x03: '以太网控制',
    0x04: 'WiFi 控制',
    0x05: '遥控',
    LINKAGE_TEACHING_MODE: '联动示教输入',
    0x07: '离线轨迹',
}


def _mode_summary(modes: Counter) -> str:
    """Name each observed control mode with its frame count."""
    parts = []
    for mode, count in sorted(modes.items()):
        name = CTRL_MODE_NAMES.get(mode, '未定义')
        parts.append(f'0x{mode:02X} {name} x{count}')
    return '，'.join(parts)


def _listen(port: str) -> can.BusABC:
    """Open a receive-only view of one CAN interface."""
    return can.Bus(
        interface='socketcan',
        channel=port,
        receive_own_messages=False,
    )


def survey(port: str, duration: float):
    """Watch one interface and return (counts, newest payloads, modes)."""
    counter: Counter = Counter()
    last: Dict[int, bytes] = {}
    modes: Counter = Counter()
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
            mode = decode_ctrl_mode(frame.arbitration_id, frame.data)
            if mode is not None:
                modes[mode] += 1
    finally:
        bus.shutdown()
    return counter, last, modes


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


def _high_speed_ratio(counter: Counter, duration: float) -> Optional[float]:
    """Return frames/nominal if a second arm likely shares these frame IDs."""
    nominal = len(HIGH_SPEED_CAN_IDS) * NOMINAL_HIGH_SPEED_HZ * duration
    if nominal <= 0:
        return None
    observed = sum(counter.get(c, 0) for c in HIGH_SPEED_CAN_IDS)
    ratio = observed / nominal
    return ratio if ratio >= TWO_ARM_RATIO else None


def report(port: str, counter: Counter, last: Dict[int, bytes],
           modes: Counter, duration: float) -> Tuple[bool, Optional[int]]:
    """Print one bus survey and return (core frames complete, offset)."""
    print(f'--- {port} ---')
    if not counter:
        print('  本接口未收到任何帧；无法仅凭此判断设备角色或 CAN 接线')
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
        print('  反馈 ID 未偏移 → 常规反馈布局；不能单凭偏移判断角色')
    else:
        shifted = sorted(c for c in counter if c - offset in CORE_FEEDBACK_CAN_IDS)
        print(f'  *** 反馈 ID 整体偏移 0x{offset:02X}；不能据此判定 0xFA 角色 ***')
        print(f'      {len(shifted)} 个核心帧以 0x{offset:02X} 偏移出现，'
              f'例如 0x{shifted[0]:03X} 对应 0x{shifted[0] - offset:03X}')
        print('      只能确认反馈地址布局，不能确认发送臂的角色')

    print(f'  核心帧 {len(CORE_FEEDBACK_CAN_IDS) - len(missing)}/'
          f'{len(CORE_FEEDBACK_CAN_IDS)} 齐全'
          + (f'，缺失 {[hex(c) for c in missing]}' if missing else ''))
    if unknown:
        print(f'  未在 piper_sdk 定义中的额外 ID：{[hex(c) for c in unknown]}')
        for can_id in unknown:
            payload = last[can_id]
            print(f'      0x{can_id:03X} x{counter[can_id]} '
                  f'({counter[can_id] / duration:.0f} Hz) 载荷 {payload.hex(" ")}')

    commanding = commanding_can_ids(counter)
    if commanding:
        print(f'  *** 总线上有控制指令帧 {[hex(c) for c in commanding]}：'
              f'有东西在给臂下发指令 ***')
        print('      可能是主机节点（piper_two_can_teleop / piper_single_ctrl / '
              '工作区里的执行器），也可能是**示教输入臂**在播发关节指令。')
        print('      若为固件联动控制，停臂应按设备实体流程；'
              '运行中不要发送角色配置。')
    else:
        print('  控制指令  : 观察窗口内未收到指令帧')

    print(f'  关节角度  : {_angle_summary(last)}')
    print(f'  母线电压  : {_voltage_summary(last)}')
    print(f'  使能位    : {_enable_summary(last)}')
    if modes:
        print(f'  控制模式  : {_mode_summary(modes)}')
        if len(modes) > 1:
            print('      窗口内出现多种模式；可能是状态切换或多台臂，需隔离核验')
            if LINKAGE_TEACHING_MODE in modes:
                print('      0x06 是反馈控制模式，不等于已读回 0xFA 角色')
        elif LINKAGE_TEACHING_MODE in modes:
            print('      采到 0x06 控制模式；不能据此确认角色或臂的数量')
    else:
        print('  控制模式  : 无状态帧')
    print(f'  高速反馈  : {sum(counter.get(c, 0) for c in HIGH_SPEED_CAN_IDS)} 帧'
          f'，末端位姿 {sum(counter.get(c, 0) for c in END_POSE_CAN_IDS)} 帧'
          f'，夹爪 {counter.get(GRIPPER_FEEDBACK_CAN_ID, 0)} 帧')
    ratio = _high_speed_ratio(counter, duration)
    if ratio is not None:
        print(f'  *** 高速反馈帧率约为单台臂的 {ratio:.1f} 倍；'
              '可能有多个同 ID 发送源，需隔离核验 ***')
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
            counter, last, modes = survey(port, options.duration)
        except (OSError, can.CanError) as exc:
            print(f'错误：无法监听 {port}: {exc}')
            return EXIT_FAILED
        complete, _ = report(port, counter, last, modes, options.duration)
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
