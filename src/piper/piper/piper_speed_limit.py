#!/usr/bin/env python3
"""Read and set the driver-flash joint speed ceiling of Piper arms."""

# 用途：调整驱动器 flash 里的 max_joint_spd（关节限速）。出厂值是 300，即
# 0.3 rad/s ≈ 17.2 deg/s；遥操作跟随阶段 follower 追不上 master，卡的就是它。
#
# 默认干跑：逐台读取当前限位，打印将要写入的内容，不写任何东西。加 --send 才真正
# 写入，写完立即逐台读回验证。
#
# 三条硬规则：
#
# 1. 写入时三个字段（角度上限、角度下限、限速）必须全部显式给值。0x7FFF
#    （"不修改"）只有 V1.5-2 之后的固件支持，旧固件会把它读成 32.767 rad/s。
# 2. 未修改的字段一律用该台臂自己读回的原值填充，不照抄另一台臂的值——四台臂
#    出厂一致只是运气，照抄会把本来不同的限位覆盖掉。
# 3. 全程不使能、不发运动指令。写入本身不产生运动，但它会改变"越界指令被钳到
#    哪里"，所以要求该臂先失能；没确认失能时本工具拒绝写入。
#
# 读取会发送 0x472 查询帧（驱动只回一帧 0x473），除此之外不发任何帧。使能位来自
# 各臂自己周期广播的低压反馈帧，是完全被动的观测。
#
# 实测（2026-09-23，can_fl）：驱动一次只答一条查询。6 条查询以 20ms 连发时只回来
# 4 条（另外两条的答复丢失，其中一条的电机号字段是 0），连发更快时一条都不回。
# 所以这里逐条查询、每条等到答复再发下一条，答复没来就重发一次。

import math
import time
from argparse import ArgumentParser
from typing import Callable, Dict, Optional, Tuple

import can

from piper.piper_feedback import (
    JOINT_COUNT,
    LIMIT_QUERY_CAN_ID,
    LIMIT_SET_CAN_ID,
    MAX_JOINT_SPD_RAW,
    JointLimit,
    decode,
    decode_joint_limit,
    encode_limit_query,
    encode_limit_set,
)

DEFAULT_PORTS = ('can_fl', 'can_mr', 'can_fr', 'can_ml')
# 1500 = 1.5 rad/s ≈ 86 deg/s，是全部 3000（3 rad/s ≈ 172 deg/s）的一半。
DEFAULT_SPEED_RAW = 1500
# 帧间隔：24 条写入帧挤在一起会挤掉同一条总线上控制节点的反馈帧。
FRAME_GAP_S = 0.02
# 单条查询的等待时间。实测答复在 3ms 内到达，这里留了两个数量级的余量。
REPLY_TIMEOUT_S = 0.4
# 使能位要看各臂自己的广播节奏，单独给一段纯听窗口。
ENABLE_LISTEN_S = 2.0

EXIT_OK = 0
EXIT_REFUSED = 1
EXIT_FAILED = 3

# 各接口的失能提示。can_ml/can_mr 是 master，通常用 start_single_piper.launch.py
# 启动，enable 服务没被重映射；两条从臂在 start_two_piper.launch.py 里重映射过。
ENABLE_HINTS = {
    'can_fl': '/enable_srv_left',
    'can_fr': '/enable_srv_right',
}
DEFAULT_ENABLE_HINT = '/enable_srv'


def _open(port: str) -> can.BusABC:
    """Open one CAN interface without taking part in its traffic."""
    return can.Bus(
        interface='socketcan',
        channel=port,
        receive_own_messages=False,
    )


def _absorb(frame, limits: Dict[int, JointLimit],
            enabled: Dict[int, bool]) -> None:
    """Record whatever one incoming frame carries."""
    limit = decode_joint_limit(frame.arbitration_id, frame.data)
    if limit is not None:
        limits[limit.joint] = limit
        return
    feedback = decode(frame.arbitration_id, frame.data)
    if feedback is not None:
        enabled[feedback.joint] = feedback.enabled


def _listen(bus: can.BusABC, seconds: float,
            stop: Optional[Callable[[Dict, Dict], bool]] = None
            ) -> Tuple[Dict[int, JointLimit], Dict[int, bool]]:
    """Listen for a while, or until ``stop`` says there is nothing to wait for."""
    limits: Dict[int, JointLimit] = {}
    enabled: Dict[int, bool] = {}
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if stop is not None and stop(limits, enabled):
            break
        frame = bus.recv(timeout=0.05)
        if frame is None:
            continue
        _absorb(frame, limits, enabled)
    return limits, enabled


def read_state(bus: can.BusABC
               ) -> Tuple[Dict[int, JointLimit], Dict[int, Optional[bool]]]:
    """Query one arm's limits and observe its per-joint enable bits."""
    limits: Dict[int, JointLimit] = {}
    enabled: Dict[int, bool] = {}
    for joint in range(1, JOINT_COUNT + 1):
        for _ in range(2):  # 答复丢了就重发一次查询
            bus.send(can.Message(arbitration_id=LIMIT_QUERY_CAN_ID,
                                 data=encode_limit_query(joint),
                                 is_extended_id=False))
            found, seen = _listen(bus, REPLY_TIMEOUT_S,
                                  lambda limits_, _: joint in limits_)
            limits.update(found)
            enabled.update(seen)
            if joint in limits:
                break
    if len(enabled) < JOINT_COUNT:
        # 使能位只在各臂自己周期广播的低压反馈帧里，错过就再纯听一段。
        found, seen = _listen(
            bus, ENABLE_LISTEN_S,
            lambda _, enabled_: len(enabled_) >= JOINT_COUNT)
        limits.update(found)
        enabled.update(seen)
    return limits, enabled


def write_limits(bus: can.BusABC, wanted: Dict[int, JointLimit]) -> None:
    """Write one limit frame per joint, with every field given explicitly."""
    for joint in sorted(wanted):
        bus.send(can.Message(arbitration_id=LIMIT_SET_CAN_ID,
                             data=encode_limit_set(wanted[joint]),
                             is_extended_id=False))
        time.sleep(FRAME_GAP_S)


def _speed_text(raw: int) -> str:
    """Render a raw speed limit as raw, rad/s and deg/s."""
    return (f'{raw}（{raw * 0.001:.3f} rad/s ≈ '
            f'{math.degrees(raw * 0.001):.1f} deg/s）')


def _print_readout(port: str, before: Dict[int, JointLimit],
                   enabled: Dict[int, Optional[bool]]) -> None:
    """Print one arm's limits and enable bits as read."""
    print(f'--- {port} ---')
    print(f'  {"joint":<7}{"min":>9}{"max":>9}{"speed":>8}{"enable":>9}')
    for joint in range(1, JOINT_COUNT + 1):
        limit = before.get(joint)
        if limit is None:
            print(f'  j{joint:<6}{"??":>9}{"??":>9}{"??":>8}{"??":>9}')
            continue
        flag = enabled.get(joint)
        state = 'unknown' if flag is None else ('ON' if flag else 'off')
        print(f'  j{joint:<6}{limit.min_angle_deg:>9.1f}'
              f'{limit.max_angle_deg:>9.1f}{limit.max_joint_spd:>8}'
              f'{state:>9}')


def _enabled_problem(enabled: Dict[int, Optional[bool]]) -> Optional[str]:
    """Return why the arm may not be written to, or None when it is disabled."""
    if len(enabled) < JOINT_COUNT:
        missing = [f'j{joint}' for joint in range(1, JOINT_COUNT + 1)
                   if joint not in enabled]
        return f'没有收到 {"、".join(missing)} 的低压反馈帧，无法确认已失能'
    on = [f'j{joint}' for joint in sorted(enabled) if enabled[joint]]
    if on:
        return '该臂仍在使能（' + '、'.join(on) + '）'
    return None


def _verify(port: str, wanted: Dict[int, JointLimit],
            before: Dict[int, JointLimit],
            after: Dict[int, JointLimit]) -> bool:
    """Check the read-back matches the plan and the angle limits survived."""
    problems = []
    for joint in range(1, JOINT_COUNT + 1):
        read_back = after.get(joint)
        if read_back is None:
            problems.append(f'j{joint} 没有读回')
            continue
        if read_back.max_joint_spd != wanted[joint].max_joint_spd:
            problems.append(f'j{joint} 限速读回 {read_back.max_joint_spd}，'
                            f'期望 {wanted[joint].max_joint_spd}')
        if (abs(read_back.max_angle_deg - before[joint].max_angle_deg) > 1e-9
                or abs(read_back.min_angle_deg
                       - before[joint].min_angle_deg) > 1e-9):
            problems.append(
                f'j{joint} 角度限位被改动：'
                f'[{before[joint].min_angle_deg:g}, '
                f'{before[joint].max_angle_deg:g}] -> '
                f'[{read_back.min_angle_deg:g}, '
                f'{read_back.max_angle_deg:g}]')
    if problems:
        print(f'  !! {port} 写入后验证失败：')
        for problem in problems:
            print(f'     - {problem}')
        return False
    print(f'  {port} 验证通过：6/6 关节限速读回一致，角度限位与写入前相同')
    return True


def _parser() -> ArgumentParser:
    parser = ArgumentParser(description=__doc__)
    parser.add_argument('--send', action='store_true',
                        help='真正写入驱动 flash；不加此参数只读取并打印计划')
    parser.add_argument('--port', action='append', dest='ports', metavar='IFACE',
                        help=f'要处理的 CAN 接口（默认 {" ".join(DEFAULT_PORTS)}）')
    parser.add_argument('--spd', type=int, default=DEFAULT_SPEED_RAW,
                        help='新的最大关节速度，单位 0.001 rad/s，1..3000'
                             '（默认 %(default)s）')
    parser.add_argument('--allow-enabled', action='store_true',
                        help='该臂未确认失能时仍然写入（默认拒绝）')
    return parser


def main(args=None) -> int:
    """Read every arm's limits, and write the new speed ceiling with --send."""
    options = _parser().parse_args(args)
    ports = options.ports or list(DEFAULT_PORTS)
    if not 1 <= options.spd <= MAX_JOINT_SPD_RAW:
        print(f'拒绝：--spd 必须在 1..{MAX_JOINT_SPD_RAW}')
        return EXIT_REFUSED

    print(f'目标限速：{_speed_text(options.spd)}')
    if options.send:
        print('写入模式：不改动角度限位（用各台臂自己读回的原值填充），'
              '不发使能指令、不发运动指令')
    else:
        print('干跑：只读取并打印计划，不写入任何内容'
              '（读取会发送 0x472 查询帧，不发其他帧）')
    print()

    originals: Dict[str, Dict[int, JointLimit]] = {}
    for port in ports:
        try:
            bus = _open(port)
        except (OSError, can.CanError) as exc:
            print(f'--- {port} ---')
            print(f'  错误：无法打开 {port}: {exc}')
            return EXIT_FAILED
        try:
            before, enabled = read_state(bus)
            _print_readout(port, before, enabled)
            missing = [joint for joint in range(1, JOINT_COUNT + 1)
                       if joint not in before]
            if missing:
                print(f'  错误：没有读到关节 {missing} 的限位，'
                      f'停止处理（不做部分写入）')
                return EXIT_FAILED
            print('  计划：' + '、'.join(
                f'j{joint} {before[joint].max_joint_spd} -> {options.spd}'
                for joint in range(1, JOINT_COUNT + 1)))
            if not options.send:
                continue

            problem = _enabled_problem(enabled)
            if problem and not options.allow_enabled:
                print(f'  拒绝写入：{problem}。写入本身不产生运动，但会改变越界'
                      f'指令被钳制的位置，可能在控制器正在发指令时改变行为。')
                print(f'  先失能再执行：ros2 service call '
                      f'{ENABLE_HINTS.get(port, DEFAULT_ENABLE_HINT)} '
                      f'piper_msgs/srv/Enable "{{enable_request: false}}"')
                print('  确认可以带使能写入时才加 --allow-enabled。')
                return EXIT_REFUSED

            # 未修改的字段用该台臂自己读回的原值填充。
            wanted = {
                joint: JointLimit(joint=joint,
                                  max_angle_deg=before[joint].max_angle_deg,
                                  min_angle_deg=before[joint].min_angle_deg,
                                  max_joint_spd=options.spd)
                for joint in range(1, JOINT_COUNT + 1)
            }
            print(f'  写入 {JOINT_COUNT} 条 0x474 帧 ...')
            write_limits(bus, wanted)
            after, _ = read_state(bus)
            if not _verify(port, wanted, before, after):
                return EXIT_FAILED
            originals[port] = before
        finally:
            bus.shutdown()
        print()

    if not options.send:
        print('干跑结束：未写入任何内容。上面每台臂读回的限速就是它的当前值；'
              '确认无误后加 --send 执行。')
        return EXIT_OK

    print('=' * 60)
    print('回滚：把限速改回各台写入前的原值（角度限位原样保留）')
    for port in ports:
        values = {before[joint].max_joint_spd
                  for joint in originals.get(port, {})}
        if len(values) != 1:
            print(f'  {port}: 写入前各关节限速不一致 {sorted(values)}，'
                  f'无法用单条命令回滚，请逐关节核对')
            continue
        original = values.pop()
        print(f'  # {port}: 写入前为 {_speed_text(original)}')
        print(f'  piper_speed_limit --port {port} --spd {original} --send')
    print('说明：回滚只改限速，角度限位取自回滚那一刻读到的值，'
          '所以它不会把 2026-09-23 放宽过的 joint2/joint3 限位一起改回去。')
    return EXIT_OK


if __name__ == '__main__':
    raise SystemExit(main())
