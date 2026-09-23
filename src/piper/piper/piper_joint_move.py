#!/usr/bin/env python3
"""Send one small joint-space move to a Piper follower arm."""

# 默认只做干跑，必须显式加 --send 才发布。发布前会强制校验所有关节的目标值是否
# 落在可指令范围内：越界的目标不会被驱动器拒绝，而是被钳制到最近的边界，从而把
# 「保持这个关节不动」变成一次真实运动。2026-09-23 的首次运动测试就是这样让
# joint2 动了 2.38°、joint3 动了 1.57°。
#
# 因此本工具在发现任何关节越界时默认拒绝发送，必须显式加 --allow-clamped，
# 并会列出预计的非预期位移，由操作者判断是否可接受。

from argparse import ArgumentParser
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from piper_msgs.msg import PiperEnableStatusMsg
from piper.piper_feedback import (
    JOINT_COMMAND_LIMITS_DEG,
    JOINT_COUNT,
    out_of_limits,
)

NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper']
# 左右从臂的接口名按 config/pi05_can_map.json：follower_left=can_fl,
# follower_right=can_fr。角色一律以 config 的序列号为准——接口名在
# 2026-09-24 被重新绑定过一次，按名字反推角色不可靠。
SIDES = {
    'left': ('/joint_ctrl_cmd_left', '/joint_states_left',
             '/arm_enable_status_left', 'follower_left, can_fl'),
    'right': ('/joint_ctrl_cmd_right', '/joint_states_right',
              '/arm_enable_status_right', 'follower_right, can_fr'),
}
DEFAULT_SIDE = 'left'
DEFAULT_TARGET = 'joint1'
# 归位时把越界关节放到距限位边界多远的内侧，留出伺服静差的余量。
NORMALIZE_MARGIN_DEG = 0.2
DEFAULT_DELTA_DEG = 0.5
DEFAULT_SPEED = 2
MAX_ABS_ANGLE = 3.5

EXIT_OK = 0
EXIT_REFUSED = 1


class Mover(Node):
    """Read follower feedback, build one joint command, maybe publish."""

    def __init__(self, side=DEFAULT_SIDE):
        super().__init__('piper_joint_move')
        self.cmd_topic, self.feedback_topic, self.status_topic, self.arm = (
            SIDES[side])
        self.feedback = None
        self.enable_status = None
        self.history = {}
        self.create_subscription(JointState, self.feedback_topic,
                                 self._on_feedback, 10)
        self.create_subscription(PiperEnableStatusMsg, self.status_topic,
                                 self._on_status, 10)
        self.publisher = self.create_publisher(JointState, self.cmd_topic, 10)

    def _on_feedback(self, msg):
        if len(msg.position) < JOINT_COUNT:
            return
        self.feedback = [float(v) for v in msg.position[:JOINT_COUNT]]
        for index, value in enumerate(self.feedback):
            low, high = self.history.get(index, (value, value))
            self.history[index] = (min(low, value), max(high, value))

    def _on_status(self, msg):
        self.enable_status = msg

    def spin_for(self, seconds):
        """Pump callbacks for a while."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)


def _check_feedback(node):
    """Return the six current angles in degrees, or an error string."""
    if node.feedback is None:
        return None, f'{node.feedback_topic} 上没有收到关节反馈'
    degrees = {}
    for index, value in enumerate(node.feedback):
        if not math.isfinite(value):
            return None, f'joint{index + 1} 的角度不是有限值: {value}'
        if abs(value) > MAX_ABS_ANGLE:
            return None, (f'joint{index + 1} 的角度 {value:.4f} rad 超出 '
                          f'±{MAX_ABS_ANGLE}，拒绝以此为基准')
        degrees[index + 1] = math.degrees(value)
    return degrees, None


def _describe_limits(current_deg, target_deg):
    """Print each joint's angle against its commandable range."""
    print('关节可指令范围校验：')
    overshoot = out_of_limits(target_deg)
    for joint in range(1, JOINT_COUNT + 1):
        low, high = JOINT_COMMAND_LIMITS_DEG[joint]
        note = '在范围内'
        if joint in overshoot:
            clamped = max(low, min(target_deg[joint], high))
            note = (f'!! 超出 {overshoot[joint]:.3f}°，'
                    f'会被钳到 {clamped:+.3f}°（相对当前位移 '
                    f'{clamped - current_deg[joint]:+.3f}°）')
        print(f'  joint{joint}: 当前 {current_deg[joint]:+8.3f}°  '
              f'目标 {target_deg[joint]:+8.3f}°  '
              f'范围 [{low:g}, {high:g}]  {note}')
    return overshoot


def _parser():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument('--send', action='store_true',
                        help='真正发布指令；不加此参数只做干跑')
    parser.add_argument('--side', choices=tuple(SIDES), default=DEFAULT_SIDE,
                        help='左臂还是右臂（默认 %(default)s）')
    parser.add_argument('--target', default=DEFAULT_TARGET,
                        choices=NAMES[:JOINT_COUNT],
                        help='要移动的关节（默认 %(default)s）')
    parser.add_argument('--delta-deg', type=float, default=None,
                        help='相对当前值的移动幅度，度（默认 %(default)s）')
    parser.add_argument('--to-deg', type=float, default=None,
                        help='把目标关节设到这个绝对角度（与其余模式互斥）')
    parser.add_argument('--normalize', action='store_true', default=None,
                        help='把所有越界关节一次带进限位内，其余关节保持当前值')
    parser.add_argument('--speed', type=int, default=DEFAULT_SPEED,
                        help='速度百分比 1-100（默认 %(default)s）')
    parser.add_argument('--watch', type=float, default=6.0,
                        help='发送后观察秒数（默认 %(default)s）')
    parser.add_argument('--allow-clamped', action='store_true',
                        help='明知有越界关节会导致非预期运动，仍继续发送')
    return parser


def main(args=None):
    """Dry-run, or actually send one small joint move."""
    options = _parser().parse_args(args)
    if not 1 <= options.speed <= 100:
        print('拒绝：--speed 必须在 1..100')
        return EXIT_REFUSED

    rclpy.init()
    node = Mover(options.side)
    try:
        node.spin_for(3.0)
        current_deg, error = _check_feedback(node)
        if error:
            print(f'拒绝发送：{error}')
            return EXIT_REFUSED

        exclusive = [name for name, value in (
            ('--delta-deg', options.delta_deg), ('--to-deg', options.to_deg),
            ('--normalize', options.normalize)) if value is not None
            and value is not False]
        if len(exclusive) > 1:
            print(f'拒绝：{" 与 ".join(exclusive)} 不能同时使用')
            return EXIT_REFUSED

        target_deg = dict(current_deg)
        delta = (DEFAULT_DELTA_DEG if options.delta_deg is None
                 else options.delta_deg)
        if options.normalize:
            # 一次把所有越界关节都带进限位内，因为只归一个关节时，其余越界
            # 关节仍会让整条指令失效。
            for joint in sorted(out_of_limits(current_deg)):
                low, high = JOINT_COMMAND_LIMITS_DEG[joint]
                if current_deg[joint] < low:
                    target_deg[joint] = low + NORMALIZE_MARGIN_DEG
                else:
                    target_deg[joint] = high - NORMALIZE_MARGIN_DEG
        else:
            index = NAMES.index(options.target)
            target_deg[index + 1] = (
                options.to_deg if options.to_deg is not None
                else current_deg[index + 1] + delta
            )

        print(f'目标话题：{node.cmd_topic}（{node.arm}）')
        if options.normalize:
            moved = sorted(
                j for j in target_deg
                if abs(target_deg[j] - current_deg[j]) > 1e-9
            )
            print('归位模式：把越界关节 ' +
                  '、'.join(f'j{j}' for j in moved) +
                  f' 带进限位内（距边界 {NORMALIZE_MARGIN_DEG}°），'
                  f'速度 {options.speed}%')
        elif options.to_deg is not None:
            print(f'要移动的关节：{options.target} 设为绝对角度 '
                  f'{options.to_deg:+.3f}°，速度 {options.speed}%')
        else:
            print(f'要移动的关节：{options.target} '
                  f'{delta:+.3f}°，速度 {options.speed}%')
        print()
        overshoot = _describe_limits(current_deg, target_deg)

        if overshoot and not options.allow_clamped:
            print()
            print('拒绝发送：上述越界关节会被钳制到边界，'
                  '从而产生非预期的真实运动。')
            print('处置方式：先把机械臂摆到所有关节都落在可指令范围内的姿态；'
                  '若确知并接受该位移，可加 --allow-clamped 继续。')
            return EXIT_REFUSED

        if not options.send:
            print()
            print('干跑结束：未发送任何内容。确认无误后加 --send 执行。')
            return EXIT_OK

        error = _check_enabled(node)
        if error:
            print(f'拒绝发送：{error}')
            return EXIT_REFUSED

        command = JointState()
        command.name = list(NAMES)
        command.position = [math.radians(target_deg[joint])
                            for joint in range(1, JOINT_COUNT + 1)] + [0.0]
        # 第 7 项是速度百分比；它非零，节点才会走低速分支而不是 100%。
        command.velocity = [0.0] * JOINT_COUNT + [float(options.speed)]
        command.effort = [0.0] * (JOINT_COUNT + 1)

        print()
        print('整臂已确认 ENABLED，发布 1 条指令 ...')
        node.publisher.publish(command)
        node.spin_for(options.watch)

        print()
        print(f'发送后各关节的实际变化（{options.watch:g}s 内）：')
        for index in range(JOINT_COUNT):
            low, high = node.history.get(index, (float('nan'),) * 2)
            final = (math.degrees(node.feedback[index])
                     if node.feedback else float('nan'))
            print(f'  joint{index + 1}: 起始 {current_deg[index + 1]:+8.3f}°  '
                  f'最小 {math.degrees(low):+8.3f}°  '
                  f'最大 {math.degrees(high):+8.3f}°  '
                  f'末值 {final:+8.3f}°  '
                  f'位移 {final - current_deg[index + 1]:+7.4f}°')
        return EXIT_OK
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _check_enabled(node):
    """Return None when the arm is fully enabled, else an error string."""
    status = node.enable_status
    if status is None:
        return f'{node.status_topic} 上没有收到使能状态'
    if not status.all_enabled:
        return (f'整臂未确认使能（state={status.state} '
                f'all_enabled={status.all_enabled}），拒绝发送运动指令')
    return None


if __name__ == '__main__':
    raise SystemExit(main())
