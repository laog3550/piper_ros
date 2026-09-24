#!/usr/bin/env python3
"""Drive a follower arm to match a master arm's pose, then follow it live."""

# 用途：主从遥操作。以 master 臂的物理姿态为准——follower 先平滑移动到与 master
# 相同的姿态，随后持续跟随 master 的实时姿态，结束时自动回到启动时的姿态。
#
# 三个关键设计：
#
# 1. 绝对映射，不是增量映射。follower 的目标就是 master 当前的关节角度，所以两臂
#    最终处于同一物理姿态。代价是初始姿态不同时 follower 要做一次较大的移动
#    （实测左臂 j6 差 66.7 度、j4 差 10.4 度），所以对齐阶段是必需的。
#
# 2. 对齐阶段用三次插值过渡。取 3t^2-2t^3，它在两端速度为零，因此起始和结束都
#    没有速度突变。对齐期间 master 的姿态被冻结为轨迹终点；若 master 被移动超过
#    阈值，会自动以新姿态重新对齐，避免对齐结束时突然跳过去。
#
# 3. 结束回位（默认开启）。home 是本程序启动时 follower 的姿态，自动记录、没有
#    配置参数。--duration 到时和 Ctrl-C 都会触发回位；回位用同一套三次插值，时长
#    由位移和回位速度算出，不设固定时长。回位期间再按一次 Ctrl-C 会立即停在原地。
#
# 安全提醒：Ctrl-C 之后机械臂仍在运动，这是本工具唯一「按了键还在动」的行为，
# 所以回位开始时必须打印醒目提示（见 _run_return_home）。--no-return-home 可以
# 完全关掉回位。
#
# 跟随阶段默认用 α-β 状态估计同时获得平滑位置和速度；速度供后级前馈使用，
# 关节速度上限仍由驱动器和运动平滑级共同约束。
#
# 默认是干跑：打印两臂姿态、所需对齐位移、回位计划，不发送任何内容。必须显式加
# --enable 才真正发布运动指令。

from argparse import ArgumentParser
from dataclasses import dataclass
import math
import time
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from piper_msgs.msg import PiperEnableStatusMsg
from piper.piper_feedback import (
    CUBIC_PEAK_FACTOR,
    DEFAULT_ALPHA_BETA_ALPHA,
    DEFAULT_ALPHA_BETA_BETA,
    DEFAULT_ALPHA_BETA_MAX_DT_S,
    DEFAULT_DEADBAND_DEG,
    DEFAULT_DEADBAND_SPEED_DEG_S,
    DEFAULT_FILTER_TAU_S,
    DEFAULT_GRIPPER_DEADBAND_M,
    DEFAULT_GRIPPER_EFFORT_NM,
    DEFAULT_GRIPPER_SCALE,
    DEFAULT_ONE_EURO_BETA,
    DEFAULT_ONE_EURO_D_CUTOFF_HZ,
    DEFAULT_ONE_EURO_MIN_CUTOFF_HZ,
    DEFAULT_RETURN_MAX_PEAK_DEG_S,
    DEFAULT_RETURN_SPEED_DEG_S,
    DEFAULT_SMOOTH_BANDWIDTH_RAD_S,
    DEFAULT_SMOOTH_MAX_ACCELERATION_DEG_S2,
    DEFAULT_SMOOTH_MAX_JERK_DEG_S3,
    DEFAULT_SMOOTH_MAX_VELOCITY_DEG_S,
    GRIPPER_OPEN_MAX_M,
    JOINT_COUNT,
    MEASURED_MAX_JOINT_SPD_DEG_S,
    AlphaBetaFilter,
    DeadbandGate,
    LowPassFilter,
    MotionSmoother,
    OneEuroFilter,
    align_targets,
    clamp_gripper,
    clamp_targets,
    limit_step,
    plan_return,
    scale_gripper,
)

JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6',
               'gripper']
SIDES = {
    'left': ('/joint_ctrl_cmd_left', '/joint_states_left',
             '/arm_enable_status_left', 'follower_left (can_fl)'),
    'right': ('/joint_ctrl_cmd_right', '/joint_states_right',
              '/arm_enable_status_right', 'follower_right (can_fr)'),
}
ENABLE_SERVICES = {'left': '/enable_srv_left', 'right': '/enable_srv_right'}
DEFAULT_MASTER_TOPIC = '/joint_states_single'
DEFAULT_SIDE = 'left'
# 对齐时长。位移不变时它直接决定速度：默认 4.0 秒是原先 8.0 秒的两倍速
# （实测 37 度的对齐位移峰值约 14 deg/s，仍低于实测跟随能力 86 deg/s）。
DEFAULT_ALIGN_SECONDS = 4.0
# 0 表示不做跳变限制。同步优先：任何对目标值的改动都会让 follower 与
# master 不同步，所以默认关闭；需要防异常跳变时才设非零值。
DEFAULT_MAX_STEP_DEG = 0.0
# 速度百分比：节点把它转发给 MotionCtrl_2，是「对整臂最大速度（3 rad/s ≈
# 172 deg/s）的缩放」，不是模式开关。100 = 不额外限速——2026-09-23 实测，
# 10% 时 follower 的持续速度被压到 17.2 deg/s 且必然跟不上快速拖动；
# 100% 时达到 45.6~86.2 deg/s 并跟着拖动速度走。遥操作要的就是跟随 master
# 原值，所以默认不给它再加一道软件限速。
DEFAULT_SPEED = 100
DEFAULT_RATE_HZ = 50.0
FILTERS = ('alpha-beta', 'one-euro', 'lowpass', 'none')
DEFAULT_FILTER = 'alpha-beta'
MIN_ALIGN_SECONDS = 1.0
# 对齐期间 master 若被移动超过这个量，且**已经停下**，就以新姿态重新对齐。
REALIGN_THRESHOLD_DEG = 2.0
# 重新对齐的次数上限。对齐的目标是启动时冻结的 master 快照，操作者一直握着
# master 拖动时它永远追不上——2026-09-23 实测：25 秒的会话里重新对齐 3 次，
# 最终模式仍是 align，follower 全程只在慢速追一个过时的快照，看起来就是
# "几乎不跟随"。所以：只有"碰了一下又停下"才重新对齐，而且有次数上限。
MAX_REALIGNS = 2
# 判定 master 仍在被拖动的速度阈值，deg/s。
MASTER_MOVING_DEG_S = 1.0
STATUS_PERIOD = 1.0
# 使能状态话题以 10 Hz 发布；比这更旧的状态不再能支撑「整臂使能」这个结论。
STATUS_TIMEOUT_S = 1.0

EXIT_OK = 0
EXIT_REFUSED = 1


class TeleopBridge(Node):
    """Read both arms and optionally publish aligned follower commands."""

    def __init__(self, side, master_topic):
        super().__init__('piper_teleop')
        self.cmd_topic, self.follower_topic, self.status_topic, self.arm = (
            SIDES[side])
        self.master_topic = master_topic
        self.master = None
        self.follower = None
        # 夹爪开口（米），来自各自消息的第 7 项；消息不足 7 项时保持 None，
        # 遥操作据此判断这一路能不能镜像。
        self.master_gripper = None
        self.follower_gripper = None
        self.status = None
        self.status_time = None
        self.create_subscription(JointState, master_topic,
                                 self._on_master, 10)
        self.create_subscription(JointState, self.follower_topic,
                                 self._on_follower, 10)
        self.create_subscription(PiperEnableStatusMsg, self.status_topic,
                                 self._on_status, 10)
        self.publisher = self.create_publisher(JointState, self.cmd_topic, 10)

    def _angles(self, msg):
        if len(msg.position) < JOINT_COUNT:
            return None
        values = [float(v) for v in msg.position[:JOINT_COUNT]]
        if not all(math.isfinite(v) for v in values):
            return None
        return {i + 1: math.degrees(v) for i, v in enumerate(values)}

    def _gripper(self, msg):
        """Read the gripper opening in metres, or None when it is absent."""
        if len(msg.position) < JOINT_COUNT + 1:
            return None
        value = float(msg.position[JOINT_COUNT])
        return value if math.isfinite(value) else None

    def _on_master(self, msg):
        angles = self._angles(msg)
        if angles is not None:
            self.master = angles
        gripper = self._gripper(msg)
        if gripper is not None:
            self.master_gripper = gripper

    def _on_follower(self, msg):
        angles = self._angles(msg)
        if angles is not None:
            self.follower = angles
        gripper = self._gripper(msg)
        if gripper is not None:
            self.follower_gripper = gripper

    def _on_status(self, msg):
        self.status = msg
        self.status_time = time.monotonic()

    def enable_error(self):
        """Return None when the arm is confirmed enabled, else why not."""
        if self.status is None:
            return f'{self.status_topic} 上没有收到使能状态'
        age = time.monotonic() - self.status_time
        if age > STATUS_TIMEOUT_S:
            # 使能状态自己也是反馈：话题停了就说明这一路已经掉线，此时
            # 不能把最后一条「已使能」当成现在的结论。
            return f'使能状态已过期（{age:.1f}s 没有更新）'
        if not self.status.all_enabled:
            return (f'整臂未确认使能（state={self.status.state} '
                    f'all_enabled={self.status.all_enabled}）')
        return None

    def spin_for(self, seconds):
        """Pump callbacks for a while."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)


def _publish_targets(node, targets_deg, speed, gripper=None, gripper_effort=0.0):
    """Publish one absolute joint target for all six joints (plus the gripper).

    ``gripper`` is the opening in metres, or None to keep the old behaviour of
    commanding 0 (which the node turns into a fully closed gripper).
    """
    command = JointState()
    command.name = list(JOINT_NAMES)
    command.position = [math.radians(targets_deg[joint])
                        for joint in range(1, JOINT_COUNT + 1)] + [
                            float(gripper) if gripper is not None else 0.0]
    # 第 7 项是速度百分比；非零才会走低速分支而不是 100%。
    command.velocity = [0.0] * JOINT_COUNT + [float(speed)]
    command.effort = [0.0] * JOINT_COUNT + [
        float(gripper_effort) if gripper is not None else 0.0]
    node.publisher.publish(command)


def _print_alignment_plan(master, follower, align_seconds,
                          master_gripper=None, follower_gripper=None,
                          gripper_scale=1.0):
    """Print the pose difference and the implied peak joint speeds."""
    print('两臂当前姿态与所需对齐位移（度）：')
    print(f'  {"joint":<7}{"master":>10}{"follower":>10}{"move":>10}'
          f'{"peak":>12}')
    worst = 0.0
    for joint in range(1, JOINT_COUNT + 1):
        delta = master[joint] - follower[joint]
        worst = max(worst, abs(delta))
        peak = abs(delta) / align_seconds * CUBIC_PEAK_FACTOR
        print(f'  j{joint:<6}{master[joint]:>10.3f}{follower[joint]:>10.3f}'
              f'{delta:>+10.3f}{peak:>10.2f} deg/s')
    if master_gripper is not None and follower_gripper is not None:
        # 夹爪单列一行，单位是毫米（它与关节的"度"不是一回事）。move 是 follower
        # 真正要走的量，所以按换算后的目标算。
        goal = scale_gripper(master_gripper, gripper_scale)
        print(f'  {"gripper":<7}{master_gripper * 1000:>10.2f}'
              f'{follower_gripper * 1000:>10.2f}'
              f'{(goal - follower_gripper) * 1000:>+10.2f}{"mm":>12}')
        if abs(gripper_scale - 1.0) > 1e-9:
            print(f'    （夹爪按 master × {gripper_scale:g} 换算：'
                  f'{master_gripper * 1000:.2f}mm → 目标 {goal * 1000:.2f}mm；'
                  f'从臂行程 0~{GRIPPER_OPEN_MAX_M * 1000:g}mm）')
    print(f'  最大位移 {worst:.3f} 度，在 {align_seconds:g}s 内完成，'
          f'插值峰值速度约 {worst / align_seconds * CUBIC_PEAK_FACTOR:.2f} deg/s')
    return worst


def _make_filter(options):
    """Build the follow-phase filter the operator asked for."""
    if options.filter == 'none':
        return None
    if options.filter == 'lowpass':
        return LowPassFilter(options.filter_tau)
    if options.filter == 'alpha-beta':
        return AlphaBetaFilter(options.alpha_beta_alpha,
                               options.alpha_beta_beta,
                               options.alpha_beta_max_dt)
    return OneEuroFilter(options.one_euro_min_cutoff, options.one_euro_beta,
                         options.one_euro_d_cutoff)


def _make_smoother(options):
    """Build the jerk-limited smoothing stage, or None when it is off."""
    if options.smooth_bandwidth <= 0.0:
        return None
    return MotionSmoother(options.smooth_bandwidth,
                          options.smooth_max_velocity,
                          options.smooth_max_acceleration,
                          options.smooth_max_jerk)


def _filter_note(options, prefix='跟随处理：越界钳制（始终启用），'):
    """Describe the follow-phase filtering in use."""
    if options.filter == 'none':
        note = '无滤波'
    elif options.filter == 'lowpass':
        note = f'一阶低通 τ={options.filter_tau:g}s'
    elif options.filter == 'alpha-beta':
        note = ('α-β 状态估计（'
                f'α={options.alpha_beta_alpha:g}、'
                f'β={options.alpha_beta_beta:g}、'
                f'最大间隔={options.alpha_beta_max_dt:g}s）')
    else:
        note = ('One Euro 滤波（'
                f'min_cutoff={options.one_euro_min_cutoff:g}Hz、'
                f'beta={options.one_euro_beta:g}、'
                f'd_cutoff={options.one_euro_d_cutoff:g}Hz）')
    if options.deadband_deg > 0.0:
        note += (f'，死区 {options.deadband_deg:g} 度'
                 f'（速度门限 {options.deadband_speed:g} deg/s）')
    if options.smooth_bandwidth > 0.0:
        note += (f'，五次插值平滑（带宽 '
                 f'{options.smooth_bandwidth:g}rad/s、加速度上限 '
                 f'{options.smooth_max_acceleration:g}deg/s²）')
    return prefix + note


@dataclass
class RunSummary:
    """How the following phase ended, for the closing report."""

    elapsed: float
    stopped_by: str
    mode: str
    realigns: int
    limit_hits: Dict[int, int]
    last_targets: Optional[Dict[int, float]]
    last_gripper: Optional[float] = None


def _run_teleop(node, options):
    """Align the follower to the master, then follow it until told to stop."""
    period = 1.0 / options.rate
    mode = 'align'
    started = time.monotonic()
    align_started = started
    align_from = dict(node.follower)
    align_goal = dict(node.master)
    previous = dict(align_from)
    limit_hits: Dict[int, int] = {}
    realigns = 0
    loop_count = 0
    last_report = started
    next_deadline = started
    last_cycle = None
    last_targets = None
    last_gripper = None
    last_master = None
    last_master_time = None
    smoother = _make_filter(options)
    deadband = DeadbandGate(options.deadband_deg, options.deadband_speed)
    fine = _make_smoother(options)
    # 夹爪走一条独立的轻通道：读 master 的第 7 项（米），限幅后作为 follower 的
    # 第 7 项。关节那一套滤波/平滑级不参与——它们的阈值与限幅都是按"度"定的，
    # 直接套到 0.08 m 量级的开口上会把它整段按住。跟随阶段只过一道幅度死区，
    # 用来挡掉夹爪反馈里 0.1 mm 量级的抖动。
    gripper_on = (bool(getattr(options, 'gripper', False))
                  and getattr(node, 'master_gripper', None) is not None
                  and getattr(node, 'follower_gripper', None) is not None)
    grip_gate = (DeadbandGate(options.gripper_deadband, 0.0)
                 if gripper_on else None)
    grip_scale = getattr(options, 'gripper_scale', DEFAULT_GRIPPER_SCALE)
    # master 的开口乘上比例、限幅之后才是 follower 的目标（比例是机构参数：本机上
    # 主臂夹爪行程只有从臂的约 1/1.3）。follower 自己的开口本来就是从臂单位，
    # 不参与换算——对齐的起点取它。
    grip_from = getattr(node, 'follower_gripper', None)
    grip_goal = (scale_gripper(node.master_gripper, grip_scale)
                 if gripper_on else None)
    grip_target = scale_gripper(grip_from) if gripper_on else None
    stopped_by = 'ROS 上下文已结束'

    try:
        while rclpy.ok():
            now = time.monotonic()
            loop_count += 1
            rclpy.spin_once(node, timeout_sec=0.0)
            if node.master is None or node.follower is None:
                time.sleep(period)
                continue
            dt = None if last_cycle is None else now - last_cycle
            last_cycle = now
            master_speed = 0.0
            if last_master is not None and last_master_time is not None:
                span = now - last_master_time
                if span > 0.0:
                    master_speed = max(
                        abs(node.master[joint] - last_master[joint])
                        for joint in range(1, JOINT_COUNT + 1)) / span
            last_master = dict(node.master)
            last_master_time = now

            if mode == 'align':
                progress = (now - align_started) / options.align_seconds
                targets = align_targets(align_from, align_goal, progress)
                if gripper_on:
                    # 与关节同一条三次曲线：两端速度为零，所以夹爪不会先跳一下。
                    grip_target = clamp_gripper(
                        align_targets({7: grip_from}, {7: grip_goal},
                                      progress)[7])
                if progress >= 1.0:
                    # 对齐期间 master 若被动过，目标已经过时。
                    drift = max(
                        abs(node.master[j] - align_goal[j])
                        for j in range(1, JOINT_COUNT + 1))
                    if drift > REALIGN_THRESHOLD_DEG:
                        if (master_speed < MASTER_MOVING_DEG_S
                                and realigns < MAX_REALIGNS):
                            realigns += 1
                            print(f'  master 在对齐期间移动了 {drift:.2f} 度、'
                                  f'现在已静止，以新姿态重新对齐'
                                  f'（第 {realigns} 次）')
                            align_started = now
                            align_from = dict(node.follower)
                            align_goal = dict(node.master)
                            if gripper_on:
                                grip_from = node.follower_gripper
                                grip_goal = scale_gripper(node.master_gripper,
                                                          grip_scale)
                            continue
                        print(f'  master 仍在被拖动（{master_speed:.1f} deg/s、'
                              f'已偏离对齐目标 {drift:.2f} 度）：不再重新对齐，'
                              f'直接进入跟随')
                    mode = 'follow'
                    # 用最后一次发布的目标给滤波器做种子，而不是 master 的当前
                    # 值：命令流保持连续，剩下的偏差交给跟随自己收敛掉（若用
                    # master 当前值做种子，两者差多少就会当场跳多少）。夹爪同理，
                    # 种子取最后一次发布的开口。
                    seed = dict(previous)
                    if grip_gate is not None:
                        seed[7] = (grip_target if grip_target is not None
                                   else node.master_gripper)
                    if smoother is not None:
                        smoother.reset(seed)
                    deadband.reset(dict(node.master))
                    if grip_gate is not None:
                        grip_gate.reset(
                            {7: scale_gripper(node.master_gripper,
                                              grip_scale)})
                    if fine is not None:
                        fine.reset(seed)
                    print('  对齐完成 -> 进入绝对跟随'
                          + _filter_note(options, prefix='，'))
            else:
                # 跟随阶段的处理顺序：死区 -> 状态估计 -> 五次插值平滑。
                # 死区丢掉小于阈值的输入变化（静止时目标完全不动）；状态估计
                # 压掉高频手抖；平滑级用固定带宽的三阶级联环限制加速度跳变，
                # 并用滤波器的速度估计做前馈，保证跟手不滞后。dt 一律传实测
                # 循环间隔而不是设定周期：实际周期会波动，用设定值会让截止频率
                # 跟着它一起变。
                cycle = 0.0 if dt is None else dt
                sample = deadband.update(node.master, cycle)
                if grip_gate is not None:
                    # 夹爪作为第 7 项并入同一条链，这样"对齐 → 跟随"切换与重新
                    # 对齐都不会让它跳变；只有死区阈值换成了米（0.08 m 的行程上
                    # 套 0.2 度的阈值等于永远不动）。
                    sample = dict(sample)
                    sample[7] = grip_gate.update(
                        {7: scale_gripper(node.master_gripper, grip_scale)},
                        cycle)[7]
                if smoother is not None:
                    sample = smoother.update(sample, cycle)
                if fine is not None:
                    sample = fine.update(sample, smoother.velocities()
                                         if smoother is not None else {}, cycle)
                targets = {joint: sample[joint]
                           for joint in range(1, JOINT_COUNT + 1)}
                if grip_gate is not None:
                    grip_target = clamp_gripper(sample[7])

            # 越界钳制是必需的：含越界目标的指令行为未定义。除此之外不再
            # 改动目标——同步只允许保留 master 的原值（外加默认开启的低通滤波）。
            targets, clamped = clamp_targets(targets)
            if options.max_step_deg > 0.0:
                targets, _ = limit_step(targets, previous,
                                        options.max_step_deg)
            for joint in clamped:
                limit_hits[joint] = limit_hits.get(joint, 0) + 1
            previous = targets
            last_targets = targets
            last_gripper = grip_target

            if options.enable:
                _publish_targets(node, targets, options.speed,
                                 gripper=grip_target,
                                 gripper_effort=options.gripper_effort)

            if now - last_report >= STATUS_PERIOD:
                # 实际频率直接决定跟随延迟。它低于 --rate 说明单次循环
                # 的处理耗时超过了周期，需要提高 --rate 或降低负载。
                actual_hz = loop_count / max(now - last_report, 1e-9)
                loop_count = 0
                last_report = now
                if mode == 'align':
                    headline = ('  [对齐] 进度 '
                                f'{(now - align_started) / options.align_seconds * 100:5.1f}%'
                                f'  {actual_hz:5.1f}Hz')
                else:
                    headline = f'  [跟随] {actual_hz:5.1f}Hz'
                print(headline + ' follower 目标：' + ' '.join(
                    f'j{j}={targets[j]:+8.3f}'
                    for j in range(1, JOINT_COUNT + 1))
                    + (f' 夹爪={grip_target * 1000:+8.2f}mm'
                       if grip_target is not None else ''))
                if limit_hits:
                    print('    !! 已触限位的关节：' + '、'.join(
                        f'j{j}({limit_hits[j]})'
                        for j in sorted(limit_hits)))
                if node.status is not None:
                    error = node.enable_error()
                    if error:
                        print(f'    !! follower 使能异常：{error}')

            if (options.duration is not None
                    and now - started >= options.duration):
                stopped_by = f'运行到 --duration {options.duration:g}s'
                break
            # 按累积截止时间休眠并扣除处理耗时，否则实际频率会低于设定，
            # 直接表现为跟随延迟。
            next_deadline += period
            left = next_deadline - time.monotonic()
            if left > 0:
                time.sleep(left)
            else:
                next_deadline = time.monotonic()
    except KeyboardInterrupt:
        stopped_by = 'Ctrl-C 中断'

    return RunSummary(
        elapsed=time.monotonic() - started,
        stopped_by=stopped_by,
        mode=mode,
        realigns=realigns,
        limit_hits=limit_hits,
        last_targets=last_targets,
        last_gripper=last_gripper,
    )


def _print_return_plan(start, home, plan, start_gripper=None,
                       home_gripper=None):
    """Print what the return move will do, before any of it happens."""
    print(f'  {"joint":<7}{"start":>10}{"home":>10}{"move":>10}{"peak":>10}')
    for joint in range(1, JOINT_COUNT + 1):
        delta = plan.deltas_deg[joint]
        peak = (CUBIC_PEAK_FACTOR * abs(delta) / plan.duration
                if plan.duration > 0.0 else 0.0)
        mark = '  !!' if joint in plan.clamped_deg else ''
        print(f'  j{joint:<6}{start[joint]:>10.3f}{home[joint]:>10.3f}'
              f'{delta:>+10.3f}{peak:>10.2f}{mark}')
    if start_gripper is not None and home_gripper is not None:
        # 夹爪跟着同一条三次曲线回去，它的移动量比关节小得多，不参与时长推导。
        print(f'  {"gripper":<7}{start_gripper * 1000:>10.2f}{home_gripper * 1000:>10.2f}'
              f'{(home_gripper - start_gripper) * 1000:>+10.2f}{"mm":>10}')
    if plan.duration <= 0.0:
        print('  起点与 home 已经相同，无需移动')
        return
    worst = max(plan.deltas_deg, key=lambda j: abs(plan.deltas_deg[j]))
    print(f'  最大位移 {plan.max_delta_deg:.3f} 度（j{worst}）→ 回位时长 '
          f'{plan.duration:.2f}s，平均 {plan.max_delta_deg / plan.duration:.2f} '
          f'deg/s，峰值 {plan.peak_deg_s:.2f} deg/s')
    if plan.clamped_deg:
        # 钳制意味着这一段指令偏离了计划路径，必须在使用前说清楚。
        print('  !! 会触及限位的关节：' + '、'.join(
            f'j{joint}(路径上最多被钳 {amount:.3f} 度)'
            for joint, amount in sorted(plan.clamped_deg.items())))
    if plan.peak_deg_s > MEASURED_MAX_JOINT_SPD_DEG_S:
        print(f'  注意：峰值 {plan.peak_deg_s:.2f} deg/s 高于实测跟随能力 '
              f'{MEASURED_MAX_JOINT_SPD_DEG_S:g} deg/s，'
              f'follower 会滞后于这条轨迹（终点仍会到达）')


def _follow_return_path(node, options, start, home, plan,
                        start_gripper=None, home_gripper=None):
    """Publish the cubic path home; returns False if it had to stop early."""
    period = 1.0 / options.rate
    started = time.monotonic()
    loop_count = 0
    last_report = started
    next_deadline = started
    return_gripper = (start_gripper is not None and home_gripper is not None
                      and getattr(options, 'gripper', False))
    while True:
        now = time.monotonic()
        loop_count += 1
        if rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
        # 回位前和回位中都要确认整臂使能：掉线之后继续发指令是盲目动作。
        error = node.enable_error()
        if error:
            print()
            print(f'  !! 回位中止：{error}；机械臂停在当前姿态，不再发指令')
            return False
        progress = (now - started) / plan.duration
        targets, clamped = clamp_targets(align_targets(start, home, progress))
        grip_target = None
        if return_gripper:
            grip_target = clamp_gripper(
                align_targets({7: start_gripper}, {7: home_gripper},
                              progress)[7])
        _publish_targets(node, targets, options.speed, gripper=grip_target,
                         gripper_effort=getattr(options, 'gripper_effort', 0.0))
        if now - last_report >= STATUS_PERIOD:
            actual_hz = loop_count / max(now - last_report, 1e-9)
            loop_count = 0
            last_report = now
            print(f'  [回位] {min(progress, 1.0) * 100:5.1f}% {actual_hz:5.1f}Hz '
                  'follower 目标：' + ' '.join(
                      f'j{j}={targets[j]:+8.3f}'
                      for j in range(1, JOINT_COUNT + 1))
                  + (f' 夹爪={grip_target * 1000:+8.2f}mm'
                     if grip_target is not None else ''))
            if clamped:
                print('    !! 回位目标被钳制：' + '、'.join(
                    f'j{j}({clamped[j]:+.3f})' for j in sorted(clamped)))
        if progress >= 1.0:
            return True
        # 与跟随阶段一致：按累积截止时间休眠并扣除处理耗时。
        next_deadline += period
        left = next_deadline - time.monotonic()
        if left > 0:
            time.sleep(left)
        else:
            next_deadline = time.monotonic()


def _run_return_home(node, options, home, summary, home_gripper=None):
    """Bring the follower back to the pose this run started from."""
    start = (summary.last_targets if summary.last_targets is not None
             else node.follower)
    if start is None:
        print('  回位中止：没有可用的 follower 姿态，不发送任何指令')
        return
    start_gripper = (summary.last_gripper if summary.last_gripper is not None
                     else getattr(node, 'follower_gripper', None))
    if not getattr(options, 'gripper', False):
        start_gripper = home_gripper = None
    plan = plan_return(start, home, options.return_speed,
                       options.return_max_peak)
    print()
    print('=' * 72)
    if options.enable:
        print(f'遥操作结束（{summary.stopped_by}）：正在回到初始位置，'
              f'预计 {plan.duration:.1f} 秒；再按一次 Ctrl-C 可立即停下')
    else:
        print(f'遥操作结束（{summary.stopped_by}）：干跑，以下是回位计划，'
              '不发送任何内容')
    print('=' * 72)
    if summary.last_targets is not None:
        print('  回位起点：最后一次发布的跟随目标'
              + ('（干跑时 follower 不会真的移动，用它预览回位计划）'
                 if not options.enable else ''))
    else:
        print('  回位起点：follower 的实测姿态')
    _print_return_plan(start, home, plan, start_gripper, home_gripper)

    if not options.enable:
        return
    error = node.enable_error()
    if error:
        print(f'  回位中止：{error}，不发送任何指令')
        return
    if plan.duration <= 0.0:
        print('  回位结束：follower 已经在初始姿态上，无需移动')
        return
    try:
        if _follow_return_path(node, options, start, home, plan,
                               start_gripper, home_gripper):
            print(f'  回位完成：{plan.duration:.1f}s 内走完轨迹，'
                  'follower 保持使能停在初始姿态')
            print('  如需失能：ros2 service call '
                  f'{ENABLE_SERVICES[options.side]} piper_msgs/srv/Enable '
                  '"{enable_request: false}"')
    except KeyboardInterrupt:
        print()
        print('  再次 Ctrl-C：已中止回位，机械臂停在原地（仍使能）')


def _parser():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument('--side', choices=tuple(SIDES), default=DEFAULT_SIDE,
                        help='要驱动的 follower（默认 %(default)s）')
    parser.add_argument('--master-topic', default=DEFAULT_MASTER_TOPIC,
                        help='master 臂关节角度话题（默认 %(default)s）')
    parser.add_argument('--align-seconds', type=float,
                        default=DEFAULT_ALIGN_SECONDS,
                        help='对齐阶段时长，秒（默认 %(default)s）')
    parser.add_argument('--max-step-deg', type=float,
                        default=DEFAULT_MAX_STEP_DEG,
                        help='可选：每周期目标最大变化，0 表示不限制（默认 %(default)s）')
    parser.add_argument('--speed', type=int, default=DEFAULT_SPEED,
                        help='follower 速度百分比 1-100，100 表示不额外限速'
                             '（默认 %(default)s）')
    parser.add_argument('--rate', type=float, default=DEFAULT_RATE_HZ,
                        help='发布频率 Hz（默认 %(default)s）')
    parser.add_argument('--duration', type=float, default=None,
                        help='可选：运行指定秒数后自动结束（随后回位）')
    parser.add_argument('--smooth-bandwidth', type=float,
                        default=DEFAULT_SMOOTH_BANDWIDTH_RAD_S,
                        help='五次插值平滑的带宽 rad/s；0 表示关掉这一级'
                             '（默认 %(default)s）')
    parser.add_argument('--smooth-max-velocity', type=float,
                        default=DEFAULT_SMOOTH_MAX_VELOCITY_DEG_S,
                        help='平滑级的速度上限 deg/s（默认 %(default)s）')
    parser.add_argument('--smooth-max-acceleration', type=float,
                        default=DEFAULT_SMOOTH_MAX_ACCELERATION_DEG_S2,
                        help='平滑级的加速度上限 deg/s²（默认 %(default)s）')
    parser.add_argument('--smooth-max-jerk', type=float,
                        default=DEFAULT_SMOOTH_MAX_JERK_DEG_S3,
                        help='平滑级的 jerk 上限 deg/s³（默认 %(default)s）')
    parser.add_argument('--deadband-speed', type=float,
                        default=DEFAULT_DEADBAND_SPEED_DEG_S,
                        help='死区的速度门限 deg/s：低于它才认为手停着；'
                             '0 表示只按幅度保持（默认 %(default)s）')
    parser.add_argument('--deadband-deg', type=float,
                        default=DEFAULT_DEADBAND_DEG,
                        help='跟随阶段的死区，度：小于它的输入变化不传递，'
                             '0 表示关闭（默认 %(default)s）')
    parser.add_argument('--no-gripper', dest='gripper', action='store_false',
                        help='不镜像夹爪：指令的第 7 项恒为 0（旧行为）。'
                             '默认镜像 master 的第 7 项')
    parser.add_argument('--gripper-scale', type=float,
                        default=DEFAULT_GRIPPER_SCALE,
                        help='夹爪行程倍数：follower 目标 = master 开口 × 该值，'
                             '1.0 表示纯镜像（默认 %(default)s）')
    parser.add_argument('--gripper-effort', type=float,
                        default=DEFAULT_GRIPPER_EFFORT_NM,
                        help='从臂夹爪的夹持力 N·m；节点会把它钳到 [0.5, 3]'
                             '（默认 %(default)s）')
    parser.add_argument('--gripper-deadband', type=float,
                        default=DEFAULT_GRIPPER_DEADBAND_M,
                        help='夹爪的幅度死区，米：小于它的变化不传递，'
                             '0 表示不设死区（默认 %(default)s，即 0.5mm）')
    parser.add_argument('--filter', choices=FILTERS, default=DEFAULT_FILTER,
                        help='跟随阶段的滤波方案（默认 %(default)s）：'
                             'alpha-beta 同时估计平滑位置与速度；'
                             'one-euro 自适应低通，压手抖且快拖不滞后；'
                             'lowpass 固定截止的一阶低通（对照用）；none 不滤波')
    parser.add_argument('--alpha-beta-alpha', type=float,
                        default=DEFAULT_ALPHA_BETA_ALPHA,
                        help='α-β：位置残差增益，范围 (0, 1]（默认 %(default)s）')
    parser.add_argument('--alpha-beta-beta', type=float,
                        default=DEFAULT_ALPHA_BETA_BETA,
                        help='α-β：速度残差增益，范围 [0, 1]（默认 %(default)s）')
    parser.add_argument('--alpha-beta-max-dt', type=float,
                        default=DEFAULT_ALPHA_BETA_MAX_DT_S,
                        help='α-β：超过该采样间隔就清零速度并重新锚定，秒'
                             '（默认 %(default)s）')
    parser.add_argument('--one-euro-min-cutoff', type=float,
                        default=DEFAULT_ONE_EURO_MIN_CUTOFF_HZ,
                        help='One Euro：静止时的截止频率 Hz（默认 %(default)s）')
    parser.add_argument('--one-euro-beta', type=float,
                        default=DEFAULT_ONE_EURO_BETA,
                        help='One Euro：截止频率随速度的增长，Hz per deg/s'
                             '（默认 %(default)s）')
    parser.add_argument('--one-euro-d-cutoff', type=float,
                        default=DEFAULT_ONE_EURO_D_CUTOFF_HZ,
                        help='One Euro：速度估计自身的截止频率 Hz（默认 %(default)s）')
    parser.add_argument('--filter-tau', type=float, default=DEFAULT_FILTER_TAU_S,
                        help='仅 --filter lowpass 使用：时间常数，秒（默认 %(default)s）')
    parser.add_argument('--return-speed', type=float,
                        default=DEFAULT_RETURN_SPEED_DEG_S,
                        help='回位平均速度 deg/s（默认 %(default)s）')
    parser.add_argument('--return-max-peak', type=float,
                        default=DEFAULT_RETURN_MAX_PEAK_DEG_S,
                        help='回位峰值速度上限 deg/s（默认 %(default)s）')
    parser.add_argument('--no-return-home', action='store_true',
                        help='结束时停在原地，不回到初始位置')
    parser.add_argument('--enable', action='store_true',
                        help='真正发布运动指令；不加此参数只做干跑')
    return parser


def main(args=None):
    """Dry-run, or align the follower to the master and then follow it."""
    options = _parser().parse_args(args)
    if options.align_seconds < MIN_ALIGN_SECONDS:
        print(f'拒绝：--align-seconds 不得小于 {MIN_ALIGN_SECONDS}')
        return EXIT_REFUSED
    if not 1 <= options.speed <= 100:
        print('拒绝：--speed 必须在 1..100')
        return EXIT_REFUSED
    if options.filter_tau < 0.0:
        print('拒绝：--filter-tau 不得为负（0 表示不滤波）')
        return EXIT_REFUSED
    if not 0.0 < options.alpha_beta_alpha <= 1.0:
        print('拒绝：--alpha-beta-alpha 必须在 (0, 1]')
        return EXIT_REFUSED
    if not 0.0 <= options.alpha_beta_beta <= 1.0:
        print('拒绝：--alpha-beta-beta 必须在 [0, 1]')
        return EXIT_REFUSED
    if options.alpha_beta_max_dt <= 0.0:
        print('拒绝：--alpha-beta-max-dt 必须为正')
        return EXIT_REFUSED
    if options.deadband_deg < 0.0:
        print('拒绝：--deadband-deg 不得为负（0 表示关闭死区）')
        return EXIT_REFUSED
    if options.deadband_speed < 0.0:
        print('拒绝：--deadband-speed 不得为负（0 表示只按幅度保持）')
        return EXIT_REFUSED
    if options.gripper_scale <= 0.0:
        print('拒绝：--gripper-scale 必须为正（1.0 表示纯镜像）')
        return EXIT_REFUSED
    if options.gripper_effort < 0.0:
        print('拒绝：--gripper-effort 不得为负（0 表示最弱夹持）')
        return EXIT_REFUSED
    if options.gripper_deadband < 0.0:
        print('拒绝：--gripper-deadband 不得为负（0 表示不设死区）')
        return EXIT_REFUSED
    if options.smooth_bandwidth < 0.0:
        print('拒绝：--smooth-bandwidth 不得为负（0 表示关闭平滑级）')
        return EXIT_REFUSED
    if options.smooth_max_velocity <= 0.0 \
            or options.smooth_max_acceleration <= 0.0 \
            or options.smooth_max_jerk <= 0.0:
        print('拒绝：平滑级的三个上限都必须为正')
        return EXIT_REFUSED
    if options.return_speed <= 0.0 or options.return_max_peak <= 0.0:
        print('拒绝：--return-speed 与 --return-max-peak 必须为正')
        return EXIT_REFUSED

    # 自己接管 SIGINT（SignalHandlerOptions.NO → Python 默认行为：抛
    # KeyboardInterrupt）。rclpy 默认的 SIGINT 处理会在 Ctrl-C 时立刻关闭上下文，
    # 之后发布和订阅都不再工作——而回位恰恰发生在 Ctrl-C 之后，必须还能发指令、
    # 还能读使能状态。代价是 SIGTERM(kill) 不再触发回位，它是立即终止。
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = TeleopBridge(options.side, options.master_topic)
    try:
        node.spin_for(3.0)
        print(f'follower 目标话题：{node.cmd_topic}（{node.arm}）')
        print(f'master 角度话题  ：{node.master_topic}')
        if node.master is None:
            print(f'拒绝：{node.master_topic} 上没有收到 master 的关节角度')
            return EXIT_REFUSED
        if node.follower is None:
            print(f'拒绝：{node.follower_topic} 上没有收到 follower 的角度')
            return EXIT_REFUSED
        error = node.enable_error()
        if options.enable and error:
            print(f'拒绝：{error}，拒绝发布运动指令')
            return EXIT_REFUSED
        # home 就是本程序启动时 follower 的姿态，自动记录，不需要配置参数。
        home = dict(node.follower)
        home_gripper = node.follower_gripper

        print()
        _print_alignment_plan(node.master, node.follower,
                              options.align_seconds,
                              node.master_gripper, node.follower_gripper,
                              options.gripper_scale)
        print()
        print(f'模式：先在 {options.align_seconds:g}s 内三次插值对齐，'
              f'再绝对跟随 master；{options.rate:g} Hz，速度 {options.speed}%'
              + ('' if options.enable else '（干跑，不发送任何内容）'))
        print(_filter_note(options))
        if options.gripper:
            if node.master_gripper is None or node.follower_gripper is None:
                print('夹爪      ：话题里没有第 7 项，本次不镜像夹爪')
            else:
                print(f'夹爪      ：镜像 master 第 7 项 × {options.gripper_scale:g}'
                      f'——限幅 0~{GRIPPER_OPEN_MAX_M * 1000:g}mm、死区 '
                      f'{options.gripper_deadband * 1000:g}mm、夹持力 '
                      f'{options.gripper_effort:g}N·m')
                print(f'            当前 master={node.master_gripper * 1000:.2f}mm'
                      f'（→目标 '
                      f'{scale_gripper(node.master_gripper, options.gripper_scale) * 1000:.2f}mm）'
                      f'、follower={node.follower_gripper * 1000:.2f}mm')
            print('            要让夹爪指令真正生效，从臂节点需以 '
                  'gripper_exist:=true 启动')
        else:
            print('夹爪      ：不镜像（--no-gripper），指令第 7 项恒为 0')
        if options.no_return_home:
            return_note = '不回位，停在原地（--no-return-home）'
        else:
            return_note = (f'回到启动姿态 home，平均 {options.return_speed:g} '
                           f'deg/s、峰值上限 {options.return_max_peak:g} deg/s'
                           f'（时长由位移算出）')
        print('结束时  ：' + return_note)
        print('注意：对齐期间请不要触碰 master，否则会自动重新对齐。')

        summary = _run_teleop(node, options)

        print()
        print(f'结束：运行 {summary.elapsed:.1f}s（{summary.stopped_by}），'
              f'最终模式 {summary.mode}，重新对齐 {summary.realigns} 次'
              + ('，未发送任何内容（干跑）' if not options.enable else ''))
        if summary.limit_hits:
            print('  触限位统计：' + '、'.join(
                f'j{j} {summary.limit_hits[j]} 次'
                for j in sorted(summary.limit_hits)))
        else:
            print('  触限位统计：无')

        if options.no_return_home:
            print('  回位：已禁用（--no-return-home），follower 停在原地')
        else:
            _run_return_home(node, options, home, summary, home_gripper)
        return EXIT_OK
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())

