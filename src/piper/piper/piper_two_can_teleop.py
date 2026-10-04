#!/usr/bin/env python3
"""Stream one master arm's pose to its follower over a single shared CAN."""

# 用途：两 CAN 主机桥接摇操——同一条总线上既有主臂（反馈 ID 带 0x20 偏移）也有从臂
# （默认地址），上位机读主臂、算目标、只向从臂标准地址发指令。

# 设计要点（与 docs/TWO_CAN_HOST_TELEOP_IMPLEMENTATION.md 对应）：
#
# 1. 一条 SocketCAN 上同时有主臂和从臂：主臂反馈带 0x20 偏移（0x2C1～0x2C8），
#    从臂使用默认地址（0x2A1～0x2A8）。上位机只向从臂标准地址发送
#    0x151/0x155～0x157/0x159，从不发送 0x170 或 0x470。
# 2. 实时环不经过 ROS 话题：按命令频率读取最新六关节快照，经跟随链处理后直接写回
#    同一条总线。ROS 只用于启停服务、状态和诊断（见 piper_two_can_manager）。
# 3. 进入与退出都走显式状态机。进入：只读身份与使能门禁 → 整侧使能读回 → 操作员
#    确认主臂已进入实体示教 → 受限 minimum-jerk 对齐 → 跟随。退出：停目标流 →
#    从臂切回待机 → 操作员确认实体示教已退出 → 整侧失能读回。故障一律锁存，
#    本进程不再自动进入摇操。
# 4. 默认干跑：不加 --send 只执行只读门禁并打印对齐计划，不发送任何 CAN 帧。
#    即使加了 --send，也必须显式给出 --workspace-clear（人员和线缆退出运动空间）。

from argparse import ArgumentParser
from collections import defaultdict
from dataclasses import dataclass
from enum import Enum
import math
import signal
import sys
import time
from typing import Dict, Optional, Tuple

import can

from piper.piper_feedback import (
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
    DEFAULT_SMOOTH_BANDWIDTH_RAD_S,
    DEFAULT_SMOOTH_MAX_ACCELERATION_DEG_S2,
    DEFAULT_SMOOTH_MAX_JERK_DEG_S3,
    DEFAULT_SMOOTH_MAX_VELOCITY_DEG_S,
    JOINT_COUNT,
    AlphaBetaFilter,
    DeadbandGate,
    LowPassFilter,
    MotionSmoother,
    OneEuroFilter,
    clamp_gripper,
    clamp_targets,
    decode,
    limit_step,
    scale_gripper,
)
from piper.piper_two_can_align import (
    DEFAULT_ERROR_POLICY,
    DEFAULT_GRIPPER_STALE_S,
    DEFAULT_MAX_ERROR_RATE_HZ,
    DEFAULT_MAX_GOAL_CLAMP_DEG,
    DEFAULT_MAX_SKEW_S,
    DEFAULT_PREFLIGHT_SECONDS,
    DEFAULT_STALE_S,
    EMERGENCY_STOP,
    ENABLE_CAN_ID,
    ERROR_POLICIES,
    MOTION_CTRL_1_ID,
    AlignmentPlan,
    GripperRamp,
    PreflightReport,
    RuntimeMonitor,
    SafetyError,
    capture_preflight,
    encode_motion_ctrl_1,
    plan_alignment,
    print_plan,
    print_preflight,
    run_alignment,
    send_standby,
    send_target,
    socket_filters,
)

DEFAULT_COMMAND_RATE_HZ = 200.0
# 速度百分比是「对整臂最大速度的缩放」，不是模式开关。旧四 CAN 方案实测 100 才能
# 跟上手动拖动（10% 时被压到 17.2 deg/s），所以默认不给从臂再加软件限速。
DEFAULT_SPEED_PERCENT = 100
DEFAULT_ARM_VERIFY_S = 1.0
DEFAULT_DISABLE_VERIFY_S = 2.0
DEFAULT_DISABLE_RETRIES = 2
DEFAULT_START_TIMEOUT_S = 300.0
DEFAULT_TEACH_TIMEOUT_S = 300.0
DEFAULT_TEACH_RELEASE_TIMEOUT_S = 300.0
DEFAULT_ALIGN_RATE_HZ = 50.0
DEFAULT_ALIGN_SPEED_PERCENT = 5
DEFAULT_MAX_DELTA_DEG = 30.0
DEFAULT_MIN_ALIGN_S = 2.0
DEFAULT_MAX_PEAK_DEG_S = 5.0
DEFAULT_SETTLE_S = 1.0
DEFAULT_GOAL_TOLERANCE_DEG = 0.5
DEFAULT_MASTER_DRIFT_DEG = 0.5
DEFAULT_ALIGN_FOLLOW_ERROR_DEG = 3.0
# 跟随阶段的跟踪误差门禁：从臂滞后于目标本身是正常的（驱动器加速度有限），
# 所以只有**持续**超限才算故障，默认 10°、持续 1 s。
DEFAULT_TRACKING_ERROR_DEG = 10.0
DEFAULT_TRACKING_ERROR_GRACE_S = 1.0
DEFAULT_MAX_STEP_DEG = 0.0
DEFAULT_FILTER = 'alpha-beta'
FILTERS = ('alpha-beta', 'one-euro', 'lowpass', 'none')
# 故障路径里的 0x150 急停帧：官方定义为 MotionCtrl_1(emergency_stop=0x01)，但现场
# 尚未单独验收它的行为，所以默认不发；确认过行为后再用 --fault-fast-stop 打开。
DEFAULT_FAULT_FAST_STOP = False
STATUS_PERIOD_S = 1.0

EXIT_OK = 0
EXIT_REFUSED = 2
EXIT_FAILED = 3
EXIT_PENDING_RELEASE = 4


class TeleopState(str, Enum):
    """One side's teleop session state, matching section 6 of the design."""

    OFFLINE = 'OFFLINE'
    IDLE_DISABLED = 'IDLE_DISABLED'
    ARMING = 'ARMING'
    WAIT_TEACH = 'WAIT_TEACH'
    ALIGNING = 'ALIGNING'
    ACTIVE = 'ACTIVE'
    WAIT_TEACH_RELEASE = 'WAIT_TEACH_RELEASE'
    STOPPING = 'STOPPING'
    FAULT_LATCHED = 'FAULT_LATCHED'


STATE_LABELS = {
    TeleopState.OFFLINE: '反馈窗口不完整或身份未确认',
    TeleopState.IDLE_DISABLED: '整侧已失能，等待启动请求',
    TeleopState.ARMING: '正在整侧使能并读回验证',
    TeleopState.WAIT_TEACH: '等待操作员用实体按钮让主臂进入示教并确认',
    TeleopState.ALIGNING: '从臂正在受限对齐主臂',
    TeleopState.ACTIVE: '实时摇操中',
    TeleopState.WAIT_TEACH_RELEASE: '已停目标流，等待操作员确认实体示教已退出',
    TeleopState.STOPPING: '正在整侧失能并读回验证',
    TeleopState.FAULT_LATCHED: '故障已锁存，本进程不再自动进入摇操',
}


@dataclass
class TeleopConfig:
    """One side's teleop tunables, thresholds and gates."""

    side: str = 'left'
    port: str = 'can_left'
    send: bool = False
    align_only: bool = False
    duration_s: Optional[float] = None

    command_rate_hz: float = DEFAULT_COMMAND_RATE_HZ
    speed_percent: int = DEFAULT_SPEED_PERCENT
    stale_s: float = DEFAULT_STALE_S
    max_skew_s: float = DEFAULT_MAX_SKEW_S
    preflight_seconds: float = DEFAULT_PREFLIGHT_SECONDS
    arm_verify_s: float = DEFAULT_ARM_VERIFY_S
    disable_verify_s: float = DEFAULT_DISABLE_VERIFY_S
    disable_retries: int = DEFAULT_DISABLE_RETRIES
    start_timeout_s: float = DEFAULT_START_TIMEOUT_S
    teach_timeout_s: float = DEFAULT_TEACH_TIMEOUT_S
    teach_release_timeout_s: float = DEFAULT_TEACH_RELEASE_TIMEOUT_S

    align_rate_hz: float = DEFAULT_ALIGN_RATE_HZ
    align_speed_percent: int = DEFAULT_ALIGN_SPEED_PERCENT
    max_delta_deg: float = DEFAULT_MAX_DELTA_DEG
    max_goal_clamp_deg: float = DEFAULT_MAX_GOAL_CLAMP_DEG
    min_align_s: float = DEFAULT_MIN_ALIGN_S
    max_peak_deg_s: float = DEFAULT_MAX_PEAK_DEG_S
    settle_s: float = DEFAULT_SETTLE_S
    goal_tolerance_deg: float = DEFAULT_GOAL_TOLERANCE_DEG
    master_drift_deg: float = DEFAULT_MASTER_DRIFT_DEG
    align_follow_error_deg: float = DEFAULT_ALIGN_FOLLOW_ERROR_DEG
    tracking_error_deg: float = DEFAULT_TRACKING_ERROR_DEG
    tracking_error_grace_s: float = DEFAULT_TRACKING_ERROR_GRACE_S

    deadband_deg: float = DEFAULT_DEADBAND_DEG
    deadband_speed: float = DEFAULT_DEADBAND_SPEED_DEG_S
    filter_name: str = DEFAULT_FILTER
    filter_tau: float = DEFAULT_FILTER_TAU_S
    alpha_beta_alpha: float = DEFAULT_ALPHA_BETA_ALPHA
    alpha_beta_beta: float = DEFAULT_ALPHA_BETA_BETA
    alpha_beta_max_dt: float = DEFAULT_ALPHA_BETA_MAX_DT_S
    one_euro_min_cutoff: float = DEFAULT_ONE_EURO_MIN_CUTOFF_HZ
    one_euro_beta: float = DEFAULT_ONE_EURO_BETA
    one_euro_d_cutoff: float = DEFAULT_ONE_EURO_D_CUTOFF_HZ
    smooth_bandwidth: float = DEFAULT_SMOOTH_BANDWIDTH_RAD_S
    smooth_max_velocity: float = DEFAULT_SMOOTH_MAX_VELOCITY_DEG_S
    smooth_max_acceleration: float = DEFAULT_SMOOTH_MAX_ACCELERATION_DEG_S2
    smooth_max_jerk: float = DEFAULT_SMOOTH_MAX_JERK_DEG_S3
    max_step_deg: float = DEFAULT_MAX_STEP_DEG

    gripper: bool = True
    gripper_scale: float = DEFAULT_GRIPPER_SCALE
    gripper_deadband_m: float = DEFAULT_GRIPPER_DEADBAND_M
    gripper_effort_nm: float = DEFAULT_GRIPPER_EFFORT_NM

    error_policy: str = DEFAULT_ERROR_POLICY
    max_error_rate_hz: float = DEFAULT_MAX_ERROR_RATE_HZ
    fault_fast_stop: bool = DEFAULT_FAULT_FAST_STOP


def _validate_config(config: TeleopConfig) -> None:
    """Reject invalid control parameters before opening or enabling a bus."""
    for name, value in vars(config).items():
        if isinstance(value, (int, float)) and not math.isfinite(value):
            raise ValueError(f'{name} 必须为有限数值')
    positive = (
        'command_rate_hz', 'align_rate_hz', 'stale_s', 'max_skew_s',
        'preflight_seconds', 'arm_verify_s', 'disable_verify_s',
        'start_timeout_s', 'teach_timeout_s', 'teach_release_timeout_s',
        'min_align_s', 'max_peak_deg_s', 'max_delta_deg',
        'goal_tolerance_deg', 'master_drift_deg', 'align_follow_error_deg',
        'tracking_error_deg', 'max_error_rate_hz',
    )
    for name in positive:
        if getattr(config, name) <= 0:
            raise ValueError(f'{name} 必须大于 0')
    for name in ('settle_s', 'tracking_error_grace_s', 'max_goal_clamp_deg',
                 'disable_retries', 'max_step_deg', 'deadband_deg',
                 'deadband_speed', 'gripper_deadband_m'):
        if getattr(config, name) < 0:
            raise ValueError(f'{name} 不能为负数')
    for name in ('speed_percent', 'align_speed_percent'):
        if not 1 <= getattr(config, name) <= 100:
            raise ValueError(f'{name} 必须在 1～100 之间')
    if config.duration_s is not None and config.duration_s <= 0:
        raise ValueError('duration_s 必须大于 0')
    if config.error_policy not in ERROR_POLICIES:
        raise ValueError('未知的 CAN 错误策略')
    if config.filter_name not in FILTERS:
        raise ValueError('未知的滤波器')
    # Construct the processing chain now, so its own parameter checks also
    # run before arming instead of after completing an alignment.
    FollowChain(config, gripper_on=config.gripper)


def build_config(options) -> TeleopConfig:
    """Turn parsed CLI options into one side's configuration."""
    side = options.side
    return TeleopConfig(
        side=side,
        port=options.port or f'can_{side}',
        send=bool(options.send),
        align_only=options.align_only,
        duration_s=options.duration,
        command_rate_hz=options.rate,
        speed_percent=options.speed_percent,
        stale_s=options.stale_seconds,
        max_skew_s=options.max_skew_seconds,
        preflight_seconds=options.preflight_seconds,
        arm_verify_s=options.arm_verify_seconds,
        disable_verify_s=options.disable_verify_seconds,
        start_timeout_s=options.start_timeout,
        teach_timeout_s=options.teach_timeout,
        teach_release_timeout_s=options.teach_release_timeout,
        align_rate_hz=options.align_rate,
        align_speed_percent=options.align_speed_percent,
        max_delta_deg=options.max_delta_deg,
        max_goal_clamp_deg=options.max_goal_clamp_deg,
        min_align_s=options.min_align,
        max_peak_deg_s=options.max_peak_speed,
        settle_s=options.settle_seconds,
        goal_tolerance_deg=options.goal_tolerance_deg,
        master_drift_deg=options.master_drift_deg,
        align_follow_error_deg=options.max_follow_error_deg,
        tracking_error_deg=options.tracking_error_deg,
        tracking_error_grace_s=options.tracking_error_grace,
        deadband_deg=options.deadband_deg,
        deadband_speed=options.deadband_speed,
        filter_name=options.filter,
        filter_tau=options.filter_tau,
        alpha_beta_alpha=options.alpha_beta_alpha,
        alpha_beta_beta=options.alpha_beta_beta,
        alpha_beta_max_dt=options.alpha_beta_max_dt,
        one_euro_min_cutoff=options.one_euro_min_cutoff,
        one_euro_beta=options.one_euro_beta,
        one_euro_d_cutoff=options.one_euro_d_cutoff,
        smooth_bandwidth=options.smooth_bandwidth,
        smooth_max_velocity=options.smooth_max_velocity,
        smooth_max_acceleration=options.smooth_max_acceleration,
        smooth_max_jerk=options.smooth_max_jerk,
        max_step_deg=options.max_step_deg,
        gripper=not options.no_gripper,
        gripper_scale=options.gripper_scale,
        gripper_deadband_m=options.gripper_deadband,
        gripper_effort_nm=options.gripper_effort,
        error_policy=options.error_policy,
        max_error_rate_hz=options.max_error_rate_hz,
        fault_fast_stop=options.fault_fast_stop,
    )


class RateWindow:
    """Count events and report the rate over the most recent window."""

    def __init__(self, window_s: float = 1.0):
        self.window_s = float(window_s)
        self._times = []
        self.total = 0

    def add(self, now: Optional[float] = None) -> None:
        """Record one event."""
        moment = time.monotonic() if now is None else float(now)
        self._times.append(moment)
        self.total += 1
        limit = moment - self.window_s
        while self._times and self._times[0] < limit:
            self._times.pop(0)

    def rate(self, now: Optional[float] = None) -> float:
        """Return the event rate inside the window, in Hz."""
        moment = time.monotonic() if now is None else float(now)
        limit = moment - self.window_s
        while self._times and self._times[0] < limit:
            self._times.pop(0)
        return len(self._times) / self.window_s


def _make_estimator(config: TeleopConfig):
    """Build the follow-phase state estimator the operator asked for."""
    if config.filter_name == 'none':
        return None
    if config.filter_name == 'lowpass':
        return LowPassFilter(config.filter_tau)
    if config.filter_name == 'alpha-beta':
        return AlphaBetaFilter(config.alpha_beta_alpha, config.alpha_beta_beta,
                               config.alpha_beta_max_dt)
    return OneEuroFilter(config.one_euro_min_cutoff, config.one_euro_beta,
                         config.one_euro_d_cutoff)


def _filter_note(config: TeleopConfig) -> str:
    """Describe the follow chain in use, for the operator log."""
    if config.filter_name == 'none':
        note = '无状态估计'
    elif config.filter_name == 'lowpass':
        note = f'一阶低通 τ={config.filter_tau:g}s'
    elif config.filter_name == 'alpha-beta':
        note = (f'α-β 状态估计（α={config.alpha_beta_alpha:g}、'
                f'β={config.alpha_beta_beta:g}）')
    else:
        note = (f'One Euro 滤波（min_cutoff={config.one_euro_min_cutoff:g}Hz、'
                f'beta={config.one_euro_beta:g}）')
    if config.deadband_deg > 0.0:
        note += (f'，死区 {config.deadband_deg:g}°'
                 f'（速度门限 {config.deadband_speed:g}°/s）')
    if config.smooth_bandwidth > 0.0:
        note += (f'，五次插值平滑（带宽 {config.smooth_bandwidth:g}rad/s）')
    return note


class FollowChain:
    """Deadband, state estimate and quintic smoothing, plus the gripper."""

    def __init__(self, config: TeleopConfig, *, gripper_on: bool):
        self.config = config
        self.gate = DeadbandGate(config.deadband_deg, config.deadband_speed)
        self.estimator = _make_estimator(config)
        self.smoother = None
        if config.smooth_bandwidth > 0.0:
            self.smoother = MotionSmoother(
                config.smooth_bandwidth, config.smooth_max_velocity,
                config.smooth_max_acceleration, config.smooth_max_jerk)
        self.gripper_on = gripper_on
        self.gripper_gate = (DeadbandGate(config.gripper_deadband_m, 0.0)
                             if gripper_on else None)
        self.limit_hits: Dict[int, int] = {}
        self.clamp_hits: Dict[int, int] = {}
        self._previous: Optional[Dict[int, float]] = None
        self._gripper_target: Optional[float] = None

    def reset(self, command_deg, master_deg,
              command_gripper_m: Optional[float] = None,
              master_gripper_m: Optional[float] = None) -> None:
        """Seed the chain: commands follow the target, the gate the master."""
        seed = dict(command_deg)
        if self.gripper_on and command_gripper_m is not None:
            seed[7] = float(command_gripper_m)
            self._gripper_target = float(command_gripper_m)
        if self.estimator is not None:
            self.estimator.reset(seed)
        if self.smoother is not None:
            self.smoother.reset(seed)
        self.gate.reset(dict(master_deg))
        if self.gripper_gate is not None and master_gripper_m is not None:
            self.gripper_gate.reset(
                {7: scale_gripper(master_gripper_m,
                                  self.config.gripper_scale)})
        self._previous = dict(command_deg)

    def update(self, master_deg, master_gripper_m: Optional[float],
               dt: float) -> Tuple[Dict[int, float], Optional[float]]:
        """Turn one master reading into a follower command set."""
        bounded, _ = clamp_targets(dict(master_deg))
        if any(abs(master_deg[j] - bounded[j]) > self.config.max_goal_clamp_deg
               for j in range(1, JOINT_COUNT + 1)):
            raise SafetyError('主臂目标超限过多，拒绝跟随')
        sample = self.gate.update(master_deg, dt)
        if self.gripper_gate is not None and master_gripper_m is not None:
            # 夹爪并入同一条链：对齐 → 跟随的切换不会让它跳变，阈值换成米
            # （0.08m 的行程上套角度阈值等于永远不动）。
            sample = dict(sample)
            sample[7] = self.gripper_gate.update(
                {7: scale_gripper(master_gripper_m,
                                  self.config.gripper_scale)}, dt)[7]
        if self.estimator is not None:
            sample = self.estimator.update(sample, dt)
        if self.smoother is not None:
            velocities = (self.estimator.velocities()
                          if self.estimator is not None else {})
            sample = self.smoother.update(sample, velocities, dt)
        targets = {joint: sample[joint]
                   for joint in range(1, JOINT_COUNT + 1)}
        targets, clamped = clamp_targets(targets)
        for joint in clamped:
            self.clamp_hits[joint] = self.clamp_hits.get(joint, 0) + 1
        if self.config.max_step_deg > 0.0:
            targets, stepped = limit_step(targets, self._previous,
                                          self.config.max_step_deg)
            for joint in stepped:
                self.limit_hits[joint] = self.limit_hits.get(joint, 0) + 1
        self._previous = targets
        if self.gripper_gate is not None:
            self._gripper_target = clamp_gripper(sample[7])
        elif self.gripper_on and master_gripper_m is not None:
            self._gripper_target = clamp_gripper(
                scale_gripper(master_gripper_m, self.config.gripper_scale))
        return dict(targets), self._gripper_target


class OperatorLink:
    """The confirmation channel between a session and its operator."""

    def notify(self, message: str) -> None:
        """Tell the operator something happened."""
        print(message)

    def require_start(self, timeout_s: float) -> None:
        """Wait for a start request; raise SafetyError when refused."""
        raise NotImplementedError

    def wait_teach_engaged(self, timeout_s: float) -> bool:
        """Wait for the operator to confirm the master is in teach mode."""
        raise NotImplementedError

    def poll_stop(self) -> Optional[str]:
        """Return a stop reason, or None to keep running."""
        raise NotImplementedError

    def wait_teach_released(self, timeout_s: float) -> bool:
        """Wait until the operator confirms the teach button is released."""
        raise NotImplementedError

    def abort_waits(self) -> None:
        """
        Release any blocked wait so the session can stop promptly.

        The console operator needs nothing here (the operator sits at the
        prompt), but a service-driven operator must unblock when the process
        that drives it is going away.
        """
        return None


class ConsoleOperator(OperatorLink):
    """Drive the session from terminal prompts; EOF aborts."""

    def _ask(self, prompt: str) -> Optional[str]:
        try:
            return input(prompt)
        except EOFError:
            return None

    def require_start(self, timeout_s: float) -> None:
        answer = self._ask(
            f'按回车开始（{timeout_s:g}s 内有效）；'
            '输入 q 后回车退出：')
        if answer is None:
            raise SafetyError('没有可用的交互输入，拒绝进入摇操')
        if answer.strip().lower().startswith('q'):
            raise SafetyError('操作员取消了启动请求')

    def wait_teach_engaged(self, timeout_s: float) -> bool:
        answer = self._ask(
            '请用主臂实体按钮进入示教/重力补偿并托稳主臂，'
            f'然后按回车确认（{timeout_s:g}s 内）；输入 q 放弃：')
        if answer is None:
            raise SafetyError('没有可用的交互输入，拒绝继续')
        return not answer.strip().lower().startswith('q')

    def poll_stop(self) -> Optional[str]:
        """Return None: the console stops on Ctrl-C, handled by run()."""
        return None

    def wait_teach_released(self, timeout_s: float) -> bool:
        answer = self._ask(
            '请关闭主臂实体示教按钮并托稳主臂，然后按回车确认失能'
            f'（{timeout_s:g}s 内）；直接输入 q 表示暂不失能：')
        if answer is None:
            return False
        return not answer.strip().lower().startswith('q')


class GuardedBus:
    """A bus wrapper that refuses to transmit unless sending is allowed."""

    def __init__(self, bus, allow_send: bool):
        self._bus = bus
        self.allow_send = allow_send
        self.sent_ids: Dict[int, int] = {}

    def recv(self, timeout: Optional[float] = None):
        """Receive one frame, or None on timeout."""
        return self._bus.recv(timeout=timeout)

    def send(self, message) -> None:
        """Transmit one frame; forbidden unless ``allow_send``."""
        if not self.allow_send:
            raise SafetyError(
                f'干跑模式：拒绝发送 0x{message.arbitration_id:03X}')
        self._bus.send(message)
        self.sent_ids[message.arbitration_id] = (
            self.sent_ids.get(message.arbitration_id, 0) + 1)

    def shutdown(self) -> None:
        """Close the underlying socket."""
        self._bus.shutdown()


@dataclass
class SideStatus:
    """One session's externally visible state."""

    side: str
    state: TeleopState
    fault: Optional[str]
    command_hz: float
    master_hz: float
    follower_hz: float
    error_frames: int
    error_rate_hz: float
    tracking_error_deg: float
    master_gripper_m: Optional[float]
    follower_gripper_m: Optional[float]

    @property
    def label(self) -> str:
        """Return the Chinese description of the current state."""
        return STATE_LABELS.get(self.state, self.state.value)


def _enable_payload(enable: bool) -> bytes:
    """Encode the 0x471 broadcast: motor number 7 (all), flag, padding."""
    return bytes((JOINT_COUNT + 1, 0x02 if enable else 0x01)) + bytes(6)


class TeleopSession:
    """One side's session: explicit state machine and realtime loop."""

    def __init__(self, config: TeleopConfig, operator: OperatorLink, *,
                 log=print):
        _validate_config(config)
        self.config = config
        self.operator = operator
        self.log = log
        self.state = TeleopState.OFFLINE
        self.fault: Optional[str] = None
        self.report: Optional[PreflightReport] = None
        self.plan: Optional[AlignmentPlan] = None
        self.bus: Optional[GuardedBus] = None
        self._monitor: Optional[RuntimeMonitor] = None
        self._chain: Optional[FollowChain] = None
        self._gripper_on = False
        self._armed = False
        self._streamed = False
        self._stored_frames = 0
        self._last_targets: Optional[Dict[int, float]] = None
        self._last_gripper: Optional[float] = None
        self._tracking_error = 0.0
        self._tracking_since: Optional[float] = None
        self._command_rate = RateWindow()
        self._master_rate = RateWindow()
        self._follower_rate = RateWindow()
        self._last_report = 0.0
        self._skip_release_wait = False
        self._master_versions = (0, 0, 0)
        self.enter_active_hook = None

    # ---- 对外状态 -----------------------------------------------------

    def status(self) -> SideStatus:
        """Return the current status snapshot for publishing."""
        now = time.monotonic()
        master_gripper = follower_gripper = None
        if self._monitor is not None:
            follower_gripper, master_gripper = self._monitor.grippers(now)
        return SideStatus(
            side=self.config.side,
            state=self.state,
            fault=self.fault,
            command_hz=self._command_rate.rate(now),
            master_hz=self._master_rate.rate(now),
            follower_hz=self._follower_rate.rate(now),
            error_frames=(self._monitor.error_frames
                          if self._monitor is not None else 0),
            error_rate_hz=(self._monitor.error_rate_hz(now)
                           if self._monitor is not None else 0.0),
            tracking_error_deg=self._tracking_error,
            master_gripper_m=master_gripper,
            follower_gripper_m=follower_gripper,
        )

    # ---- 总线与监视 ---------------------------------------------------

    def _open_bus(self) -> None:
        raw = can.Bus(interface='socketcan', channel=self.config.port,
                      receive_own_messages=False,
                      can_filters=socket_filters())
        self.bus = GuardedBus(raw, self.config.send)
        self._monitor = RuntimeMonitor(
            stale_s=self.config.stale_s, max_skew_s=self.config.max_skew_s,
            error_policy=self.config.error_policy,
            max_error_rate_hz=self.config.max_error_rate_hz,
            gripper_stale_s=DEFAULT_GRIPPER_STALE_S,
        )

    def _close_bus(self) -> None:
        if self.bus is not None:
            try:
                self.bus.shutdown()
            except can.CanError:
                pass
            self.bus = None

    def _drain_until(self, deadline: float, expect_enabled: bool = True
                     ) -> None:
        """Process incoming frames until a deadline, keeping every gate."""
        while True:
            now = time.monotonic()
            if now >= deadline:
                return
            frame = self.bus.recv(timeout=min(0.005, deadline - now))
            if frame is None:
                continue
            self._observe(frame, expect_enabled=expect_enabled)

    def _observe(self, frame, *, expect_enabled: bool = True) -> None:
        """Run one frame through the gates and the rate bookkeeping."""
        now = time.monotonic()
        self._monitor.process(frame, now, expect_enabled=expect_enabled)
        if frame.is_error_frame or frame.is_extended_id:
            return
        if frame.arbitration_id == 0x2C7:
            self._master_rate.add(now)
        elif frame.arbitration_id == 0x2A7:
            self._follower_rate.add(now)

    def _verify_enable_window(self, seconds: float, expected: bool
                              ) -> Tuple[bool, str]:
        """
        Judge every enable sample in a window against the expected state.

        The window does not raise on a sample that disagrees: frames that were
        already in flight when the broadcast went out still carry the old
        state, so only the verdict over the whole window decides.
        """
        samples = defaultdict(list)
        # Ignore in-flight samples during the bounded actuator transition,
        # then demand a complete window with every sample agreeing.
        settled_at = time.monotonic() + 0.25
        deadline = settled_at + seconds
        while time.monotonic() < deadline:
            frame = self.bus.recv(timeout=0.05)
            if frame is None:
                continue
            self._observe(frame, expect_enabled=False)
            if time.monotonic() < settled_at:
                continue
            if frame.is_error_frame or frame.is_extended_id:
                continue
            feedback = decode(frame.arbitration_id, frame.data)
            if feedback is not None:
                samples[feedback.joint].append(feedback.enabled)
        missing = [joint for joint in range(1, JOINT_COUNT + 1)
                   if not samples.get(joint)]
        if missing:
            joints = '、'.join(f'j{joint}' for joint in missing)
            return False, f'观察窗口内没有 {joints} 的低速反馈样本'
        mixed = [joint for joint, values in samples.items()
                 if any(value is not expected for value in values)]
        if mixed:
            joints = '、'.join(f'j{joint}' for joint in sorted(mixed))
            return False, f'观察窗口内 {joints} 的使能位不是一致的 {expected}'
        return True, '全部样本一致'

    # ---- 各阶段 -------------------------------------------------------

    def _run_preflight(self) -> None:
        """Read-only identity, enable and pose gate; never transmits."""
        self.state = TeleopState.OFFLINE
        self.log(f'监听 {self.config.side} / {self.config.port}，执行 '
                 f'{self.config.preflight_seconds:.1f}s 只读门禁'
                 f'（错误帧策略 {self.config.error_policy}）。')
        report = capture_preflight(
            self.config.port, self.config.preflight_seconds,
            side=self.config.side, stale_s=self.config.stale_s,
            max_skew_s=self.config.max_skew_s,
            error_policy=self.config.error_policy,
            max_error_rate_hz=self.config.max_error_rate_hz,
        )
        self.report = report
        print_preflight(report)
        if report.error_frames and report.identity.confirmed:
            self.log(f'  注意：本窗口容忍了 {report.error_frames} 个孤立 CAN '
                     f'错误帧（平均 {report.max_error_rate_hz:.2f} Hz、单秒'
                     f'峰值 {report.peak_error_rate_hz:.1f} Hz）；'
                     '状态退化或持续速率超限仍会中止，物理层排查见 '
                     'docs/CAN_BUS_INTEGRITY.md。')
        if not report.identity.confirmed:
            reasons = '；'.join(report.identity.reasons) or '未给出原因'
            raise SafetyError(
                f'身份门禁未通过：{report.identity.status.value}（{reasons}）')
        if not report.all_samples_disabled:
            raise SafetyError('初始状态不是“整侧全部失能”，禁止继续使能')
        self.plan = plan_alignment(
            report.follower_deg, report.master_deg,
            max_delta_deg=self.config.max_delta_deg,
            max_goal_clamp_deg=self.config.max_goal_clamp_deg,
            min_duration_s=self.config.min_align_s,
            max_peak_deg_s=self.config.max_peak_deg_s,
        )
        print_plan(self.plan)
        self.log('  （以上为当前门禁下的对齐计划；进入对齐前会按最新姿态重新规划。）')
        self.state = TeleopState.IDLE_DISABLED

    def _run_arm(self) -> None:
        """Broadcast the side-wide enable and verify the read-back."""
        self.state = TeleopState.ARMING
        self.log('发送整侧使能 0x471 07 02（主从臂一起使能）…')
        self.bus.send(can.Message(
            arbitration_id=ENABLE_CAN_ID, data=_enable_payload(True),
            is_extended_id=False))
        self._armed = True
        try:
            ok, note = self._verify_enable_window(
                self.config.arm_verify_s, expected=True)
        except SafetyError as exc:
            raise SafetyError(f'整侧使能读回失败：{exc}') from exc
        if not ok:
            raise SafetyError(f'整侧使能读回失败：{note}')
        self.log('整侧使能读回通过。')
        self.state = TeleopState.WAIT_TEACH

    def _run_wait_teach(self) -> None:
        """Wait for the operator to confirm the master is teaching."""
        if not self.operator.wait_teach_engaged(self.config.teach_timeout_s):
            raise SafetyError('操作员未确认主臂进入实体示教，放弃启动')
        # Discard the socket backlog accumulated while the operator held the
        # prompt. Old frames must never become fresh just because we read them.
        self._close_bus()
        self._open_bus()
        ok, note = self._verify_enable_window(0.2, expected=True)
        if not ok:
            raise SafetyError(f'实体示教确认后的使能读回失败：{note}')
        self._monitor.snapshots()
        self.log('主臂反馈正常，开始自动对齐。')

    def _gripper_ramp(self) -> Optional[GripperRamp]:
        """Build the gripper ramp for the alignment move, if both are known."""
        if not self.config.gripper:
            return None
        follower_m, master_m = self._monitor.grippers()
        if follower_m is None or master_m is None:
            self.log('  夹爪反馈不完整：本次不对夹爪下发目标（只是不对夹爪'
                     '发帧，关节照常）。')
            return None
        ramp = GripperRamp(
            start_m=clamp_gripper(follower_m),
            goal_m=scale_gripper(master_m, self.config.gripper_scale),
            effort_nm=self.config.gripper_effort_nm,
        )
        self.log(f'  夹爪随对齐一起移动：{ramp.start_m * 1000:.2f}mm → '
                 f'{ramp.goal_m * 1000:.2f}mm（主臂 {master_m * 1000:.2f}mm '
                 f'× {self.config.gripper_scale:g}）')
        return ramp

    def _run_align(self) -> None:
        """Plan and run the bounded alignment, then seed the follow chain."""
        self.state = TeleopState.ALIGNING
        follower, master = self._monitor.snapshots()
        self.plan = plan_alignment(
            follower, master,
            max_delta_deg=self.config.max_delta_deg,
            max_goal_clamp_deg=self.config.max_goal_clamp_deg,
            min_duration_s=self.config.min_align_s,
            max_peak_deg_s=self.config.max_peak_deg_s,
        )
        print_plan(self.plan)
        ramp = self._gripper_ramp()
        self._streamed = True
        plan = run_alignment(
            self.bus, self._monitor, self.plan,
            rate_hz=self.config.align_rate_hz,
            speed_percent=self.config.align_speed_percent,
            master_drift_deg=self.config.master_drift_deg,
            max_follow_error_deg=self.config.align_follow_error_deg,
            settle_s=self.config.settle_s,
            goal_tolerance_deg=self.config.goal_tolerance_deg,
            gripper=ramp,
            standby_on_finish=False,
            check_stop=self._check_stop,
        )
        self._streamed = True
        self.plan = plan
        self._chain = FollowChain(self.config, gripper_on=ramp is not None)
        master_gripper = self._monitor.grippers()[1]
        self._chain.reset(
            plan.goal_deg, master,
            command_gripper_m=ramp.goal_m if ramp is not None else None,
            master_gripper_m=master_gripper,
        )
        self._gripper_on = ramp is not None
        self._last_targets = dict(plan.goal_deg)
        self._last_gripper = ramp.goal_m if ramp is not None else None
        self.log(f'对齐完成：最大位移 {plan.max_delta_deg:.3f}°，'
                 f'时长 {plan.duration_s:.3f}s。')

    def _check_stop(self) -> None:
        """Honor stop requests during alignment as well as following."""
        reason = self.operator.poll_stop()
        if reason:
            self.log(f'停止请求：{reason}')
            raise KeyboardInterrupt

    def wait_for_peer(self) -> None:
        """Keep receiving and checking stop while the other side aligns."""
        self._check_stop()
        self._drain_until(time.monotonic() + 0.005)
        follower, master = self._monitor.snapshots()
        drift = max(abs(master[j] - self.plan.requested_goal_deg[j])
                    for j in range(1, JOINT_COUNT + 1))
        error = max(abs(follower[j] - self.plan.goal_deg[j])
                    for j in range(1, JOINT_COUNT + 1))
        if (drift > self.config.master_drift_deg
                or error > self.config.align_follow_error_deg):
            raise SafetyError('等待另一侧对齐时姿态已偏离，需重新启动对齐')

    def _run_follow(self) -> None:
        """Stream follower targets from the live master reading."""
        self.state = TeleopState.ACTIVE
        self.log(f'进入实时跟随：命令频率 {self.config.command_rate_hz:g}Hz，'
                 f'速度百分比 {self.config.speed_percent}，'
                 + _filter_note(self.config))
        period = 1.0 / self.config.command_rate_hz
        started = time.monotonic()
        next_command = started
        last_cycle = None
        self._last_report = started
        while True:
            now = time.monotonic()
            if now >= next_command:
                if self._emit_target(now, last_cycle):
                    last_cycle = now
                next_command += period
                if next_command < now:
                    next_command = now + period
            else:
                frame = self.bus.recv(timeout=min(0.002, next_command - now))
                if frame is not None:
                    self._observe(frame)
            if now - self._last_report >= STATUS_PERIOD_S:
                self._report(now)
                self._last_report = now
            reason = self.operator.poll_stop()
            if reason:
                self.log(f'停止请求：{reason}')
                return
            if (self.config.duration_s is not None
                    and now - started >= self.config.duration_s):
                self.log(f'运行到 --duration {self.config.duration_s:g}s')
                return

    def _emit_target(self, now: float, last_cycle: Optional[float]) -> bool:
        """Build and send one follower command set from the freshest pose."""
        follower, master = self._monitor.snapshots(now)
        versions = self._monitor.master.versions()
        if not all(new > old for new, old in
                   zip(versions, self._master_versions)):
            return False
        dt = 0.0 if last_cycle is None else max(now - last_cycle, 1e-4)
        master_gripper = self._monitor.grippers(now)[1]
        if self._gripper_on and master_gripper is None:
            raise SafetyError('主臂夹爪反馈过期，停止跟随')
        targets, gripper_target = self._chain.update(
            master, master_gripper if self._gripper_on else None, dt)
        error = max(abs(follower[joint] - targets[joint])
                    for joint in range(1, JOINT_COUNT + 1))
        self._tracking_error = error
        limit = self.config.tracking_error_deg
        if error > limit:
            if self._tracking_since is None:
                self._tracking_since = now
            elif (now - self._tracking_since
                  > self.config.tracking_error_grace_s):
                raise SafetyError(
                    f'从臂跟踪误差 {error:.3f}° 持续超过 {limit:.3f}°'
                    f'（{self.config.tracking_error_grace_s:g}s）')
        else:
            self._tracking_since = None
        send_target(self.bus, targets, self.config.speed_percent,
                    gripper_target if self._gripper_on else None,
                    self.config.gripper_effort_nm)
        self._command_rate.add(now)
        self._last_targets = dict(targets)
        self._last_gripper = gripper_target
        self._master_versions = versions
        if self._chain.clamp_hits or self._chain.limit_hits:
            self._report_limit_hits()
        return True

    def _report_limit_hits(self) -> None:
        """Print the joints that hit their command limits, once per set."""
        hits = dict(self._chain.clamp_hits)
        if not hits:
            return
        if getattr(self, '_reported_hits', None) == set(hits):
            return
        self._reported_hits = set(hits)
        self.log('    !! 已触限位的关节：' + '、'.join(
            f'j{joint}({count})' for joint, count in sorted(hits.items())))

    def _report(self, now: float) -> None:
        """Print one periodic status line for the operator."""
        targets = self._last_targets or {}
        pose = ' '.join(f'j{joint}={targets[joint]:+8.3f}'
                        for joint in range(1, JOINT_COUNT + 1))
        gripper = (f'  夹爪={self._last_gripper * 1000:+8.2f}mm'
                   if self._last_gripper is not None else '')
        status = self.status()
        self.log(f'  [跟随] 命令 {status.command_hz:5.1f}Hz  '
                 f'主臂 {status.master_hz:5.1f}Hz  '
                 f'从臂 {status.follower_hz:5.1f}Hz  '
                 f'跟踪误差 {status.tracking_error_deg:5.2f}°  '
                 f'错误帧 {status.error_frames}'
                 f'（{status.error_rate_hz:.1f}Hz）')
        self.log('  从臂目标：' + pose + gripper)

    def _run_standby(self) -> None:
        """Stop the target stream without touching the master's addresses."""
        if self._streamed:
            send_standby(self.bus, self.config.speed_percent)
            self.log('已停止从臂目标流并切回待机（只发从臂标准地址）。')
        self.state = TeleopState.WAIT_TEACH_RELEASE

    def _disable_verified(self) -> Tuple[bool, str]:
        """Broadcast the side-wide disable and verify the read-back."""
        self.state = TeleopState.STOPPING
        note = '没有尝试发送失能帧'
        for attempt in range(1, self.config.disable_retries + 2):
            self.log('发送整侧失能 0x471 07 01'
                     f'（主从臂一起失能，第 {attempt} 次）…')
            self.bus.send(can.Message(
                arbitration_id=ENABLE_CAN_ID, data=_enable_payload(False),
                is_extended_id=False))
            ok, note = self._verify_enable_window(
                self.config.disable_verify_s, expected=False)
            if ok:
                self.log('整侧失能读回通过：窗口内所有低速样本均为 disabled。')
                return True, note
            self.log(f'  失能读回未通过：{note}')
        return False, note

    def _run_disable(self) -> None:
        """Disable the side and move to IDLE_DISABLED, or raise."""
        ok, note = self._disable_verified()
        if not ok:
            raise SafetyError(f'整侧失能读回未通过（{note}），请人工失能或断电')
        self.state = TeleopState.IDLE_DISABLED

    def _fault_stop(self, reason: str) -> None:
        """Best-effort stop after a latched fault; never resumes following."""
        self.state = TeleopState.FAULT_LATCHED
        self.fault = reason
        self.log('=' * 72)
        self.log(f'故障锁存：{reason}')
        self.log('=' * 72)
        if not self._streamed and not self._armed:
            self.log('尚未使能也未下发过目标，不发送任何停止帧。')
            self._close_bus()
            return
        try:
            if self._streamed:
                send_standby(self.bus, self.config.speed_percent)
                self.log('已停止从臂目标流并切回待机。')
            if self.config.fault_fast_stop:
                self.bus.send(can.Message(
                    arbitration_id=MOTION_CTRL_1_ID,
                    data=encode_motion_ctrl_1(EMERGENCY_STOP),
                    is_extended_id=False))
                self.log('已向从臂发送 0x150 急停（未现场验收的载荷）。')
            if self._armed:
                self.state = TeleopState.WAIT_TEACH_RELEASE
                self.log('运动流已停止；关闭实体示教并确认后才整侧失能。')
                if (not self._skip_release_wait
                        and self.operator.wait_teach_released(
                            self.config.teach_release_timeout_s)):
                    # Fresh receive queue for the disable transition.
                    self._close_bus()
                    self._open_bus()
                    ok, note = self._disable_verified()
                    if not ok:
                        self.log(f'故障流程的失能读回未通过：{note}')
                else:
                    self.log('未确认实体示教退出，保留当前使能状态。')
        except (SafetyError, can.CanError, OSError, KeyboardInterrupt) as exc:
            self.log(f'故障停止没有完成：{exc}')
            self.log('请人工托住主臂并断电或使用 piper_arm_enable --disable '
                     '--send 失能。')
        self.state = TeleopState.FAULT_LATCHED
        self._close_bus()

    def skip_release_wait(self, reason: str) -> None:
        """
        Stop without waiting for the release confirmation.

        Used when the process is going away: the follower must stop taking
        targets at once, but the disable still waits for a human because the
        master may still be in physical teach mode.
        """
        self._skip_release_wait = True
        self.log(f'不再等待实体示教退出确认：{reason}')

    # ---- 主流程 -------------------------------------------------------

    def run(self) -> int:
        """Run one side's session and return a process exit code."""
        try:
            self._run_preflight()
            if not self.config.send:
                self.log('干跑完成：没有发送任何 CAN 帧。'
                         '确认无误后加 --send --workspace-clear 执行。')
                return EXIT_OK
            self.operator.require_start(self.config.start_timeout_s)
            self._run_preflight()
            self._open_bus()
            self._run_arm()
            self._run_wait_teach()
            self._run_align()
            if self.config.align_only:
                return self._shutdown_after_stop()
            if self.enter_active_hook is not None:
                self.enter_active_hook()
            self._run_follow()
        except KeyboardInterrupt:
            self.log('收到 Ctrl-C。')
            self.fault = None
            return self._shutdown_after_stop()
        # Unexpected failures after motion must also stop the target stream.
        except Exception as exc:
            if self.bus is None:
                self.log(f'拒绝：{exc}')
                return EXIT_REFUSED
            self._fault_stop(str(exc))
            return EXIT_FAILED
        return self._shutdown_after_stop()

    def _shutdown_after_stop(self) -> int:
        """Stop the stream and run the operator-confirmed disable flow."""
        try:
            if self.bus is None:
                return EXIT_REFUSED if not self._armed else EXIT_FAILED
            self._run_standby()
            self.log('已停止从臂目标流。请确认主臂实体示教按钮已关闭，'
                     '然后执行整侧失能（0x471 07 01）。')
            released = False
            if self._skip_release_wait:
                self.operator.abort_waits()
            else:
                released = self.operator.wait_teach_released(
                    self.config.teach_release_timeout_s)
            if not released:
                self.log('没有得到“实体示教已退出”的确认：**不发送失能帧**。')
                self.log('主从臂目前仍然使能并保持姿态。请关闭主臂实体示教后'
                         '运行 ros2 run piper piper_arm_enable --disable --send，'
                         '或重新运行本工具完成退出流程。')
                return EXIT_PENDING_RELEASE
            self._close_bus()
            self._open_bus()
            self._run_disable()
            self.log('退出完成：整侧已失能，主从臂都保持失能状态。')
            return EXIT_OK
        except (SafetyError, can.CanError, OSError) as exc:
            self._fault_stop(str(exc))
            return EXIT_FAILED
        except KeyboardInterrupt:
            self.log('再次 Ctrl-C：停止退出流程。机械臂停在当前状态，'
                     '请人工确认使能状态后再离开。')
            return EXIT_PENDING_RELEASE
        finally:
            self._close_bus()


def argument_parser(add_help: bool = True,
                    description: Optional[str] = None) -> ArgumentParser:
    """Build the shared option set used by the CLI and the manager node."""
    parser = ArgumentParser(description=description or __doc__,
                            add_help=add_help)
    parser.add_argument('--side', choices=('left', 'right'), default='left',
                        help='本进程负责的一侧（默认 %(default)s）')
    parser.add_argument('--port', help='SocketCAN 接口（默认 can_<side>）')
    parser.add_argument('--rate', type=float, default=DEFAULT_COMMAND_RATE_HZ,
                        help='跟随阶段的命令频率 Hz（默认 %(default)s）')
    parser.add_argument('--speed-percent', type=int,
                        default=DEFAULT_SPEED_PERCENT,
                        help='从臂速度百分比 1-100，100 表示不额外限速'
                             '（默认 %(default)s）')
    parser.add_argument('--duration', type=float, default=None,
                        help='运行指定秒数后自动停止（默认不自动停止）')
    parser.add_argument('--send', action='store_true',
                        help='真正发送使能、对齐与跟随帧；默认只干跑')
    parser.add_argument('--align-only', action='store_true',
                        help='只验收姿态对齐，完成后停流并等待实体示教退出确认')
    parser.add_argument('--workspace-clear', action='store_true',
                        help='确认人员和线缆已退出运动空间；--send 时必需')

    group = parser.add_argument_group('门禁与超时')
    group.add_argument('--preflight-seconds', type=float,
                       default=DEFAULT_PREFLIGHT_SECONDS,
                       help='只读门禁时长，秒（默认 %(default)s）')
    group.add_argument('--arm-verify-seconds', type=float,
                       default=DEFAULT_ARM_VERIFY_S,
                       help='整侧使能的读回窗口，秒（默认 %(default)s）')
    group.add_argument('--disable-verify-seconds', type=float,
                       default=DEFAULT_DISABLE_VERIFY_S,
                       help='整侧失能的读回窗口，秒（默认 %(default)s）')
    group.add_argument('--start-timeout', type=float,
                       default=DEFAULT_START_TIMEOUT_S,
                       help='等待启动请求的秒数（默认 %(default)s）')
    group.add_argument('--teach-timeout', type=float,
                       default=DEFAULT_TEACH_TIMEOUT_S,
                       help='等待“主臂已进入示教”确认的秒数（默认 %(default)s）')
    group.add_argument('--teach-release-timeout', type=float,
                       default=DEFAULT_TEACH_RELEASE_TIMEOUT_S,
                       help='等待“实体示教已退出”确认的秒数（默认 %(default)s）')
    group.add_argument('--stale-seconds', type=float, default=DEFAULT_STALE_S,
                       help='六关节反馈的新鲜度阈值，秒（默认 %(default)s）')
    group.add_argument('--max-skew-seconds', type=float,
                       default=DEFAULT_MAX_SKEW_S,
                       help='六关节快照允许的最大时间跨度，秒（默认 %(default)s）')
    group.add_argument('--error-policy', choices=ERROR_POLICIES,
                       default=DEFAULT_ERROR_POLICY,
                       help='recoverable：容忍孤立错误帧，只在状态退化或持续'
                            '速率超限时中止（默认）；strict：任何错误帧都中止')
    group.add_argument('--max-error-rate-hz', type=float,
                       default=DEFAULT_MAX_ERROR_RATE_HZ,
                       help='recoverable 策略下的错误帧速率上限（默认 %(default)s）')

    group = parser.add_argument_group('自动对齐')
    group.add_argument('--align-rate', type=float,
                       default=DEFAULT_ALIGN_RATE_HZ,
                       help='对齐阶段的命令频率 Hz（默认 %(default)s）')
    group.add_argument('--align-speed-percent', type=int,
                       default=DEFAULT_ALIGN_SPEED_PERCENT,
                       help='对齐阶段的速度百分比（默认 %(default)s）')
    group.add_argument('--max-delta-deg', type=float,
                       default=DEFAULT_MAX_DELTA_DEG,
                       help='允许自动对齐的最大姿态差，度（默认 %(default)s）')
    group.add_argument('--max-goal-clamp-deg', type=float,
                       default=DEFAULT_MAX_GOAL_CLAMP_DEG,
                       help='主臂目标允许的限位钳制量，度（默认 %(default)s）')
    group.add_argument('--min-align', type=float, default=DEFAULT_MIN_ALIGN_S,
                       help='对齐最短时长，秒（默认 %(default)s）')
    group.add_argument('--max-peak-speed', type=float,
                       default=DEFAULT_MAX_PEAK_DEG_S,
                       help='对齐允许的 minimum-jerk 峰值速度，度/秒'
                            '（默认 %(default)s）')
    group.add_argument('--settle-seconds', type=float,
                       default=DEFAULT_SETTLE_S,
                       help='对齐结束后的稳定观察时间，秒（默认 %(default)s）')
    group.add_argument('--goal-tolerance-deg', type=float,
                       default=DEFAULT_GOAL_TOLERANCE_DEG,
                       help='对齐末端的姿态容差，度（默认 %(default)s）')
    group.add_argument('--master-drift-deg', type=float,
                       default=DEFAULT_MASTER_DRIFT_DEG,
                       help='对齐期间允许的主臂漂移，度（默认 %(default)s）')
    group.add_argument('--max-follow-error-deg', type=float,
                       default=DEFAULT_ALIGN_FOLLOW_ERROR_DEG,
                       help='对齐期间的跟踪误差上限，度（默认 %(default)s）')

    group = parser.add_argument_group('跟随处理')
    group.add_argument('--tracking-error-deg', type=float,
                       default=DEFAULT_TRACKING_ERROR_DEG,
                       help='跟随阶段的跟踪误差门禁，度（默认 %(default)s）')
    group.add_argument('--tracking-error-grace', type=float,
                       default=DEFAULT_TRACKING_ERROR_GRACE_S,
                       help='跟踪误差持续超限多久后判为故障，秒（默认 %(default)s）')
    group.add_argument('--deadband-deg', type=float,
                       default=DEFAULT_DEADBAND_DEG,
                       help='幅度死区，度；0 表示不设死区（默认 %(default)s）')
    group.add_argument('--deadband-speed', type=float,
                       default=DEFAULT_DEADBAND_SPEED_DEG_S,
                       help='死区的速度门限，度/秒（默认 %(default)s）')
    group.add_argument('--filter', choices=FILTERS, default=DEFAULT_FILTER,
                       help='状态估计器（默认 %(default)s）')
    group.add_argument('--filter-tau', type=float,
                       default=DEFAULT_FILTER_TAU_S,
                       help='低通时间常数，秒（默认 %(default)s）')
    group.add_argument('--alpha-beta-alpha', type=float,
                       default=DEFAULT_ALPHA_BETA_ALPHA,
                       help='α-β 的 α（默认 %(default)s）')
    group.add_argument('--alpha-beta-beta', type=float,
                       default=DEFAULT_ALPHA_BETA_BETA,
                       help='α-β 的 β（默认 %(default)s）')
    group.add_argument('--alpha-beta-max-dt', type=float,
                       default=DEFAULT_ALPHA_BETA_MAX_DT_S,
                       help='α-β 单次更新的最大间隔，秒（默认 %(default)s）')
    group.add_argument('--one-euro-min-cutoff', type=float,
                       default=DEFAULT_ONE_EURO_MIN_CUTOFF_HZ,
                       help='One Euro 的 min_cutoff，Hz（默认 %(default)s）')
    group.add_argument('--one-euro-beta', type=float,
                       default=DEFAULT_ONE_EURO_BETA,
                       help='One Euro 的 beta（默认 %(default)s）')
    group.add_argument('--one-euro-d-cutoff', type=float,
                       default=DEFAULT_ONE_EURO_D_CUTOFF_HZ,
                       help='One Euro 的 d_cutoff，Hz（默认 %(default)s）')
    group.add_argument('--smooth-bandwidth', type=float,
                       default=DEFAULT_SMOOTH_BANDWIDTH_RAD_S,
                       help='五次插值平滑的带宽 rad/s；0 表示关掉这一级'
                            '（默认 %(default)s）')
    group.add_argument('--smooth-max-velocity', type=float,
                       default=DEFAULT_SMOOTH_MAX_VELOCITY_DEG_S,
                       help='平滑级的速度上限，度/秒（默认 %(default)s）')
    group.add_argument('--smooth-max-acceleration', type=float,
                       default=DEFAULT_SMOOTH_MAX_ACCELERATION_DEG_S2,
                       help='平滑级的加速度上限，度/秒²（默认 %(default)s）')
    group.add_argument('--smooth-max-jerk', type=float,
                       default=DEFAULT_SMOOTH_MAX_JERK_DEG_S3,
                       help='平滑级的 jerk 上限，度/秒³（默认 %(default)s）')
    group.add_argument('--max-step-deg', type=float,
                       default=DEFAULT_MAX_STEP_DEG,
                       help='每周期目标最大变化，度；0 表示不限制'
                            '（默认 %(default)s）')

    group = parser.add_argument_group('夹爪')
    group.add_argument('--no-gripper', action='store_true',
                       help='不镜像夹爪（默认镜像）')
    group.add_argument('--gripper-scale', type=float,
                       default=DEFAULT_GRIPPER_SCALE,
                       help='主臂开口乘这个比例才是从臂目标（默认 %(default)s）')
    group.add_argument('--gripper-deadband', type=float,
                       default=DEFAULT_GRIPPER_DEADBAND_M,
                       help='夹爪幅度死区，米（默认 %(default)s）')
    group.add_argument('--gripper-effort', type=float,
                       default=DEFAULT_GRIPPER_EFFORT_NM,
                       help='夹持力矩，N·m（默认 %(default)s）')

    group = parser.add_argument_group('故障路径')
    group.add_argument('--fault-fast-stop', action='store_true',
                       default=DEFAULT_FAULT_FAST_STOP,
                       help='故障时额外发送 0x150 急停帧（现场未验收，默认不发送）')
    return parser


def install_sigterm_handler() -> None:
    """
    Turn SIGTERM into KeyboardInterrupt so the stop path still runs.

    The design requires the process to try a fault stop when it is signalled;
    Python's default SIGTERM handling would kill it without stopping the
    follower target stream first.  The manager installs the same handler.
    """
    def handler(_signum, _frame):
        raise KeyboardInterrupt

    try:
        signal.signal(signal.SIGTERM, handler)
    except ValueError:
        # 不在主线程时无法安装信号处理器；此时由主线程负责退出流程。
        pass


def main(args=None) -> int:
    """Run one side's teleop session from the command line."""
    install_sigterm_handler()
    options = argument_parser().parse_args(args)
    if options.send and not options.workspace_clear:
        print('错误：--send 必须同时给出 --workspace-clear')
        return EXIT_REFUSED
    config = build_config(options)
    try:
        session = TeleopSession(config, ConsoleOperator())
    except (ValueError, SafetyError) as exc:
        print(f'错误：{exc}')
        return EXIT_REFUSED
    code = session.run()
    if code == EXIT_PENDING_RELEASE:
        print('结果：目标流已停止，但没有确认实体示教已退出，'
              '因此没有失能。请人工完成退出。')
    return code


if __name__ == '__main__':
    sys.exit(main())
