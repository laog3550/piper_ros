"""Tests for the teleop control flow: filtering and the return-home phase."""

# These tests drive the real functions from piper_teleop with a fake clock, a
# fake rclpy and a recording publisher, so the timing behaviour -- how many
# commands a return move sends, when it stops -- is checked without a robot,
# a CAN bus or a ROS graph.

import math
import types

import pytest

from piper import piper_teleop, piper_teleop_fast
from piper.piper_feedback import JOINT_COUNT


def _pose(value):
    """Build a six-joint pose holding ``value`` degrees in every joint."""
    return {joint: float(value) for joint in range(1, JOINT_COUNT + 1)}


def test_fast_entrypoint_keeps_defaults_before_ros_args(monkeypatch):
    """Keep application defaults outside the ROS argument section."""
    captured = []
    monkeypatch.setattr(
        piper_teleop_fast, 'teleop_main',
        lambda args: captured.append(args) or 0)

    result = piper_teleop_fast.main(
        ['--side', 'left', '--enable', '--ros-args'])

    assert result == 0
    assert captured == [[
        '--side', 'left', '--enable',
        '--master-topic', '/joint_states_master_left',
        '--quick-reset', '--ros-args',
    ]]


def test_direct_entrypoint_drops_program_name_after_ros_filter(monkeypatch):
    """Do not pass argv[0] to argparse when main receives no explicit list."""
    monkeypatch.setattr(
        piper_teleop, 'remove_ros_args',
        lambda args: ['/installed/piper_teleop', '--side', 'left'])

    assert piper_teleop._application_args(None) == ['--side', 'left']


class FakeClock:
    """A monotonic clock that advances only when someone sleeps."""

    def __init__(self):
        self.now = 0.0
        self.slept = 0.0

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.slept += seconds
        self.now += max(0.0, seconds)


class FakePublisher:
    """Record every published command, optionally failing part way."""

    def __init__(self, fail_at=None):
        self.sent = []
        self.efforts = []
        self.fail_at = fail_at

    def publish(self, msg):
        if self.fail_at is not None and len(self.sent) + 1 == self.fail_at:
            raise KeyboardInterrupt
        self.sent.append([float(v) for v in msg.position[:JOINT_COUNT + 1]])
        self.efforts.append(float(msg.effort[JOINT_COUNT]))


class FakeRclpy:
    """Stand in for rclpy so no ROS context is created."""

    def __init__(self):
        self.ok_value = True
        self.spins = 0

    def ok(self):
        return self.ok_value

    def spin_once(self, node, timeout_sec=0.0):
        self.spins += 1
        if node.on_spin is not None:
            node.on_spin(self.spins)


class FakeNode:
    """Minimal stand-in for TeleopBridge."""

    def __init__(self, follower=None, master=None, enabled=True):
        self.publisher = FakePublisher()
        self.follower = dict(follower) if follower else _pose(0.0)
        self.master = dict(master) if master else _pose(0.0)
        # 夹爪开口（米）。保持 None 表示这一路没有夹爪信息，遥操作不会镜像它。
        self.master_gripper = None
        self.follower_gripper = None
        self.status = object()
        self.error = None if enabled else '整臂未确认使能（state=1 all_enabled=False）'
        self.on_spin = None

    def enable_error(self):
        return self.error


def _options(**overrides):
    """Build a full option set, overriding individual fields per test."""
    options = types.SimpleNamespace(
        side='left', rate=50.0, speed=10, enable=True,
        filter=piper_teleop.DEFAULT_FILTER, filter_tau=0.02,
        alpha_beta_alpha=piper_teleop.DEFAULT_ALPHA_BETA_ALPHA,
        alpha_beta_beta=piper_teleop.DEFAULT_ALPHA_BETA_BETA,
        alpha_beta_max_dt=0.10,
        one_euro_min_cutoff=1.0, one_euro_beta=0.3, one_euro_d_cutoff=1.0,
        deadband_deg=0.0,
        deadband_speed=piper_teleop.DEFAULT_DEADBAND_SPEED_DEG_S,
        gripper=True,
        gripper_effort=piper_teleop.DEFAULT_GRIPPER_EFFORT_NM,
        gripper_deadband=piper_teleop.DEFAULT_GRIPPER_DEADBAND_M,
        gripper_scale=piper_teleop.DEFAULT_GRIPPER_SCALE,
        # 默认关掉平滑级：滤波/死区的单元用例要单独断言它们自己的行为，
        # 整条流水线由 test_the_follow_pipeline_... 覆盖。
        smooth_bandwidth=0.0,
        smooth_max_velocity=piper_teleop.DEFAULT_SMOOTH_MAX_VELOCITY_DEG_S,
        smooth_max_acceleration=(
            piper_teleop.DEFAULT_SMOOTH_MAX_ACCELERATION_DEG_S2),
        smooth_max_jerk=piper_teleop.DEFAULT_SMOOTH_MAX_JERK_DEG_S3,
        align_seconds=0.1, duration=0.4, max_step_deg=0.0,
        return_speed=piper_teleop.DEFAULT_RETURN_SPEED_DEG_S,
        return_max_peak=piper_teleop.DEFAULT_RETURN_MAX_PEAK_DEG_S,
        no_return_home=False)
    for name, value in overrides.items():
        setattr(options, name, value)
    return options


def _summary(last_targets):
    return piper_teleop.RunSummary(
        elapsed=1.0, stopped_by='Ctrl-C 中断', mode='follow', realigns=0,
        limit_hits={}, last_targets=last_targets)


def _run_return(node, options, home, start):
    """Call the return phase with a run summary that reported ``start``."""
    return piper_teleop._run_return_home(
        node, options, home, _summary(start))


def _return_fixture(monkeypatch):
    """Patch the clock and rclpy, and return a 12 degree return setup."""
    clock = FakeClock()
    monkeypatch.setattr(piper_teleop, 'time', clock)
    monkeypatch.setattr(piper_teleop, 'rclpy', FakeRclpy())
    start = _pose(0.0)
    home = _pose(0.0)
    home[1] = 12.0
    return start, home


def test_return_path_ends_exactly_at_home(monkeypatch):
    start, home = _return_fixture(monkeypatch)
    node = FakeNode(follower=home)
    _run_return(node, _options(), home, start)
    published = [[math.degrees(v) for v in command]
                 for command in node.publisher.sent]
    assert len(published) > 25, '回位应当按设定频率持续发完整条轨迹'
    assert published[0][0] == pytest.approx(0.0)
    assert published[-1][0] == pytest.approx(12.0)
    # 每个关节都单调向 home 收敛，没有回退。
    for joint in range(JOINT_COUNT):
        series = [command[joint] for command in published]
        assert all(a <= b + 1e-9 for a, b in zip(series, series[1:]))


def test_return_path_respects_the_peak_speed_ceiling(monkeypatch):
    start, home = _return_fixture(monkeypatch)
    node = FakeNode(follower=home)
    options = _options(return_max_peak=15.0)
    _run_return(node, options, home, start)
    series = [math.degrees(command[0]) for command in node.publisher.sent]
    steps = [b - a for a, b in zip(series, series[1:])]
    # 峰值上限 15 deg/s、周期 20ms：单周期位移不得超过 0.3 度（离散采样留
    # 5% 余量），同时要真的用上这个速度，而不是慢得多。
    assert max(steps) <= 15.0 / options.rate * 1.05
    assert max(steps) > 15.0 / options.rate * 0.9
    # 整条轨迹的位移与时长也要对得上，说明确实是走完了全程而不是被截断。
    assert series[-1] - series[0] == pytest.approx(12.0)


def test_return_path_stops_when_the_arm_stops_reporting_enabled(monkeypatch,
                                                                capsys):
    start, home = _return_fixture(monkeypatch)
    node = FakeNode(follower=home)
    original = node.publisher.publish

    def publish(msg):
        original(msg)
        if len(node.publisher.sent) >= 4:
            node.error = '使能状态已过期（1.2s 没有更新）'

    node.publisher.publish = publish
    _run_return(node, _options(), home, start)
    # 第 4 条之后使能掉线：下个周期的检查必须拦住，不能再盲目发指令。
    assert len(node.publisher.sent) == 4
    out = capsys.readouterr().out
    assert '回位中止' in out


def test_return_is_refused_before_any_publish_when_not_enabled(monkeypatch,
                                                               capsys):
    start, home = _return_fixture(monkeypatch)
    node = FakeNode(follower=home, enabled=False)
    _run_return(node, _options(), home, start)
    assert node.publisher.sent == []
    assert '回位中止' in capsys.readouterr().out


def test_a_second_ctrl_c_stops_the_return_in_place(monkeypatch, capsys):
    start, home = _return_fixture(monkeypatch)
    node = FakeNode(follower=home)
    node.publisher.fail_at = 5  # 第 5 条指令时模拟操作者再按一次 Ctrl-C
    _run_return(node, _options(), home, start)
    assert len(node.publisher.sent) == 4
    out = capsys.readouterr().out
    assert '已中止回位' in out
    assert '停在原地' in out


def test_the_return_banner_is_printed_before_the_first_command(monkeypatch):
    start, home = _return_fixture(monkeypatch)
    events = []
    monkeypatch.setattr(
        piper_teleop, 'print',
        lambda *args, **kwargs: events.append(
            ('print', args[0] if args else '')), raising=False)
    node = FakeNode(follower=home)
    original = node.publisher.publish

    def publish(msg):
        events.append(('publish', msg))
        original(msg)

    node.publisher.publish = publish
    _run_return(node, _options(), home, start)
    banners = [index for index, (kind, payload) in enumerate(events)
               if kind == 'print' and '正在回到初始位置' in str(payload)]
    commands = [index for index, (kind, _) in enumerate(events)
                if kind == 'publish']
    # 机械臂还在动而操作者以为已经停了，是本工具最大的安全隐患；这条提示
    # 必须先于第一条运动指令出现。
    assert banners, '回位开始时必须打印醒目提示'
    assert commands, '这次运行应当真的发了指令'
    assert banners[0] < commands[0]


def test_dry_run_prints_the_plan_and_sends_nothing(monkeypatch, capsys):
    start, home = _return_fixture(monkeypatch)
    node = FakeNode(follower=home)
    _run_return(node, _options(enable=False), home, start)
    out = capsys.readouterr().out
    assert node.publisher.sent == []
    assert '不发送任何内容' in out
    assert '回位时长' in out
    assert '峰值' in out


def test_dry_run_reports_a_home_pose_that_would_be_clamped(
        monkeypatch, capsys):
    start, home = _return_fixture(monkeypatch)
    home[3] = 2.15  # joint3 上限 2.0：home 落在指令范围外
    node = FakeNode(follower=home)
    _run_return(node, _options(enable=False), home, start)
    out = capsys.readouterr().out
    assert '会触及限位' in out
    assert 'j3' in out


def test_return_falls_back_to_the_measured_pose(monkeypatch, capsys):
    _return_fixture(monkeypatch)
    node = FakeNode(follower=_pose(3.0))
    _run_return(node, _options(enable=False), _pose(0.0), None)
    out = capsys.readouterr().out
    assert 'follower 的实测姿态' in out
    assert node.publisher.sent == []


def test_default_speed_does_not_throttle_the_follower():
    # 2026-09-23 实测：`--speed 10` 是真正的跟随瓶颈——节点把它转发给
    # MotionCtrl_2 当作「整臂最大速度的百分比」（10% × 3 rad/s ≈ 17.2 deg/s)，而
    # 固件 max_joint_spd=300 在流式位置指令这条路径上并不是速度硬上限：把百分比
    # 提到 100% 后 follower 的持续速度升到 45.6~86.2 deg/s 并跟着拖动速度走。
    # 遥操作的目标是跟随 master 原值，所以默认不给它再加一道软件限速。
    assert piper_teleop.DEFAULT_SPEED == 100


def test_dry_run_warns_when_the_return_peak_exceeds_the_measured_capability(
        monkeypatch, capsys):
    start, home = _return_fixture(monkeypatch)
    node = FakeNode(follower=home)
    # 平均速度与峰值都调大，计划的峰值才会真的超过实测能力（只调峰值时，
    # 时长由平均速度决定，峰值仍被压在 15 deg/s 以内）。
    _run_return(node, _options(enable=False, return_speed=200.0,
                               return_max_peak=500.0), home, start)
    out = capsys.readouterr().out
    assert '高于实测跟随能力' in out
    assert node.publisher.sent == []


def _align_run(monkeypatch, master_at_spin):
    """Run the teleop for one short session with a scripted master."""
    clock = FakeClock()
    fake_rclpy = FakeRclpy()
    monkeypatch.setattr(piper_teleop, 'time', clock)
    monkeypatch.setattr(piper_teleop, 'rclpy', fake_rclpy)
    node = FakeNode()

    def on_spin(count):
        if count in master_at_spin:
            node.master = _pose(master_at_spin[count])

    node.on_spin = on_spin
    options = _options(align_seconds=0.1, duration=0.4)
    return piper_teleop._run_teleop(node, options), node


def test_default_alignment_finishes_in_two_seconds():
    # 已知起终点的对齐用 2 秒 minimum-jerk 轨迹，不再慢速爬行 4 秒。
    assert piper_teleop.DEFAULT_ALIGN_SECONDS == 2.0


def test_a_dragged_master_goes_straight_to_following(monkeypatch, capsys):
    # 实测教训：操作者一直握着 master 拖时，对齐冻结的目标永远追不上，重新
    # 对齐会把整个会话耗掉（25 秒里重对齐 3 次、最终模式仍是 align，表现为
    # "几乎不跟随"）。这种情形必须直接进入跟随。
    summary, _ = _align_run(
        monkeypatch, {count: count * 2.0 for count in range(5, 20)})
    out = capsys.readouterr().out
    assert summary.realigns == 0
    assert summary.mode == 'follow'
    assert '仍在被拖动' in out


def test_a_static_bump_is_still_realigned_once(monkeypatch, capsys):
    # 反过来，"碰了一下又停下"仍值得重新对齐一次：否则那一跳会当场砸到
    # follower 上。
    summary, _ = _align_run(monkeypatch, {5: 5.0})
    out = capsys.readouterr().out
    assert summary.realigns == 1
    assert summary.mode == 'follow'
    assert '现在已静止' in out


def test_realigning_is_bounded(monkeypatch, capsys):
    # 反复"碰一下又停"也不能无限重对齐：到上限之后一律进入跟随。
    summary, _ = _align_run(
        monkeypatch, {5: 5.0, 11: 10.0, 17: 15.0, 23: 20.0})
    assert summary.realigns == piper_teleop.MAX_REALIGNS
    assert summary.mode == 'follow'


def test_the_filter_is_seeded_with_the_last_published_target(monkeypatch):
    # 进入跟随时若 master 已偏离，用"最后一次发布的目标"做种子，命令流才连续；
    # 若用 master 当前值做种子，差多少就会当场跳多少。
    clock = FakeClock()
    monkeypatch.setattr(piper_teleop, 'time', clock)
    monkeypatch.setattr(piper_teleop, 'rclpy', FakeRclpy())
    node = FakeNode()

    def on_spin(count):
        # 1.5 度 < 重新对齐阈值：对齐期间被轻轻挪动，不该触发重新对齐。
        if count >= 5:
            node.master = _pose(1.5)

    node.on_spin = on_spin
    options = _options(align_seconds=0.1, duration=0.4, filter='lowpass')
    piper_teleop._run_teleop(node, options)
    series = [math.degrees(command[0]) for command in node.publisher.sent]
    after = [value for value in series if value > 1e-9]
    # 低通 α=0.5：种子为最后一次发布的目标（0）时第一拍是 0.75；
    # 若按 master 当前值（1.5）做种子，则第一拍就是 1.5。
    assert after[0] == pytest.approx(0.75, abs=1e-6)


def _follow_run(monkeypatch, **overrides):
    """Run the teleop while the master steps 10 degrees after alignment."""
    clock = FakeClock()
    fake_rclpy = FakeRclpy()
    monkeypatch.setattr(piper_teleop, 'time', clock)
    monkeypatch.setattr(piper_teleop, 'rclpy', fake_rclpy)
    node = FakeNode()
    step = _pose(0.0)
    step[1] = 10.0

    def on_spin(count):
        # 第 8 次 spin 时对齐已经完成（进度在对齐时长为 0.1s、周期 20ms 时
        # 于第 6 个周期到达 100%），此时 master 才动，避免触发重新对齐。
        if count >= 8:
            node.master = dict(step)

    node.on_spin = on_spin
    options = _options(duration=0.4, **overrides)
    summary = piper_teleop._run_teleop(node, options)
    series = [math.degrees(command[0]) for command in node.publisher.sent]
    return summary, [value for value in series if value > 1e-9]


def test_a_small_hand_wobble_does_not_bias_the_target(monkeypatch):
    # 滤波级的单元用例（平滑级默认关闭，见 _options）：5 Hz、±0.15 度的抖晃幅度
    # 低于死区，但速度门限会放行它（那是为了让慢拖不被切成台阶）。
    assert piper_teleop.DEFAULT_DEADBAND_DEG == 0.2

    clock = FakeClock()
    monkeypatch.setattr(piper_teleop, 'time', clock)
    monkeypatch.setattr(piper_teleop, 'rclpy', FakeRclpy())
    node = FakeNode()

    def on_spin(count):
        if count >= 6:
            node.master = _pose(10.0)
        if count >= 8:
            node.master = _pose(10.0 + 0.15 * math.sin(count * 0.6283))

    node.on_spin = on_spin
    # 0.4 秒只够看阶跃暂态、量不到稳态，所以这里跑长一些：断言的是**收敛后**的值。
    options = _options(align_seconds=0.1, duration=1.2, deadband_deg=0.2)
    piper_teleop._run_teleop(node, options)
    series = [math.degrees(command[0]) for command in node.publisher.sent]
    # 判据取**整数个抖晃周期**的均值：单个采样点看的是被滤过抖晃的瞬时相位，均值
    # 才说明有没有稳态偏置。5 Hz 在 50 Hz 下正好 10 拍一个周期，末 20 拍是两个
    # 周期；实测该均值为 10.0000 度（λ=0.65 与 0.40 都一样）。
    assert sum(series[-20:]) / 20 == pytest.approx(10.0, abs=0.03)
    # 幅度在这个频段**不会**被这一级抹平：实测末 20 拍峰峰 0.305 度，与输入自身
    # 的 0.30 度同量级，所以这里只守住"不放大"。整条链的衰减由下一条用例覆盖。
    assert max(series[-20:]) - min(series[-20:]) < 0.35


def test_the_full_chain_attenuates_hand_tremor(monkeypatch):
    # 手抖集中在 8~12 Hz，这一档才是"整条链压掉多少"要看的频段。这里把平滑级按
    # 真机默认打开（_options 默认关闭它，原因见上一条用例的说明）。
    # 稳定优先配置 λ=0.65、带宽 12 rad/s 会进一步压低这两个频段。
    for hz, amp, step, bound in ((10.0, 0.3, 1.2566, 0.1),
                                 (5.0, 0.15, 0.6283, 0.2)):
        monkeypatch.setattr(piper_teleop, 'time', FakeClock())
        monkeypatch.setattr(piper_teleop, 'rclpy', FakeRclpy())
        node = FakeNode()

        def on_spin(count, amp=amp, step=step):
            if count >= 6:
                node.master = _pose(10.0)
            if count >= 8:
                node.master = _pose(10.0 + amp * math.sin(count * step))

        node.on_spin = on_spin
        options = _options(
            align_seconds=0.1, duration=1.6, deadband_deg=0.2,
            smooth_bandwidth=piper_teleop.DEFAULT_SMOOTH_BANDWIDTH_RAD_S)
        piper_teleop._run_teleop(node, options)
        series = [math.degrees(command[0]) for command in node.publisher.sent]
        tail = series[-20:]
        assert sum(tail) / 20 == pytest.approx(10.0, abs=0.03), hz
        assert max(tail) - min(tail) < bound, (hz, max(tail) - min(tail))


def test_the_follow_pipeline_is_deadband_then_filter_then_smoother(
        monkeypatch, capsys):
    # 顺序是：deadband -> α-β -> 五次插值平滑。流水线跑完后目标
    # 应当贴近 master，且带宽一行要如实打印出来。
    clock = FakeClock()
    monkeypatch.setattr(piper_teleop, 'time', clock)
    monkeypatch.setattr(piper_teleop, 'rclpy', FakeRclpy())
    node = FakeNode()

    def on_spin(count):
        if count >= 8:
            node.master = _pose(20.0)

    node.on_spin = on_spin
    options = _options(align_seconds=0.1, duration=1.5,
                       deadband_deg=piper_teleop.DEFAULT_DEADBAND_DEG,
                       smooth_bandwidth=piper_teleop.DEFAULT_SMOOTH_BANDWIDTH_RAD_S)
    summary = piper_teleop._run_teleop(node, options)
    out = capsys.readouterr().out
    assert summary.mode == 'follow'
    assert '五次插值平滑' in out and '死区' in out
    series = [math.degrees(command[0]) for command in node.publisher.sent]
    # 平滑级限制加速度，所以它需要几个周期才能走完 20 度；末值应当已经到位。
    assert series[-1] == pytest.approx(20.0, abs=0.5)
    # 这里用的是"一个周期内跳 20 度"这种非物理的参考（真实拖动每周期最多
    # 3.6 度），阶跃会让速度估计出现尖峰并被前馈进去；真实拖动必须再用
    # piper_teleop_verify 核对过冲。
    assert max(series) < 20.0 * 1.4


def test_a_large_move_still_goes_through_the_deadband(monkeypatch):
    clock = FakeClock()
    monkeypatch.setattr(piper_teleop, 'time', clock)
    monkeypatch.setattr(piper_teleop, 'rclpy', FakeRclpy())
    node = FakeNode()

    def on_spin(count):
        if count >= 8:
            node.master = _pose(30.0)

    node.on_spin = on_spin
    options = _options(align_seconds=0.1, duration=0.4, deadband_deg=0.2)
    piper_teleop._run_teleop(node, options)
    series = [math.degrees(command[0]) for command in node.publisher.sent]
    assert series[-1] > 29.9


def test_follow_phase_uses_alpha_beta_by_default(monkeypatch):
    assert piper_teleop.DEFAULT_FILTER == 'alpha-beta'
    summary, after_step = _follow_run(monkeypatch)
    assert summary.mode == 'follow'
    # λ=0.65 导出的 α=0.5775：从 0 到 10 度的第一拍走到 5.775 度；之后位置与速度
    # 状态共同收敛。这里关掉后级平滑，
    # 单独验证 α-β 的行为。
    assert after_step[0] == pytest.approx(5.775, abs=1e-6)
    assert max(after_step) < 11.0
    assert after_step[-1] > 9.99


def test_gripper_follows_the_master_without_a_jump(monkeypatch):
    """夹爪并入同一条链：开局就是自身开口，收敛到 master 的开口，没有一步跳变。"""
    clock = FakeClock()
    monkeypatch.setattr(piper_teleop, 'time', clock)
    monkeypatch.setattr(piper_teleop, 'rclpy', FakeRclpy())
    node = FakeNode()
    node.follower_gripper = 0.005            # 从臂夹爪 5mm
    node.master_gripper = 0.005

    def on_spin(count):
        node.master = _pose(6.0)
        if count >= 3:
            node.master_gripper = 0.050      # 对齐期间 master 张开到 50mm

    node.on_spin = on_spin
    options = _options(align_seconds=0.2, duration=1.0, gripper_scale=1.0)
    summary = piper_teleop._run_teleop(node, options)
    assert summary.mode == 'follow'
    grips = [command[JOINT_COUNT] for command in node.publisher.sent]
    assert grips[0] == pytest.approx(0.005, abs=1e-9)
    assert all(0.0 <= grip <= 0.08 for grip in grips)
    # 45mm 的位移分很多拍走完（单拍最大 15mm 以内），不是一次跳过去
    assert max(abs(b - a) for a, b in zip(grips, grips[1:])) < 0.015
    assert grips[-1] == pytest.approx(0.050, abs=0.002)
    # 夹持力随指令发出（节点会钳到 [0.5, 3] N·m）
    assert node.publisher.efforts[-1] == pytest.approx(options.gripper_effort)
    assert summary.last_gripper == pytest.approx(grips[-1], abs=1e-9)


def test_gripper_command_is_clamped_into_the_commandable_range(monkeypatch):
    """master 报出越界开口时，指令被钳到 0~80mm，不会原样转发。"""
    clock = FakeClock()
    monkeypatch.setattr(piper_teleop, 'time', clock)
    monkeypatch.setattr(piper_teleop, 'rclpy', FakeRclpy())
    node = FakeNode()
    node.follower_gripper = 0.0
    node.master_gripper = 0.0

    def on_spin(count):
        node.master = _pose(0.0)
        if count >= 3:
            node.master_gripper = 0.5        # 0.5m：远超 80mm 行程

    node.on_spin = on_spin
    piper_teleop._run_teleop(node, _options(align_seconds=0.2, duration=0.8,
                                            gripper_scale=1.0))
    grips = [command[JOINT_COUNT] for command in node.publisher.sent]
    assert all(0.0 <= grip <= 0.08 for grip in grips)
    assert grips[-1] == pytest.approx(0.08, abs=1e-9)


def test_no_gripper_keeps_the_old_command_shape(monkeypatch):
    """--no-gripper 时第 7 项与夹持力都保持 0，即加入夹爪之前的指令形状。"""
    clock = FakeClock()
    monkeypatch.setattr(piper_teleop, 'time', clock)
    monkeypatch.setattr(piper_teleop, 'rclpy', FakeRclpy())
    node = FakeNode()
    node.follower_gripper = 0.01
    node.master_gripper = 0.05
    node.on_spin = lambda count: setattr(node, 'master', _pose(4.0))
    options = _options(align_seconds=0.2, duration=0.6, gripper=False)
    piper_teleop._run_teleop(node, options)
    assert all(command[JOINT_COUNT] == 0.0 for command in node.publisher.sent)
    assert all(effort == 0.0 for effort in node.publisher.efforts)


def test_gripper_scale_multiplies_the_master_opening(monkeypatch):
    """默认按 1.3 倍换算：主臂夹爪行程只有从臂的约 1/1.3，走满行程正好到 80mm。"""
    clock = FakeClock()
    monkeypatch.setattr(piper_teleop, 'time', clock)
    monkeypatch.setattr(piper_teleop, 'rclpy', FakeRclpy())
    node = FakeNode()
    node.follower_gripper = 0.0
    node.master_gripper = 0.0

    def on_spin(count):
        node.master = _pose(0.0)
        if count >= 3:
            node.master_gripper = 0.030      # 主臂 30mm
        if count >= 40:
            node.master_gripper = 0.080      # 拖到主臂行程尽头

    node.on_spin = on_spin
    options = _options(align_seconds=0.2, duration=1.2)
    assert options.gripper_scale == pytest.approx(1.3)
    piper_teleop._run_teleop(node, options)
    grips = [command[JOINT_COUNT] for command in node.publisher.sent]
    # 30mm × 1.3 = 39mm
    assert max(grips) >= 0.039
    assert any(abs(grip - 0.039) < 0.002 for grip in grips)
    # 主臂 80mm × 1.3 = 104mm，被从臂行程钳到 80mm
    assert grips[-1] == pytest.approx(piper_teleop.GRIPPER_OPEN_MAX_M, abs=1e-6)
    assert all(0.0 <= grip <= 0.08 for grip in grips)


def test_gripper_scale_one_is_a_pure_mirror(monkeypatch):
    """--gripper-scale 1.0 时目标就等于 master 的读数（两台夹爪相同时用）。"""
    clock = FakeClock()
    monkeypatch.setattr(piper_teleop, 'time', clock)
    monkeypatch.setattr(piper_teleop, 'rclpy', FakeRclpy())
    node = FakeNode()
    node.follower_gripper = 0.0
    node.master_gripper = 0.0

    def on_spin(count):
        node.master = _pose(0.0)
        if count >= 3:
            node.master_gripper = 0.040

    node.on_spin = on_spin
    options = _options(align_seconds=0.2, duration=1.0, gripper_scale=1.0)
    piper_teleop._run_teleop(node, options)
    assert node.publisher.sent[-1][JOINT_COUNT] == pytest.approx(
        0.040, abs=0.001)


def test_follow_phase_one_euro_remains_selectable(monkeypatch):
    summary, after_step = _follow_run(monkeypatch, filter='one-euro')
    assert summary.mode == 'follow'
    assert 5.0 < after_step[0] < 10.0
    assert after_step[-1] > 9.99


def test_follow_phase_low_pass_still_selectable_for_comparison(monkeypatch):
    summary, after_step = _follow_run(monkeypatch, filter='lowpass')
    assert summary.mode == 'follow'
    # α = dt/(τ+dt) = 0.5（实测间隔正好等于 τ）：第一拍走到中点。
    assert after_step[0] == pytest.approx(5.0, abs=1e-6)
    assert after_step[-1] > 9.99


def test_follow_phase_without_filter_passes_the_master_through(monkeypatch):
    summary, after_step = _follow_run(monkeypatch, filter='none')
    assert summary.mode == 'follow'
    # 不滤波：同步优先，第一拍就是 master 的原值。
    assert after_step[0] == pytest.approx(10.0)

