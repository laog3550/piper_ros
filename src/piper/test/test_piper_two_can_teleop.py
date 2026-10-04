"""Tests for the single-side two-CAN teleop session."""

import math
import time
from types import SimpleNamespace

import can
import pytest

import piper.piper_two_can_align as align_module
from piper.piper_two_can_align import ENABLE_CAN_ID, SafetyError
from piper.piper_two_can_teleop import (
    EXIT_FAILED,
    EXIT_OK,
    EXIT_PENDING_RELEASE,
    EXIT_REFUSED,
    ConsoleOperator,
    FollowChain,
    GuardedBus,
    OperatorLink,
    TeleopConfig,
    TeleopSession,
    TeleopState,
    build_config,
)

STEP_S = 1.0 / 200.0


class FakeClock:
    """A monotonic clock the fake bus advances, so loops terminate."""

    def __init__(self, start: float = 0.0):
        self.now = float(start)

    def monotonic(self) -> float:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += float(seconds)


def _angles(first, second):
    return b''.join(
        int(round(value * 1000.0)).to_bytes(4, 'big', signed=True)
        for value in (first, second)
    )


def _gripper(opening_m, enabled=True):
    return (int(round(opening_m * 1e6)).to_bytes(4, 'big', signed=True)
            + (1000).to_bytes(2, 'big')
            + bytes((0xC0 if enabled else 0x80, 0x00)))


def _enable_frame(enable: bool) -> bytes:
    return bytes((7, 0x02 if enable else 0x01)) + bytes(6)


def _reachable_pose(value: float):
    """Return a pose inside the joint limits, using ``value``."""
    signs = {1: 1.0, 2: 1.0, 3: -1.0, 4: 1.0, 5: 1.0, 6: 1.0}
    return {joint: value * sign for joint, sign in signs.items()}


def _frame(can_id: int, data: bytes):
    return can.Message(arbitration_id=can_id, data=data,
                       is_extended_id=False)


def _error_frame(classes: int, data: bytes = bytes(8)):
    from can import Message
    message = Message(arbitration_id=classes, data=data,
                      is_extended_id=False)
    message.is_error_frame = True
    return message


class FakeSide:
    """A two-arm bus that answers like the real one, on a fake clock."""

    def __init__(self, clock: FakeClock, *, enabled: bool = False,
                 master_offset: bool = True, obey_enable: bool = True):
        self.clock = clock
        self.enabled = enabled
        self.obey_enable = obey_enable
        self.master_offset = master_offset
        self.master = {joint: 0.0 for joint in range(1, 7)}
        self.follower = {joint: 0.0 for joint in range(1, 7)}
        self.master_gripper = 0.03
        self.follower_gripper = 0.03
        self.sent = []
        self.shutdown_called = False
        self.cycles = 0
        self.emit_low_speed = True
        self.double_family_rate = False
        self.partial_enable_joints = ()
        self.error_frames_every_cycle = False
        self.master_silent_after_stream_cycles = None
        self.follower_silent_after_stream_cycles = None
        self.disable_after_cycle = None
        self.inject_after_cycle = []
        self.stream_inject = []
        self.ignore_disable = False
        self.ignore_enable = False
        self.hold_follower = False
        self.track_commands = True
        self._pending = []
        self._low_speed_index = 0
        self._master_silent_at = None
        self._follower_silent_at = None
        self._stream_at = None

    # ---- 收 -----------------------------------------------------------

    def _low_speed_frames(self):
        """Emit two of the six shared low-speed IDs per cycle."""
        frames = []
        for _ in range(2):
            joint = self._low_speed_index % 6 + 1
            self._low_speed_index += 1
            enabled = self.enabled and (
                not self.partial_enable_joints
                or joint in self.partial_enable_joints)
            status = 0x40 if enabled else 0x00
            frames.append(_frame(0x260 + joint,
                                 bytes.fromhex('00 E6 00 20 00') + bytes(
                                     (status, 0x00, 0x00))))
        return frames

    def _family(self, base, pose, gripper, mode):
        """Emit one arm's whole 0x2Ax/0x2Cx high-speed window."""
        return [
            _frame(base, bytes((mode, 0, 0, 0, 0, 0, 0, 0))),
            _frame(base + 1, bytes(8)),
            _frame(base + 2, bytes(8)),
            _frame(base + 3, bytes(8)),
            _frame(base + 4, _angles(pose[1], pose[2])),
            _frame(base + 5, _angles(pose[3], pose[4])),
            _frame(base + 6, _angles(pose[5], pose[6])),
            _frame(base + 7, _gripper(gripper)),
        ]

    def _master_is_silent(self):
        return (self._master_silent_at is not None
                and self.cycles >= self._master_silent_at)

    def _cycle(self):
        frames = []
        follower_silent = (self._follower_silent_at is not None
                           and self.cycles >= self._follower_silent_at)
        if not follower_silent:
            frames.extend(self._family(0x2A1, self.follower,
                                       self.follower_gripper, 0x00))
            if self.double_family_rate:
                # 未配置的从臂窗口：两台臂都报默认地址，频率翻倍。
                frames.extend(self._family(0x2A1, self.follower,
                                           self.follower_gripper, 0x00))
        if self.master_offset and not self._master_is_silent():
            frames.extend(self._family(0x2C1, self.master,
                                       self.master_gripper, 0x02))
        if self.emit_low_speed:
            frames.extend(self._low_speed_frames())
        if self.error_frames_every_cycle:
            frames.append(_error_frame(0x0C))
        return frames

    def recv(self, timeout=None):
        """Return the next frame; each full bus cycle advances the clock."""
        if not self._pending:
            self.clock.advance(STEP_S)
            self.cycles += 1
            if (self.disable_after_cycle is not None
                    and self.cycles > self.disable_after_cycle):
                self.enabled = False
            if (self.inject_after_cycle
                    and self.cycles >= self.inject_after_cycle[0][0]):
                _, frame = self.inject_after_cycle.pop(0)
                return frame
            if self._stream_at is not None and self.stream_inject:
                delay, action = self.stream_inject[0]
                if self.cycles >= self._stream_at + delay:
                    self.stream_inject.pop(0)
                    if callable(action):
                        action()
                    else:
                        return action
            self._pending = self._cycle()
        return self._pending.pop(0)

    # ---- 发 -----------------------------------------------------------

    def send(self, message):
        """Record one frame, follow the targets and obey the broadcasts."""
        self.sent.append(message)
        can_id = message.arbitration_id
        if (can_id == ENABLE_CAN_ID and self.obey_enable
                and len(message.data) >= 2):
            enable = message.data[1] == 0x02
            if self.ignore_disable and not enable:
                return
            if self.ignore_enable and enable:
                return
            if enable != self.enabled:
                # 已在途的旧载荷不能代表新的使能状态。
                self._pending = []
            self.enabled = enable
        if can_id == 0x155 and self._stream_at is None:
            self._stream_at = self.cycles
            if self.master_silent_after_stream_cycles is not None:
                self._master_silent_at = (
                    self.cycles + self.master_silent_after_stream_cycles)
            if self.follower_silent_after_stream_cycles is not None:
                self._follower_silent_at = (
                    self.cycles + self.follower_silent_after_stream_cycles)
        if self.track_commands and not self.hold_follower:
            pairs = {0x155: (1, 2), 0x156: (3, 4), 0x157: (5, 6)}
            if can_id in pairs:
                for index, joint in enumerate(pairs[can_id]):
                    raw = message.data[index * 4:(index + 1) * 4]
                    self.follower[joint] = (
                        int.from_bytes(raw, 'big', signed=True) / 1000.0)
            elif can_id == 0x159 and len(message.data) >= 4:
                self.follower_gripper = max(0.0, min(
                    int.from_bytes(message.data[0:4], 'big', signed=True)
                    / 1e6, 0.08))

    def shutdown(self):
        self.shutdown_called = True

    # ---- 断言辅助 -----------------------------------------------------

    def ids(self):
        return [message.arbitration_id for message in self.sent]

    def payload_of(self, can_id):
        """Return the most recent payload sent with this CAN ID."""
        payload = None
        for message in self.sent:
            if message.arbitration_id == can_id:
                payload = message.data
        return payload


class ScriptedOperator(OperatorLink):
    """An operator that answers from a script instead of a terminal."""

    def __init__(self, *, start=True, teach=True, release=True,
                 stop_after_polls=None):
        self.start = start
        self.teach = teach
        self.release = release
        self.stop_after_polls = stop_after_polls
        self.polls = 0
        self.messages = []

    def notify(self, message):
        self.messages.append(message)

    def require_start(self, timeout_s):
        if not self.start:
            raise SafetyError('测试：操作员没有按开始')

    def wait_teach_engaged(self, timeout_s):
        return self.teach

    def poll_stop(self):
        self.polls += 1
        if (self.stop_after_polls is not None
                and self.polls > self.stop_after_polls):
            return '测试：操作员请求停止'
        return None

    def wait_teach_released(self, timeout_s):
        return self.release


def make_config(**overrides) -> TeleopConfig:
    """Build a fast test configuration."""
    settings = dict(
        side='left',
        port='can_test',
        send=True,
        command_rate_hz=200.0,
        preflight_seconds=1.0,
        arm_verify_s=0.05,
        disable_verify_s=0.05,
        disable_retries=0,
        align_rate_hz=50.0,
        min_align_s=0.2,
        max_peak_deg_s=1000.0,
        settle_s=0.0,
        goal_tolerance_deg=0.5,
    )
    settings.update(overrides)
    return TeleopConfig(**settings)


@pytest.fixture
def harness(monkeypatch):
    """Install a fake clock, fake bus and a captured session log."""
    clock = FakeClock()
    monkeypatch.setattr(time, 'monotonic', clock.monotonic)

    def build(*, config=None, side=None, operator=None, **kwargs):
        bus = FakeSide(clock, **kwargs)
        monkeypatch.setattr(can, 'Bus', lambda **options: bus)
        session = TeleopSession(
            config or make_config(), operator or ScriptedOperator(),
            log=lambda *args: None)
        return session, bus, (operator or session.operator)

    return SimpleNamespace(clock=clock, build=build)


def test_build_config_reads_every_option():
    from piper.piper_two_can_teleop import argument_parser

    options = argument_parser().parse_args(
        ['--side', 'right', '--rate', '100', '--no-gripper'])
    config = build_config(options)

    assert config.side == 'right'
    assert config.port == 'can_right'
    assert config.command_rate_hz == 100.0
    assert config.gripper is False
    assert config.send is False


def test_dry_run_reaches_the_gates_without_sending(harness):
    session, bus, _ = harness.build(
        config=make_config(send=False), operator=ScriptedOperator())

    code = session.run()

    assert code == EXIT_OK
    assert bus.sent == []
    assert session.state is TeleopState.IDLE_DISABLED
    assert session.report.identity.confirmed


def test_preflight_refuses_without_the_master_feedback_family(harness):
    session, bus, _ = harness.build(master_offset=False)

    code = session.run()

    assert code == EXIT_REFUSED
    assert bus.sent == []
    assert session.state is TeleopState.OFFLINE


def test_preflight_refuses_when_the_side_is_already_enabled(harness):
    session, bus, _ = harness.build(enabled=True)

    code = session.run()

    assert code == EXIT_REFUSED
    assert bus.sent == []


def test_start_request_is_required_before_any_frame(harness):
    operator = ScriptedOperator(start=False)
    session, bus, _ = harness.build(operator=operator)

    code = session.run()

    assert code == EXIT_REFUSED
    assert bus.sent == []


def test_full_session_streams_only_follower_ids_and_disables(harness):
    session, bus, _ = harness.build(
        config=make_config(duration_s=0.1))

    code = session.run()

    assert code == EXIT_OK
    assert session.state is TeleopState.IDLE_DISABLED
    ids = bus.ids()
    assert ids[0] == ENABLE_CAN_ID
    assert bus.sent[0].data == _enable_frame(True)
    assert ids[-1] == ENABLE_CAN_ID
    assert bus.sent[-1].data == _enable_frame(False)
    assert not any(0x170 <= can_id <= 0x17F for can_id in ids)
    assert 0x470 not in ids
    assert 0x150 not in ids
    assert set(ids) <= {ENABLE_CAN_ID, 0x151, 0x155, 0x156, 0x157, 0x159}
    assert 0x151 in ids and 0x155 in ids and 0x157 in ids


def test_targets_are_sent_as_signed_millidegrees(harness):
    session, bus, _ = harness.build(config=make_config(duration_s=0.1))
    bus.master = {joint: 0.0 for joint in range(1, 7)}
    bus.master[6] = 4.0

    code = session.run()

    assert code == EXIT_OK
    payload = bus.payload_of(0x157)
    last_joint = int.from_bytes(payload[4:8], 'big', signed=True) / 1000.0
    assert last_joint == pytest.approx(4.0, abs=0.1)


def test_master_joint_feedback_timeout_latches_a_fault(harness):
    session, bus, _ = harness.build(
        config=make_config(duration_s=5.0),
        operator=ScriptedOperator(stop_after_polls=100000))
    # 开始下发关节目标后，主臂反馈族在 5 个周期内静默。
    bus.master_silent_after_stream_cycles = 5

    code = session.run()

    assert code == EXIT_FAILED
    assert session.state is TeleopState.FAULT_LATCHED
    assert '反馈' in (session.fault or '')
    assert bus.sent[-1].data == _enable_frame(False)


def test_disable_readback_failure_is_fatal(harness):
    session, bus, _ = harness.build(config=make_config(duration_s=0.1))
    bus.ignore_disable = True

    code = session.run()

    assert code == EXIT_FAILED
    assert session.state is TeleopState.FAULT_LATCHED
    assert '失能' in (session.fault or '')


def test_teach_release_refusal_keeps_the_arms_enabled(harness):
    operator = ScriptedOperator(release=False)
    session, bus, _ = harness.build(
        config=make_config(duration_s=0.1), operator=operator)

    code = session.run()

    assert code == EXIT_PENDING_RELEASE
    assert bus.sent[-1].arbitration_id == 0x151
    assert bus.sent[-1].data[0] == 0x00
    assert not any(message.data == _enable_frame(False)
                   for message in bus.sent)


def test_pose_gate_refuses_a_large_master_follower_difference(harness):
    session, bus, _ = harness.build()
    bus.master = _reachable_pose(40.0)

    code = session.run()

    assert code == EXIT_REFUSED
    assert bus.sent == []
    assert ('门禁' in (session.fault or '')
            or session.state is TeleopState.OFFLINE)


def test_external_control_frame_during_follow_latches_a_fault(harness):
    session, bus, _ = harness.build(config=make_config(duration_s=5.0))
    bus.stream_inject = [(2, _frame(0x155, bytes(8)))]
    session.operator.stop_after_polls = 100000

    code = session.run()

    assert code == EXIT_FAILED
    assert session.state is TeleopState.FAULT_LATCHED
    assert '外部控制帧' in (session.fault or '')


def test_error_frames_abort_under_the_strict_policy(harness):
    session, bus, _ = harness.build(
        config=make_config(duration_s=5.0, error_policy='strict'))
    bus.stream_inject = [(2, _error_frame(0x0C))]
    session.operator.stop_after_polls = 100000

    code = session.run()

    assert code == EXIT_FAILED
    assert 'CAN 错误帧' in (session.fault or '')


def test_default_policy_is_recoverable(harness):
    assert TeleopConfig().error_policy == 'recoverable'


def test_sustained_error_storm_latches_a_fault(harness):
    session, bus, _ = harness.build(config=make_config(duration_s=5.0))
    # 门禁窗口干净，错误帧风暴在开始下发目标之后才出现。
    bus.stream_inject = [
        (2, lambda: setattr(bus, 'error_frames_every_cycle', True))]
    session.operator.stop_after_polls = 100000

    code = session.run()

    assert code == EXIT_FAILED
    assert session.state is TeleopState.FAULT_LATCHED
    assert '持续速率' in (session.fault or '')


def test_recoverable_policy_tolerates_isolated_error_frames(harness):
    session, bus, _ = harness.build(
        config=make_config(duration_s=0.1, error_policy='recoverable'))
    bus.stream_inject = [(2, _error_frame(0x0C))]

    code = session.run()

    assert code == EXIT_OK
    assert session.state is TeleopState.IDLE_DISABLED


def test_enable_readback_must_cover_every_joint(harness):
    session, bus, _ = harness.build(config=make_config(duration_s=0.1))
    # 使能广播发出去以后，低速反馈仍然全部报告 disabled。
    bus.ignore_enable = True

    code = session.run()

    assert code == EXIT_FAILED
    assert session.state is TeleopState.FAULT_LATCHED
    assert '使能位不是一致的 True' in (session.fault or '')
    assert bus.sent[-1].data == _enable_frame(False)


def test_preflight_refuses_a_conflicting_follower_window(harness):
    session, bus, _ = harness.build()
    bus.double_family_rate = True

    code = session.run()

    assert code == EXIT_REFUSED
    assert bus.sent == []


def test_follower_joint_feedback_timeout_latches_a_fault(harness):
    session, bus, _ = harness.build(
        config=make_config(duration_s=5.0),
        operator=ScriptedOperator(stop_after_polls=100000))
    bus.follower_silent_after_stream_cycles = 5

    code = session.run()

    assert code == EXIT_FAILED
    assert session.state is TeleopState.FAULT_LATCHED
    assert bus.sent[-1].data == _enable_frame(False)


def test_partial_enable_readback_is_refused(harness):
    session, bus, _ = harness.build(config=make_config(duration_s=0.1))
    # 只有 j1～j3 报告 enabled：窗口内的样本不是一致的 True。
    bus.partial_enable_joints = (1, 2, 3)

    code = session.run()

    assert code == EXIT_FAILED
    assert '使能位不是一致的 True' in (session.fault or '')


def test_command_rate_is_bounded_by_the_configured_rate(harness):
    session, bus, _ = harness.build(
        config=make_config(duration_s=0.2, command_rate_hz=200.0))

    code = session.run()

    assert code == EXIT_OK
    joint_frames = sum(1 for message in bus.sent
                       if message.arbitration_id == 0x155)
    # 对齐 50Hz×约 0.8s + 跟随 200Hz×0.2s，远小于"每来一帧就发一组"的量。
    assert joint_frames < 150
    assert joint_frames > 10


def test_out_of_range_targets_are_refused_not_clamped():
    beyond = {joint: 500.0 for joint in range(1, 7)}
    with pytest.raises(SafetyError, match='限位'):
        align_module.encode_joint_targets(beyond)

    inside = {joint: 0.0 for joint in range(1, 7)}
    assert len(align_module.encode_joint_targets(inside)) == 3


def test_guarded_bus_refuses_to_transmit_in_dry_run():
    class Bus:
        def __init__(self):
            self.sent = []

        def send(self, message):
            self.sent.append(message)

        def recv(self, timeout=None):
            return None

        def shutdown(self):
            pass

    bus = Bus()
    guarded = GuardedBus(bus, allow_send=False)
    with pytest.raises(SafetyError, match='干跑模式'):
        guarded.send(_frame(0x151, bytes(8)))
    assert bus.sent == []

    allowed = GuardedBus(bus, allow_send=True)
    allowed.send(_frame(0x151, bytes(8)))
    assert [message.arbitration_id for message in bus.sent] == [0x151]
    assert allowed.sent_ids == {0x151: 1}


def test_sustained_tracking_error_latches_a_fault(harness):
    session, bus, _ = harness.build(config=make_config(duration_s=5.0))
    bus.hold_follower = True
    # 对齐（探针 + 轨迹）结束后再让主臂跳开，这样触发的是跟随期的门禁。
    bus.stream_inject = [(260, lambda: bus.master.update(
        _reachable_pose(40.0)))]
    session.operator.stop_after_polls = 100000

    code = session.run()

    assert code == EXIT_FAILED
    assert session.state is TeleopState.FAULT_LATCHED
    assert '跟踪误差' in (session.fault or '')
    assert bus.sent[-1].data == _enable_frame(False)


def test_follow_chain_holds_still_and_scales_the_gripper():
    config = make_config()
    chain = FollowChain(config, gripper_on=True)
    pose = _reachable_pose(10.0)
    chain.reset(pose, pose, command_gripper_m=0.0, master_gripper_m=0.02)

    for _ in range(400):
        targets, gripper = chain.update(pose, 0.02, STEP_S)

    assert targets == pytest.approx(pose)
    assert gripper == pytest.approx(0.02 * config.gripper_scale, abs=1e-4)


def test_follow_chain_tracks_a_moving_master():
    config = make_config(max_step_deg=0.0)
    chain = FollowChain(config, gripper_on=False)
    pose = {joint: 0.0 for joint in range(1, 7)}
    chain.reset(pose, pose)

    for _ in range(400):
        moved = _reachable_pose(20.0)
        targets, _ = chain.update(moved, None, STEP_S)

    assert targets[1] == pytest.approx(20.0, abs=0.5)


def test_follow_chain_refuses_grossly_out_of_range_master_feedback():
    config = make_config()
    chain = FollowChain(config, gripper_on=False)
    pose = {joint: 0.0 for joint in range(1, 7)}
    chain.reset(pose, pose)
    beyond = {joint: 500.0 for joint in range(1, 7)}

    with pytest.raises(SafetyError, match='主臂目标超限'):
        chain.update(beyond, None, STEP_S)


def test_console_operator_treats_missing_input_as_a_refusal(monkeypatch):
    import builtins

    def no_input(_prompt):
        raise EOFError

    monkeypatch.setattr(builtins, 'input', no_input)
    operator = ConsoleOperator()
    with pytest.raises(SafetyError):
        operator.require_start(1.0)
    assert operator.wait_teach_released(1.0) is False


def test_gripper_frames_carry_metres_and_effort():
    payload = align_module.encode_gripper_target(0.042, 1.5)

    assert int.from_bytes(payload[0:4], 'big', signed=True) == 42000
    assert int.from_bytes(payload[4:6], 'big') == 1500
    assert payload[6] == 0x01
    assert len(payload) == 8


def test_status_snapshot_reports_state_and_rates(harness):
    session, bus, _ = harness.build(config=make_config(duration_s=0.1))

    code = session.run()
    status = session.status()

    assert code == EXIT_OK
    assert status.side == 'left'
    assert status.state is TeleopState.IDLE_DISABLED
    assert status.label
    assert status.error_frames == 0


def test_angle_helpers_match_the_protocol():
    payload = _angles(1.5, -2.5)
    assert int.from_bytes(payload[0:4], 'big', signed=True) == 1500
    assert int.from_bytes(payload[4:8], 'big', signed=True) == -2500
    assert math.isclose(0.0, 0.0)


def test_error_burst_uses_full_five_second_budget():
    monitor = align_module.RuntimeMonitor(stale_s=0.03, max_skew_s=0.01)
    for _ in range(47):
        monitor.process(_error_frame(0x0C), now=10.0)
    assert monitor.error_rate_hz(10.0) == pytest.approx(9.4)
    for _ in range(53):
        monitor.process(_error_frame(0x0C), now=10.1)
    with pytest.raises(SafetyError, match='持续速率'):
        monitor.process(_error_frame(0x0C), now=10.1)
    assert monitor.error_rate_hz(15.2) == 0


def test_error_active_recovery_is_not_busoff():
    monitor = align_module.RuntimeMonitor(stale_s=0.03, max_skew_s=0.01)
    monitor.process(_error_frame(0x04, bytes((0, 0x40)) + bytes(6)), now=1.0)
    with pytest.raises(SafetyError, match='状态退化'):
        monitor.process(_error_frame(0x40), now=2.0)


def test_preflight_never_tolerates_busoff(harness):
    session, bus, _ = harness.build(config=make_config(send=False))
    bus.inject_after_cycle = [(2, _error_frame(0x40))]
    assert session.run() == EXIT_REFUSED
    assert bus.sent == []


def test_align_only_stops_before_active(harness):
    session, bus, _ = harness.build(config=make_config(align_only=True))
    session._run_follow = lambda: pytest.fail('unexpected follow')
    bus.master[6] = 4.0
    assert session.run() == EXIT_OK
    assert bus.follower[6] == pytest.approx(4.0, abs=0.1)
    assert bus.sent[-1].data == _enable_frame(False)


def test_alignment_fault_sends_standby_before_release_confirmation(harness):
    session, bus, _ = harness.build(operator=ScriptedOperator(release=False))
    bus.master_silent_after_stream_cycles = 5
    assert session.run() == EXIT_FAILED
    assert session.state is TeleopState.FAULT_LATCHED
    assert bus.sent[-1].arbitration_id == 0x151
    assert bus.sent[-1].data[0] == 0
    assert [m.data for m in bus.sent if m.arbitration_id == 0x471] == [
        _enable_frame(True)]
    assert session.bus is None


def test_inflight_disabled_samples_do_not_poison_enable_window(harness):
    session, bus, _ = harness.build(config=make_config(align_only=True))
    original_send = bus.send

    def delayed_enable(message):
        original_send(message)
        if message.arbitration_id == 0x471 and message.data[1] == 2:
            bus._pending = [_frame(0x261, bytes(8))]
    bus.send = delayed_enable
    assert session.run() == EXIT_OK


def test_follow_sends_each_complete_snapshot_once(harness):
    session, bus, _ = harness.build()
    session._open_bus()
    bus.enabled = True
    session._drain_until(harness.clock.now + 0.1)
    session._chain = FollowChain(session.config, gripper_on=False)
    session._chain.reset(bus.follower, bus.master)
    now = harness.clock.now
    assert session._emit_target(now, None) is True
    count = len(bus.sent)
    assert session._emit_target(now, now) is False
    session._monitor.master.update(0x2C5, _angles(0, 0), now)
    assert session._emit_target(now, now) is False
    assert len(bus.sent) == count


def test_return_uses_recorded_follower_pose_not_current_master(harness):
    _, bus, _ = harness.build(enabled=True)
    start = _reachable_pose(20.0)
    saved = _reachable_pose(2.0)
    bus.follower = dict(start)
    bus.master = _reachable_pose(40.0)
    plan = align_module.execute_alignment(
        'can_test', max_delta_deg=180.0, max_goal_clamp_deg=0.0,
        min_duration_s=0.2, max_peak_deg_s=1000.0,
        rate_hz=50.0, speed_percent=5, stale_s=0.03, max_skew_s=0.01,
        master_drift_deg=0.5, max_follow_error_deg=30.0,
        settle_s=0.1, goal_tolerance_deg=0.5, target_deg=saved)
    assert plan.goal_deg == saved
    assert bus.follower == pytest.approx(saved, abs=0.001)
    assert set(m.arbitration_id for m in bus.sent) == {
        0x151, 0x155, 0x156, 0x157}
    assert bus.sent[-1].data[0] == 0


@pytest.mark.parametrize('field,value', [
    ('command_rate_hz', 0), ('align_rate_hz', -1),
    ('max_error_rate_hz', float('nan')), ('speed_percent', 101),
    ('duration_s', 0), ('stale_s', float('inf')),
])
def test_invalid_control_parameters_refused_before_opening_bus(field, value):
    with pytest.raises(ValueError):
        TeleopSession(make_config(**{field: value}), ScriptedOperator())


def test_unexpected_follow_failure_still_stops_target_stream(harness):
    session, bus, _ = harness.build(operator=ScriptedOperator(release=False))

    def fail():
        raise RuntimeError('unexpected follow failure')

    session._run_follow = fail
    assert session.run() == EXIT_FAILED
    assert session.state is TeleopState.FAULT_LATCHED
    assert bus.sent[-1].arbitration_id == 0x151
    assert bus.sent[-1].data[0] == 0
    assert session.bus is None
