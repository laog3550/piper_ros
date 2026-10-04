"""Tests for the shared-CAN master/follower alignment tool."""

from types import SimpleNamespace

import pytest

import piper.piper_two_can_align as align_module
from piper.piper_two_can_align import (
    DEFAULT_MAX_DELTA_DEG,
    DEFAULT_MAX_GOAL_CLAMP_DEG,
    MINIMUM_JERK_PEAK_FACTOR,
    PoseTracker,
    RuntimeMonitor,
    SafetyError,
    encode_joint_targets,
    encode_motion_ctrl_2,
    execute_alignment,
    plan_alignment,
    send_target,
)


def _pose(value=0.0):
    return {joint: float(value) for joint in range(1, 7)}


def _angle_payload(first, second):
    return b''.join(
        int(round(value * 1000)).to_bytes(4, 'big', signed=True)
        for value in (first, second)
    )


def test_plan_uses_peak_speed_to_extend_minimum_jerk_duration():
    start = _pose()
    goal = _pose()
    goal[6] = 8.0

    plan = plan_alignment(
        start, goal, min_duration_s=2.0, max_peak_deg_s=5.0)

    assert plan.max_delta_deg == 8.0
    assert plan.duration_s == pytest.approx(
        MINIMUM_JERK_PEAK_FACTOR * 8.0 / 5.0)
    assert plan.peak_deg_s == pytest.approx(5.0)


def test_plan_refuses_a_pose_beyond_the_default_gate():
    goal = _pose()
    goal[6] = DEFAULT_MAX_DELTA_DEG + 0.001

    with pytest.raises(SafetyError, match='超过门禁'):
        plan_alignment(_pose(), goal)


def test_plan_clamps_a_small_master_target_overshoot():
    goal = _pose()
    goal[2] = -2.2
    goal[3] = 2.4

    plan = plan_alignment(_pose(), goal)

    assert plan.requested_goal_deg[2] == -2.2
    assert plan.requested_goal_deg[3] == 2.4
    assert plan.goal_deg[2] == -2.0
    assert plan.goal_deg[3] == 2.0
    assert plan.goal_corrections_deg == pytest.approx({2: -0.2, 3: 0.4})


def test_plan_refuses_a_large_master_target_clamp():
    goal = _pose()
    goal[5] = 70.0 + DEFAULT_MAX_GOAL_CLAMP_DEG + 0.001

    with pytest.raises(SafetyError, match='目标超限过多'):
        plan_alignment(goal_deg=goal, start_deg=_pose(), max_delta_deg=100.0)


def test_plan_refuses_a_follower_start_outside_command_limits():
    start = _pose()
    start[2] = -2.1

    with pytest.raises(SafetyError, match='从臂起点'):
        plan_alignment(start, _pose())


def test_motion_control_payloads_use_standard_follower_address_layout():
    assert encode_motion_ctrl_2(5) == bytes.fromhex('01 01 05 00 00 00 00 00')
    assert encode_motion_ctrl_2(5, standby=True) == bytes.fromhex(
        '00 01 05 00 00 00 00 00')
    with pytest.raises(ValueError):
        encode_motion_ctrl_2(0)


def test_joint_targets_are_signed_big_endian_millidegrees():
    targets = {1: -1.234, 2: 2.345, 3: -3.456,
               4: 4.567, 5: -5.678, 6: 6.789}

    frames = encode_joint_targets(targets)

    assert tuple(can_id for can_id, _ in frames) == (0x155, 0x156, 0x157)
    assert frames[0][1] == _angle_payload(-1.234, 2.345)
    assert frames[1][1] == _angle_payload(-3.456, 4.567)
    assert frames[2][1] == _angle_payload(-5.678, 6.789)


def test_send_target_uses_only_standard_follower_ids():
    class FakeBus:
        def __init__(self):
            self.sent = []

        def send(self, frame):
            self.sent.append(frame)

    bus = FakeBus()
    send_target(bus, _pose(), speed_percent=5)

    assert [frame.arbitration_id for frame in bus.sent] == [
        0x151, 0x155, 0x156, 0x157,
    ]
    assert not any(0x170 <= frame.arbitration_id <= 0x17F
                   for frame in bus.sent)


def test_execute_alignment_probes_stationary_target_before_motion(
        monkeypatch):
    class FakeClock:
        now = 0.0

        def monotonic(self):
            return self.now

    class FakeBus:
        def __init__(self):
            self.sent = []
            self.shutdown_called = False

        def send(self, frame):
            self.sent.append(frame)

        def shutdown(self):
            self.shutdown_called = True

    class FakeMonitor:
        def snapshots(self, now=None):
            return start, goal

    clock = FakeClock()
    bus = FakeBus()
    start = _pose()
    goal = _pose(1.0)

    monkeypatch.setattr(align_module.time, 'monotonic', clock.monotonic)
    monkeypatch.setattr(align_module, '_open_bus', lambda _port: bus)
    monkeypatch.setattr(
        align_module, 'RuntimeMonitor', lambda **_kwargs: FakeMonitor())
    monkeypatch.setattr(
        align_module, '_wait_runtime_ready',
        lambda _bus, _monitor: (start, goal),
    )

    def drain_until(_bus, _monitor, deadline):
        clock.now = deadline
        if len(bus.sent) >= 4:
            raise SafetyError('injected probe bus error')

    monkeypatch.setattr(align_module, '_drain_until', drain_until)

    with pytest.raises(SafetyError, match='injected probe bus error'):
        execute_alignment(
            'can_test',
            max_delta_deg=15.0,
            max_goal_clamp_deg=1.0,
            min_duration_s=2.0,
            max_peak_deg_s=5.0,
            rate_hz=50.0,
            speed_percent=5,
            stale_s=0.03,
            max_skew_s=0.01,
            master_drift_deg=0.5,
            max_follow_error_deg=3.0,
            settle_s=1.0,
            goal_tolerance_deg=0.5,
            stationary_probe_s=0.5,
        )

    assert [frame.arbitration_id for frame in bus.sent] == [
        0x151, 0x155, 0x156, 0x157, 0x151,
    ]
    assert bus.sent[0].data == encode_motion_ctrl_2(5)
    assert bus.sent[-1].data == encode_motion_ctrl_2(5, standby=True)
    assert tuple(frame.data for frame in bus.sent[1:4]) == tuple(
        payload for _, payload in encode_joint_targets(start)
    )
    assert bus.shutdown_called


def test_pose_tracker_decodes_the_offset_master_and_checks_skew():
    tracker = PoseTracker(offset=0x20)
    tracker.update(0x2C5, _angle_payload(1, 2), now=1.000)
    tracker.update(0x2C6, _angle_payload(3, 4), now=1.004)
    tracker.update(0x2C7, _angle_payload(5, 6), now=1.008)

    assert tracker.snapshot(1.009, stale_s=0.03, max_skew_s=0.01) == {
        1: 1.0, 2: 2.0, 3: 3.0, 4: 4.0, 5: 5.0, 6: 6.0,
    }
    with pytest.raises(SafetyError, match='不同步'):
        tracker.snapshot(1.009, stale_s=0.03, max_skew_s=0.005)


def test_pose_tracker_refuses_missing_and_stale_feedback():
    tracker = PoseTracker()
    tracker.update(0x2A5, _angle_payload(1, 2), now=1.0)
    with pytest.raises(SafetyError, match='不完整'):
        tracker.snapshot(1.01)

    tracker.update(0x2A6, _angle_payload(3, 4), now=1.0)
    tracker.update(0x2A7, _angle_payload(5, 6), now=1.0)
    with pytest.raises(SafetyError, match='过期'):
        tracker.snapshot(1.1, stale_s=0.03)


def test_preflight_enable_property_requires_every_sample_on():
    identity = SimpleNamespace()
    from piper.piper_two_can_align import PreflightReport

    samples = {joint: (True, True) for joint in range(1, 7)}
    report = PreflightReport(identity, _pose(), _pose(), samples)
    assert report.all_samples_enabled

    samples[4] = (True, False)
    report = PreflightReport(identity, _pose(), _pose(), samples)
    assert not report.all_samples_enabled


def test_runtime_monitor_rejects_external_control_and_disable():
    monitor = RuntimeMonitor(stale_s=0.03, max_skew_s=0.01)
    control = SimpleNamespace(
        is_error_frame=False, is_extended_id=False,
        arbitration_id=0x155, data=bytes(8),
    )
    with pytest.raises(SafetyError, match='外部控制帧'):
        monitor.process(control, now=1.0)

    disabled = SimpleNamespace(
        is_error_frame=False, is_extended_id=False,
        arbitration_id=0x261,
        data=bytes.fromhex('00 E6 00 20 00 00 00 00'),
    )
    with pytest.raises(SafetyError, match='disabled'):
        monitor.process(disabled, now=1.0)


def test_drain_deadline_race_never_passes_negative_timeout(monkeypatch):
    times = iter((0.999, 1.001, 1.002))
    monkeypatch.setattr(align_module.time, 'monotonic', lambda: next(times))

    class Bus:
        def recv(self, timeout):
            assert timeout == 0.0
            return None

    align_module._drain_until(Bus(), None, 1.0)
