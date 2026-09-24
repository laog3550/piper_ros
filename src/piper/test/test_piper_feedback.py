"""Tests for raw feedback decoding and per-joint freshness tracking."""

import math

import pytest

import numpy as np

from piper.piper_feedback import (
    AlphaBetaFilter,
    DEFAULT_ALPHA_BETA_ALPHA,
    DEFAULT_ALPHA_BETA_BETA,
    DEFAULT_ALPHA_BETA_MAX_DT_S,
    DEFAULT_ALPHA_BETA_POLE,
    DEFAULT_DEADBAND_DEG,
    DEFAULT_DEADBAND_SPEED_DEG_S,
    DEFAULT_FILTER_TAU_S,
    DEFAULT_GRIPPER_DEADBAND_M,
    DEFAULT_GRIPPER_EFFORT_NM,
    DEFAULT_GRIPPER_SCALE,
    DEFAULT_SMOOTH_BANDWIDTH_RAD_S,
    DEFAULT_SMOOTH_MAX_ACCELERATION_DEG_S2,
    DEFAULT_SMOOTH_MAX_JERK_DEG_S3,
    DEFAULT_SMOOTH_MAX_VELOCITY_DEG_S,
    DoubleMotionResetDetector,
    FEEDBACK_FIRST_CAN_ID,
    GRIPPER_OPEN_MAX_M,
    GRIPPER_OPEN_MIN_M,
    JOINT_COMMAND_LIMITS_DEG,
    JOINT_COUNT,
    LIMIT_FEEDBACK_CAN_ID,
    LIMIT_QUERY_CAN_ID,
    LIMIT_SET_CAN_ID,
    FeedbackTracker,
    DeadbandGate,
    JointLimit,
    LowPassFilter,
    MotionSmoother,
    OneEuroFilter,
    decode,
    align_targets,
    clamp_gripper,
    clamp_targets,
    minimum_jerk_step,
    decode_joint_limit,
    encode_limit_query,
    encode_limit_set,
    joint_for_can_id,
    limit_step,
    out_of_limits,
    plan_return,
    return_duration,
    scale_gripper,
)

ALL_ON = (True,) * JOINT_COUNT
ALL_OFF = (False,) * JOINT_COUNT


def test_alpha_beta_tracks_a_constant_velocity_without_steady_lag():
    flt = AlphaBetaFilter(alpha=0.5775, beta=0.1225)
    flt.reset({1: 0.0})
    outputs = []
    for index in range(1, 201):
        outputs.append(flt.update({1: index * 0.2}, 0.02)[1])
    assert outputs[-1] == pytest.approx(40.0, abs=0.02)
    assert flt.velocities()[1] == pytest.approx(10.0, abs=0.05)


def test_alpha_beta_smooths_position_noise_and_keeps_velocity_bounded():
    flt = AlphaBetaFilter(alpha=0.5775, beta=0.1225)
    flt.reset({1: 5.0})
    measurements = [5.0 + 0.1 * (-1) ** index for index in range(100)]
    outputs = [flt.update({1: value}, 0.02)[1] for value in measurements]
    assert max(outputs[-20:]) - min(outputs[-20:]) < 0.2
    assert abs(flt.velocities()[1]) < 1.0


def test_alpha_beta_reanchors_after_a_long_sample_gap():
    flt = AlphaBetaFilter(alpha=0.5775, beta=0.1225, max_dt=0.1)
    flt.reset({1: 0.0})
    flt.update({1: 1.0}, 0.02)
    assert flt.velocities()[1] > 0.0
    assert flt.update({1: 4.0}, 0.2)[1] == pytest.approx(4.0)
    assert flt.velocities()[1] == pytest.approx(0.0)


def test_alpha_beta_rejects_invalid_parameters():
    for kwargs in ({'alpha': 0.0}, {'alpha': 1.1}, {'beta': -0.1},
                   {'beta': 1.1}, {'max_dt': 0.0}):
        with pytest.raises(ValueError):
            AlphaBetaFilter(**kwargs)


def test_gripper_opening_is_clamped_to_the_commandable_range():
    """夹爪的可指令范围是 0~80mm：越界的 master 读数不会被原样转发。"""
    assert clamp_gripper(-0.01) == GRIPPER_OPEN_MIN_M
    assert clamp_gripper(0.5) == GRIPPER_OPEN_MAX_M
    assert clamp_gripper(0.032) == pytest.approx(0.032)
    # 死区与夹持力的默认值都要落在节点会接受/钳制的范围内
    assert DEFAULT_GRIPPER_DEADBAND_M > 0.0
    assert 0.5 <= DEFAULT_GRIPPER_EFFORT_NM <= 3.0


def test_gripper_scale_converts_master_travel_to_follower_travel():
    """行程倍数先乘后钳：主臂走满行程正好对应从臂走满行程。"""
    # 本机构：主臂全程 80/1.3 ≈ 61.5mm 映射到从臂 80mm
    assert scale_gripper(GRIPPER_OPEN_MAX_M / DEFAULT_GRIPPER_SCALE) == \
        pytest.approx(GRIPPER_OPEN_MAX_M)
    # 超出主臂行程的部分被钳在从臂上限，不会溢出
    assert scale_gripper(0.08) == GRIPPER_OPEN_MAX_M
    assert scale_gripper(-0.02) == GRIPPER_OPEN_MIN_M
    # 两台夹爪相同时用 1.0，就是纯镜像
    assert scale_gripper(0.035, 1.0) == pytest.approx(0.035)


def test_double_motion_reset_requires_two_completed_bursts():
    """Trigger reset only after two short bursts inside two seconds."""
    detector = DoubleMotionResetDetector(
        window_s=2.0, trigger_speed_deg_s=10.0,
        release_speed_deg_s=2.0, min_travel_deg=3.0)

    def pose(value):
        return {joint: float(value)
                for joint in range(1, JOINT_COUNT + 1)}

    assert detector.update(pose(0.0), 0.0) is False
    assert detector.update(pose(4.0), 0.4) is False
    assert detector.update(pose(8.0), 0.5) is False
    assert detector.update(pose(8.0), 0.6) is False
    assert detector.update(pose(12.0), 0.8) is False
    assert detector.update(pose(16.0), 0.9) is False
    assert detector.update(pose(16.0), 1.0) is True


def test_double_motion_reset_state_is_independent_per_detector():
    """Keep the quick-reset counters independent between detectors."""
    left = DoubleMotionResetDetector(
        trigger_speed_deg_s=10.0, release_speed_deg_s=2.0,
        min_travel_deg=3.0)
    right = DoubleMotionResetDetector(
        trigger_speed_deg_s=10.0, release_speed_deg_s=2.0,
        min_travel_deg=3.0)

    def pose(value):
        return {joint: float(value)
                for joint in range(1, JOINT_COUNT + 1)}

    for detector, offset in ((left, 0.0), (right, 10.0)):
        detector.update(pose(offset), 0.0)
        detector.update(pose(offset + 4.0), 0.4)
        detector.update(pose(offset + 8.0), 0.5)
        detector.update(pose(offset + 8.0), 0.6)
    assert left.update(pose(12.0), 0.8) is False
    assert right.update(pose(22.0), 0.8) is False


def test_alpha_beta_defaults_sit_on_a_repeated_error_pole():
    """默认增益由极点推导，不是手调的：α=1-λ²、β=(1-λ)²。"""
    pole = DEFAULT_ALPHA_BETA_POLE
    assert DEFAULT_ALPHA_BETA_ALPHA == pytest.approx(1.0 - pole ** 2)
    assert DEFAULT_ALPHA_BETA_BETA == pytest.approx((1.0 - pole) ** 2)
    # 第二轮真机反馈以稳定为先：从快速暂态的 0.40 调回低抖动的 0.65。
    assert pole == pytest.approx(0.65)


def _follow_pipeline(master, alpha, beta, seconds=4.0):
    """与 piper_teleop 同一条跟随链：死区 -> α-β -> 平滑级（速度前馈）。"""
    dt = 1.0 / 50.0
    gate = DeadbandGate(DEFAULT_DEADBAND_DEG, DEFAULT_DEADBAND_SPEED_DEG_S)
    flt = AlphaBetaFilter(alpha, beta, DEFAULT_ALPHA_BETA_MAX_DT_S)
    smooth = MotionSmoother(DEFAULT_SMOOTH_BANDWIDTH_RAD_S,
                            DEFAULT_SMOOTH_MAX_VELOCITY_DEG_S,
                            DEFAULT_SMOOTH_MAX_ACCELERATION_DEG_S2,
                            DEFAULT_SMOOTH_MAX_JERK_DEG_S3)
    for stage in (gate, flt, smooth):
        stage.reset({1: master(0.0)})
    outputs, references = [], []
    for index in range(int(seconds / dt)):
        value = master(index * dt)
        sample = gate.update({1: value}, dt)
        sample = flt.update(sample, dt)
        sample = smooth.update(sample, flt.velocities(), dt)
        outputs.append(sample[1])
        references.append(value)
    return outputs, references, dt


def test_the_default_pole_keeps_transients_bounded_and_steady_lag_low():
    """Stable λ costs some transient response but not steady velocity lag."""
    def speed_change(t):
        return 20.0 * min(t, 1.0) + 100.0 * max(0.0, t - 1.0)

    def steady(t):
        return 100.0 * t

    fast = (0.84, 0.36)
    stable = (DEFAULT_ALPHA_BETA_ALPHA, DEFAULT_ALPHA_BETA_BETA)
    peaks = {}
    for name, (alpha, beta) in (('fast', fast), ('stable', stable)):
        out, ref, dt = _follow_pipeline(speed_change, alpha, beta)
        start = int(1.0 / dt)
        peaks[name] = max(abs(ref[i] - out[i])
                          for i in range(start, start + 25))
    # 稳定配置允许暂态偏差增大，但不得失控。
    assert peaks['stable'] < 1.7 * peaks['fast'], peaks

    lags = {}
    for name, (alpha, beta) in (('fast', fast), ('stable', stable)):
        out, ref, dt = _follow_pipeline(steady, alpha, beta, seconds=6.0)
        start = int(3.0 / dt)
        lags[name] = (sum(ref[i] - out[i] for i in range(start, len(out)))
                      / (len(out) - start))
    # 匀速段的滞后只取决于「有没有速度前馈」，与 α、β 无关：两套参数必须一致，
    # 而且都要远小于不带前馈时的 100/7.5 ≈ 13 度。
    assert lags['fast'] == pytest.approx(lags['stable'], abs=0.05)
    assert abs(lags['stable']) < 3.0, lags


def test_the_stable_default_suppresses_residual_jitter():
    """Keep 10 Hz residual jitter below the arm's mechanical floor."""
    def wobble(t):
        return 5.0 + 0.3 * math.sin(2.0 * math.pi * 10.0 * t)

    out, _, dt = _follow_pipeline(wobble, DEFAULT_ALPHA_BETA_ALPHA,
                                  DEFAULT_ALPHA_BETA_BETA, seconds=5.0)
    tail = out[int(2.0 / dt):]
    spread = max(tail) - min(tail)
    # λ=0.65 与 10 rad/s 平滑级组合后应明显低于机械本底上沿 0.041°。
    assert spread < 0.02, spread


def _frame(enabled, joint=1, voltage=240, foc_temp=30):
    """Build a low-speed feedback frame for one joint."""
    status = 0x40 if enabled else 0x00
    return (
        FEEDBACK_FIRST_CAN_ID + joint - 1,
        bytes([voltage >> 8, voltage & 0xFF,
               (foc_temp >> 8) & 0xFF, foc_temp & 0xFF,
               25, status, 0x00, 0x00]),
    )


def _tracker_with(observations, timeout=0.5, now=0.0):
    """Feed one synthetic frame per joint, at time ``now``."""
    tracker = FeedbackTracker(timeout=timeout)
    for index, enabled in enumerate(observations, start=1):
        if enabled is None:
            continue
        can_id, data = _frame(bool(enabled), joint=index)
        tracker.update(can_id, data, now=now)
    return tracker


def test_enable_bit_is_byte_five_bit_six():
    # The bit position is the whole contract with the driver; a shift here
    # would silently invert the safety conclusion.
    for enabled in (True, False):
        can_id, data = _frame(enabled)
        assert bool(data[5] & 0x40) is enabled
        assert decode(can_id, data).enabled is enabled


def test_other_status_bits_do_not_leak_into_enable():
    # Driver error (bit 5), stall protection (bit 7) and over-current (bit 2)
    # must not be mistaken for the enable bit.
    for flags in (0x00, 0x04, 0x20, 0x80, 0xA4, 0xBF):
        _, data = _frame(False)
        data = bytes(data[:5] + bytes([flags]) + data[6:])
        assert decode(FEEDBACK_FIRST_CAN_ID, data).enabled is False


def test_decode_reports_voltage_and_temperature():
    can_id, data = _frame(True, voltage=235, foc_temp=41)
    feedback = decode(can_id, data)
    assert feedback.voltage == pytest.approx(23.5)
    assert feedback.foc_temperature == 41


def test_negative_temperature_decodes():
    can_id, data = _frame(False, foc_temp=-5)
    assert decode(can_id, data).foc_temperature == -5


@pytest.mark.parametrize('joint', range(1, JOINT_COUNT + 1))
def test_each_joint_maps_to_its_own_can_id(joint):
    can_id, data = _frame(True, joint=joint)
    assert joint_for_can_id(can_id) == joint
    assert decode(can_id, data).joint == joint


def test_unrelated_and_short_frames_are_ignored():
    assert joint_for_can_id(0x2A1) is None
    assert decode(0x2A1, bytes(8)) is None
    can_id, data = _frame(True)
    assert decode(can_id, data[:5]) is None


def test_all_six_enabled_and_all_six_disabled():
    assert _tracker_with(ALL_ON).observations(now=0.0) == ALL_ON
    assert _tracker_with(ALL_OFF).observations(now=0.0) == ALL_OFF


def test_missing_joints_are_unknown():
    tracker = _tracker_with((True, True, None, True, True, True))
    assert tracker.observations(now=0.0) == (True, True, None, True, True, True)
    assert tracker.missing_joints() == (3,)


def test_joint_that_stops_reporting_becomes_unknown():
    tracker = _tracker_with(ALL_ON, timeout=0.5, now=0.0)
    assert tracker.observations(now=0.4) == ALL_ON
    # Past the timeout the last known bit is stale and must stop counting.
    assert tracker.observations(now=0.6) == (None,) * JOINT_COUNT
    assert tracker.stale_joints(now=0.6) == tuple(range(1, JOINT_COUNT + 1))


def test_one_stale_joint_prevents_an_enabled_verdict():
    # Five joints still report enabled, the sixth went quiet: the arm must
    # not be reported as enabled.
    tracker = _tracker_with(ALL_ON, timeout=0.5, now=0.0)
    observations = tracker.observations(now=0.0)
    assert observations == ALL_ON

    can_id, data = _frame(True, joint=6)
    tracker.update(can_id, data, now=0.9)
    mixed = tracker.observations(now=1.0)
    assert mixed[5] is True
    assert mixed[:5] == (None,) * 5
    assert tracker.stale_joints(now=1.0) == (1, 2, 3, 4, 5)


def test_refreshing_a_stale_joint_restores_it():
    tracker = _tracker_with((True,) * JOINT_COUNT, timeout=0.5, now=0.0)
    assert tracker.observations(now=1.0) == (None,) * JOINT_COUNT
    for joint in range(1, JOINT_COUNT + 1):
        can_id, data = _frame(True, joint=joint)
        tracker.update(can_id, data, now=1.1)
    assert tracker.observations(now=1.1) == ALL_ON


def test_left_and_right_trackers_do_not_share_state():
    left = _tracker_with(ALL_ON, now=0.0)
    right = _tracker_with(ALL_OFF, now=0.0)

    assert left.observations(now=0.0) == ALL_ON
    assert right.observations(now=0.0) == ALL_OFF
    # Refreshing one arm must not make the other look fresh.
    for joint in range(1, JOINT_COUNT + 1):
        can_id, data = _frame(False, joint=joint)
        right.update(can_id, data, now=0.9)
    assert left.observations(now=0.9) == (None,) * JOINT_COUNT
    assert right.observations(now=0.9) == ALL_OFF


def test_timeout_must_be_positive():
    with pytest.raises(ValueError, match='timeout must be positive'):
        FeedbackTracker(timeout=0.0)


def test_documented_limits_cover_every_joint():
    assert sorted(JOINT_COMMAND_LIMITS_DEG) == list(range(1, JOINT_COUNT + 1))
    for joint, (low, high) in JOINT_COMMAND_LIMITS_DEG.items():
        assert low < high, f'joint{joint} 的范围不是递增的'


def test_angles_inside_the_range_are_accepted():
    inside = {1: 0.0, 2: 90.0, 3: -90.0, 4: 0.0, 5: 0.0, 6: 0.0}
    assert out_of_limits(inside) == {}


def test_angles_outside_the_range_report_their_overshoot():
    outside = {1: 0.0, 2: -2.381, 3: 2.150, 4: 0.0, 5: 0.0, 6: 0.0}
    overshoot = out_of_limits(outside)
    assert overshoot[2] == pytest.approx(0.381)
    assert overshoot[3] == pytest.approx(0.150)
    assert set(overshoot) == {2, 3}


def test_the_pose_from_the_first_motion_test_is_detected():
    # 首次运动测试的姿态：joint2 曾在此被钳制到边界（一次真实的非预期运动）。
    # 放宽后 joint3 已不再越界，但 joint2 仍略微越界，因为它停在机械限位外侧。
    pose_deg = {1: -6.729, 2: -2.381, 3: 1.570, 4: 9.710, 5: 23.949,
                6: -5.143}
    overshoot = out_of_limits(pose_deg)
    assert set(overshoot) == {2}
    assert overshoot[2] == pytest.approx(0.381)


def test_joint3_widening_shrank_the_master_residual():
    # 放宽 joint3 上限的直接目的。放宽前两台 master 的 j3 静置姿态分别越界
    # 0.848 与 0.902 度，follower 完全无法匹配 master；放宽到 2.0 度后残差降到
    # 0.05 与 0.10 度。残差不会归零：master 顶在机械限位上，而软件限位必须略低
    # 于机械限位，否则指令会指向不可达位置。
    for j3_angle, expected in ((2.049, 0.049), (2.102, 0.102)):
        pose = {joint: 0.0 for joint in range(1, JOINT_COUNT + 1)}
        pose[3] = j3_angle
        assert out_of_limits(pose)[3] == pytest.approx(expected, abs=1e-3)


def test_the_widened_bound_still_stays_below_a_reached_angle():
    # 安全依据：2.102 度是关节实际到达过的角度，故必然机械可达；软件限位只要
    # 不高于它，就不可能指向不可达位置。
    assert JOINT_COMMAND_LIMITS_DEG[3][1] <= 2.102


def test_boundaries_themselves_are_accepted():
    exact = {joint: bounds[0] for joint, bounds in
             JOINT_COMMAND_LIMITS_DEG.items()}
    assert out_of_limits(exact) == {}
    exact = {joint: bounds[1] for joint, bounds in
             JOINT_COMMAND_LIMITS_DEG.items()}
    assert out_of_limits(exact) == {}


def test_unknown_joints_are_ignored():
    assert out_of_limits({7: 999.0, 99: -999.0}) == {}


def test_limits_match_what_the_driver_reports():
    # 2026-09-23 用 GetAllMotorAngleLimitMaxSpd() 从驱动器读回的值。joint6 与
    # JointCtrl 的文档不符（驱动器报 ±180°，文档写 ±120°），此处以驱动器为准。
    assert JOINT_COMMAND_LIMITS_DEG == {
        1: (-150.0, 150.0),
        2: (-2.0, 180.0),
        3: (-170.0, 2.0),
        4: (-100.0, 100.0),
        5: (-70.0, 70.0),
        6: (-180.0, 180.0),
    }


def _zeros(values):
    return {i + 1: v for i, v in enumerate(values)}


def test_minimum_jerk_step_has_smooth_endpoints():
    # 五次曲线的端点速度和加速度都为零，起停不产生加速度阶跃。
    assert minimum_jerk_step(0.0) == 0.0
    assert minimum_jerk_step(1.0) == 1.0
    assert minimum_jerk_step(0.5) == pytest.approx(0.5)
    # 端点附近的增量远小于中段，说明两端慢、中间快。
    head = minimum_jerk_step(0.05) - minimum_jerk_step(0.0)
    middle = minimum_jerk_step(0.55) - minimum_jerk_step(0.50)
    assert head < middle
    # 用离散二阶差分验证两个端点的加速度趋近于零。
    dt = 1e-3
    start_acc = (minimum_jerk_step(2 * dt)
                 - 2 * minimum_jerk_step(dt)
                 + minimum_jerk_step(0.0)) / dt ** 2
    end_acc = (minimum_jerk_step(1.0)
               - 2 * minimum_jerk_step(1.0 - dt)
               + minimum_jerk_step(1.0 - 2 * dt)) / dt ** 2
    assert abs(start_acc) < 0.1
    assert abs(end_acc) < 0.1


def test_minimum_jerk_step_clamps_outside_the_unit_interval():
    assert minimum_jerk_step(-1.0) == 0.0
    assert minimum_jerk_step(2.0) == 1.0


def _pose(value):
    return {i + 1: float(value) for i in range(JOINT_COUNT)}


def test_alignment_starts_at_the_follower_pose_and_ends_at_the_master_pose():
    follower = _pose(0.0)
    master = _pose(60.0)
    assert align_targets(follower, master, 0.0)[1] == pytest.approx(0.0)
    assert align_targets(follower, master, 1.0)[1] == pytest.approx(60.0)
    assert align_targets(follower, master, 0.5)[1] == pytest.approx(30.0)


def test_alignment_is_monotonic_and_never_overshoots():
    follower = _pose(0.0)
    master = _pose(60.0)
    previous = -1.0
    for step in range(21):
        value = align_targets(follower, master, step / 20.0)[1]
        assert value >= previous, '对齐过程中不应回退'
        assert 0.0 <= value <= 60.0, '对齐不应超出两端'
        previous = value


def test_alignment_preserves_each_joint_independently():
    follower = {i: 0.0 for i in range(1, JOINT_COUNT + 1)}
    master = {i: float(i * 10) for i in range(1, JOINT_COUNT + 1)}
    mid = align_targets(follower, master, 0.5)
    for joint in range(1, JOINT_COUNT + 1):
        assert mid[joint] == pytest.approx(joint * 10 * 0.5)


def test_targets_are_clamped_into_the_commandable_range():
    # 含越界目标的指令行为未定义，所以必须钳制而不是原样发出。
    over = _zeros([0.0, -90.0, 90.0, 0.0, 0.0, 0.0])
    clamped, limited = clamp_targets(over)
    assert clamped[2] == pytest.approx(-2.0)
    assert clamped[3] == pytest.approx(2.0)
    assert set(limited) == {2, 3}
    assert limited[2] == pytest.approx(-88.0)


def test_in_range_targets_are_left_alone():
    inside = _zeros([0.0, -1.0, 1.0, 0.0, 0.0, 0.0])
    clamped, limited = clamp_targets(inside)
    assert clamped == inside
    assert limited == {}


def test_step_limiting_caps_a_sudden_jump():
    previous = _zeros([0.0] * JOINT_COUNT)
    jumped = _zeros([30.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    stepped, limited = limit_step(jumped, previous, 2.0)
    assert stepped[1] == pytest.approx(2.0)
    assert limited[1] == pytest.approx(28.0)


def test_step_limiting_allows_small_moves():
    previous = _zeros([0.0] * JOINT_COUNT)
    small = _zeros([1.0, -1.0, 0.0, 0.0, 0.0, 0.0])
    stepped, limited = limit_step(small, previous, 2.0)
    assert stepped == small
    assert limited == {}


def test_step_limiting_handles_the_first_target():
    targets = _zeros([5.0] * JOINT_COUNT)
    stepped, limited = limit_step(targets, None, 2.0)
    assert stepped == targets
    assert limited == {}


def test_limit_query_frame_asks_one_joint_for_angle_and_speed():
    # 0x472 的载荷是 [关节号, 0x01, 0*6]；查询本身不发运动指令、不使能。
    assert encode_limit_query(1) == bytes([1, 0x01, 0, 0, 0, 0, 0, 0])
    assert encode_limit_query(6) == bytes([6, 0x01, 0, 0, 0, 0, 0, 0])
    assert len(encode_limit_query(4)) == 8


def test_limit_set_frame_writes_all_three_fields_explicitly():
    # 三个字段都必须显式给值：0x7FFF（"不修改"）只有 V1.5-2 之后的固件支持，
    # 旧固件会把它读成 32.767 rad/s。这里逐字节核对布局与字节序。
    frame = encode_limit_set(
        JointLimit(joint=2, max_angle_deg=180.0, min_angle_deg=-2.0,
                   max_joint_spd=300))
    assert frame == (bytes([2]) + (1800).to_bytes(2, 'big')
                     + (-20).to_bytes(2, 'big', signed=True)
                     + (300).to_bytes(2, 'big') + b'\x00')
    assert len(frame) == 8


def test_limit_set_frame_keeps_negative_angle_limits_signed():
    # joint1 的下限是 -150.0 度（-1500），必须按 int16 有符号编码。
    frame = encode_limit_set(
        JointLimit(joint=1, max_angle_deg=150.0, min_angle_deg=-150.0,
                   max_joint_spd=1500))
    assert int.from_bytes(frame[1:3], 'big', signed=True) == 1500
    assert int.from_bytes(frame[3:5], 'big', signed=True) == -1500
    assert int.from_bytes(frame[5:7], 'big') == 1500


def _limit_frame(joint, max_angle, min_angle, speed):
    """Build the 0x473 payload a driver sends in reply to a query."""
    return (bytes([joint])
            + int(max_angle * 10).to_bytes(2, 'big', signed=True)
            + int(min_angle * 10).to_bytes(2, 'big', signed=True)
            + int(speed).to_bytes(2, 'big') + b'\x00')


def test_limit_feedback_decodes_into_degrees_and_raw_speed():
    limit = decode_joint_limit(
        LIMIT_FEEDBACK_CAN_ID, _limit_frame(2, 180.0, -2.0, 300))
    assert limit == JointLimit(joint=2, max_angle_deg=180.0,
                               min_angle_deg=-2.0, max_joint_spd=300)
    negative = decode_joint_limit(
        LIMIT_FEEDBACK_CAN_ID, _limit_frame(1, 150.0, -150.0, 1500))
    assert negative.min_angle_deg == pytest.approx(-150.0)
    assert negative.max_joint_spd == 1500


def test_limit_frames_round_trip_through_the_codec():
    # 写入用的字段必须能被读回逻辑原样解出，否则「写后读回验证」毫无意义。
    for joint, bounds in JOINT_COMMAND_LIMITS_DEG.items():
        written = JointLimit(joint=joint, max_angle_deg=bounds[1],
                             min_angle_deg=bounds[0], max_joint_spd=1500)
        again = decode_joint_limit(LIMIT_FEEDBACK_CAN_ID,
                                   encode_limit_set(written))
        assert again == written


def test_other_frames_and_short_payloads_are_not_limits():
    assert decode_joint_limit(LIMIT_QUERY_CAN_ID, bytes(8)) is None
    assert decode_joint_limit(LIMIT_SET_CAN_ID, bytes(8)) is None
    assert decode_joint_limit(LIMIT_FEEDBACK_CAN_ID, bytes(7)) is None
    # 关节号 0 与 7 都不是有效电机号，不能被当成限位反馈。
    assert decode_joint_limit(LIMIT_FEEDBACK_CAN_ID, bytes(8)) is None
    assert decode_joint_limit(LIMIT_FEEDBACK_CAN_ID,
                              bytes([7]) + bytes(7)) is None


def test_return_duration_follows_the_average_speed():
    # 把峰值放宽后只剩平均速度约束。
    assert return_duration(30.0, 10.0, 100.0) == pytest.approx(3.0)


def test_return_duration_is_capped_by_the_peak_ceiling():
    # 峰值上限更紧时，时长由 1.875*Δmax/peak 决定。
    assert return_duration(30.0, 10.0, 5.0) == pytest.approx(11.25)
    # 反过来平均速度更紧时，时长由 Δmax/speed 决定。
    assert return_duration(30.0, 1.0, 5.0) == pytest.approx(30.0)
    # 两个方向都不越界：任何一对参数下平均速度与峰值都不超过各自的上限。
    for speed, peak in ((10.0, 5.0), (1.0, 5.0), (2.0, 3.0), (10.0, 15.0)):
        duration = return_duration(30.0, speed, peak)
        assert 30.0 / duration <= speed + 1e-9
        assert 1.875 * 30.0 / duration <= peak + 1e-9


def test_return_duration_is_zero_when_nothing_moved():
    # 已经在 home 上就不该有回位动作，也不该除以零。
    assert return_duration(0.0, 10.0, 15.0) == 0.0


def test_return_duration_rejects_non_positive_limits():
    with pytest.raises(ValueError, match='must be positive'):
        return_duration(10.0, 0.0, 15.0)
    with pytest.raises(ValueError, match='must be positive'):
        return_duration(10.0, 10.0, 0.0)


def test_return_plan_reports_displacement_duration_and_peak():
    start = _pose(0.0)
    home = _pose(12.0)
    plan = plan_return(start, home, speed_deg_s=10.0, max_peak_deg_s=15.0)
    assert plan.max_delta_deg == pytest.approx(12.0)
    assert plan.duration == pytest.approx(1.5)
    # 五次曲线峰值是平均值的 1.875 倍；峰值上限决定 1.5 秒时长。
    assert plan.peak_deg_s == pytest.approx(15.0)
    assert plan.deltas_deg[1] == pytest.approx(12.0)


def test_return_plan_uses_each_joint_s_own_displacement():
    start = _pose(0.0)
    home = {1: 3.0, 2: -7.5, 3: 1.0, 4: 0.0, 5: 0.5, 6: -0.25}
    plan = plan_return(start, home)
    assert plan.max_delta_deg == pytest.approx(7.5)
    assert plan.deltas_deg == pytest.approx(home)
    # 时长由位移最大的关节决定，其余关节只是跟着这个时长走。
    assert plan.duration == pytest.approx(7.5 / 20.0)


def test_return_plan_flags_joints_the_path_would_clamp():
    # home 落在限位外时（失能下坠过的姿态就可能这样），回位终点会被钳制，
    # 计划必须在执行前把这件事说出来。
    home = _pose(0.0)
    home[3] = 2.15  # joint3 上限 2.0
    plan = plan_return(_pose(0.0), home)
    assert set(plan.clamped_deg) == {3}
    assert plan.clamped_deg[3] == pytest.approx(0.15, abs=1e-6)


def test_return_plan_of_an_in_range_pose_clamps_nothing():
    # 起点与终点都在范围内时，二者之间任意插值也在范围内（限位是区间），
    # 所以「不会被钳制」是必然结论，不是采样碰巧没碰到。
    start = {1: -3.0, 2: 0.0, 3: 0.0, 4: 2.0, 5: -1.0, 6: -150.0}
    home = {1: 9.0, 2: 1.0, 3: 0.5, 4: -2.0, 5: 1.0, 6: 60.0}
    plan = plan_return(start, home)
    assert plan.clamped_deg == {}
    for step in range(21):
        path = align_targets(start, home, step / 20.0)
        assert out_of_limits(path) == {}


def test_return_plan_rejects_a_useless_sample_count():
    with pytest.raises(ValueError, match='samples'):
        plan_return(_pose(0.0), _pose(1.0), samples=0)


def test_filter_is_seeded_by_reset_so_the_first_output_has_no_jump():
    # 进入跟随的那一刻用当前 master 值初始化，否则滤波器会从 0 开始爬升，
    # 那就是一个真实的起始跳变。
    flt = LowPassFilter(tau=0.02)
    flt.reset(_pose(33.0))
    assert flt.update(_pose(33.0), dt=0.02) == pytest.approx(_pose(33.0))


def test_filter_alpha_comes_from_the_measured_interval():
    # alpha = dt/(tau+dt)：dt=tau 时正好取中点。用实际测得的 dt，而不是设定周期。
    flt = LowPassFilter(tau=0.02)
    flt.reset({1: 0.0})
    assert flt.update({1: 10.0}, dt=0.02)[1] == pytest.approx(5.0)
    flt.reset({1: 0.0})
    assert flt.update({1: 10.0}, dt=0.06)[1] == pytest.approx(7.5)


def test_filter_first_step_of_a_step_input_is_dx_times_dt_over_tau():
    # 低通只平滑形状、不限制幅度：阶跃输入下的初始变化率是 Δx/τ（此处
    # Δx=10、τ=0.02 → 500 deg/s），所以速度上限只能来自固件参数。
    tau = 0.02
    dt = 0.001
    flt = LowPassFilter(tau=tau)
    flt.reset({1: 0.0})
    first = flt.update({1: 10.0}, dt=dt)[1]
    assert first == pytest.approx(10.0 * dt / (tau + dt))
    assert first / dt == pytest.approx(10.0 / tau, rel=0.05)


def test_filter_approaches_the_sample_without_overshooting():
    flt = LowPassFilter(tau=0.02)
    flt.reset({1: 0.0})
    previous = 0.0
    for _ in range(200):
        value = flt.update({1: 10.0}, dt=0.02)[1]
        assert previous <= value <= 10.0
        previous = value
    assert previous == pytest.approx(10.0, abs=1e-3)


def test_filter_holds_its_output_when_no_time_passed():
    flt = LowPassFilter(tau=0.02)
    flt.reset({1: 4.0})
    assert flt.update({1: 40.0}, dt=0.0)[1] == pytest.approx(4.0)
    assert flt.update({1: 40.0}, dt=-1.0)[1] == pytest.approx(4.0)


def test_filter_zero_tau_is_transparent():
    # tau=0 表示不滤波：这正是「关掉滤波」的实现方式。
    flt = LowPassFilter(tau=0.0)
    flt.reset({1: 0.0})
    assert flt.update({1: 123.0}, dt=0.0)[1] == pytest.approx(123.0)
    assert flt.update({1: -8.0}, dt=1.0)[1] == pytest.approx(-8.0)


def test_filter_seeds_a_joint_that_appears_late():
    flt = LowPassFilter(tau=0.02)
    flt.reset({1: 0.0})
    assert flt.update({1: 10.0, 2: 3.0}, dt=0.02) == pytest.approx(
        {1: 5.0, 2: 3.0})


def test_filter_keeps_joints_missing_from_a_sample():
    # 某一帧缺某个关节时，该关节保留上一次的输出，而不是被当成 0。
    flt = LowPassFilter(tau=0.02)
    flt.reset({1: 8.0, 2: -8.0})
    assert flt.update({1: 8.0}, dt=0.02) == pytest.approx({1: 8.0})
    # 下一帧 joint2 带着 0 出现：从 -8.0 平滑过去（取中点 -4.0），说明它
    # 上一帧的状态被完整保留，而不是被丢弃后重新用 0 做起点。
    assert flt.update({1: 8.0, 2: 0.0}, dt=0.02) == pytest.approx(
        {1: 8.0, 2: -4.0})


def test_deadband_holds_a_resting_hand_completely():
    # 用户要的效果：手"停着"的时候目标一点都不动。静止的手既慢（速度门限之内）
    # 又小（幅度门限之内），所以输出被保持住。
    gate = DeadbandGate(0.2, speed_threshold=0.8)
    gate.reset({1: 10.0})
    for offset in (0.05, -0.1, 0.15, -0.19, 0.0):
        assert gate.update({1: 10.0 + offset}, dt=0.5)[1] == pytest.approx(10.0)


def test_deadband_passes_a_move_that_crosses_the_threshold():
    gate = DeadbandGate(0.2, speed_threshold=0.8)
    gate.reset({1: 10.0})
    assert gate.update({1: 10.25}, dt=0.5)[1] == pytest.approx(10.25)


def test_deadband_passes_a_slow_drag_without_chopping_it_into_steps():
    # 用户的原始症状："每一小段速度都会归为零然后重新加速"。只按幅度放行的
    # 死区会把慢拖切成 0.2 度一级的台阶（台阶之间指令速度为零），加上速度
    # 门限之后，只要在动就连续跟随，速度不再归零。
    gate = DeadbandGate(0.2, speed_threshold=0.8)
    gate.reset({1: 0.0})
    step = 0.02                      # 每周期 0.02 度 = 1 deg/s
    inputs = [step * index for index in range(1, 120)]
    passed = [gate.update({1: value}, dt=0.02)[1] for value in inputs]
    # 速度估计需要几个周期建立（约 0.05 秒），之后每一次都原样通过：
    # 没有"攒够一个死区再跳一下"的台阶。
    assert passed[9:] == pytest.approx(inputs[9:])
    # 建立期间的保持量远小于阈值，且输出永不越过输入
    assert all(p <= s + 1e-9 for p, s in zip(passed, inputs))
    assert max(inputs[i] - passed[i] for i in range(9)) < 0.2


def test_deadband_still_holds_a_slow_imperceptible_wobble():
    # 反过来说：慢到 0.5 Hz、±0.15 度的手晃（速度约 0.47 deg/s）仍应被挡住
    gate = DeadbandGate(0.2, speed_threshold=0.8)
    gate.reset({1: 0.0})
    for index in range(1, 40):
        wobble = 0.15 * math.sin(2 * math.pi * 0.5 * index * 0.1)
        assert gate.update({1: wobble}, dt=0.1)[1] == pytest.approx(0.0)


def test_deadband_does_nothing_to_fast_motion():
    gate = DeadbandGate(0.2, speed_threshold=0.8)
    gate.reset({1: 0.0})
    values = [index * 0.5 for index in range(1, 20)]
    assert [gate.update({1: value}, dt=0.02)[1] for value in values] == \
        pytest.approx(values)


def test_deadband_zero_threshold_is_transparent():
    gate = DeadbandGate(0.0, speed_threshold=0.8)
    gate.reset({1: 0.0})
    assert gate.update({1: 0.03}, dt=0.5)[1] == pytest.approx(0.03)
    assert gate.update({1: 0.031}, dt=0.5)[1] == pytest.approx(0.031)


def test_deadband_zero_speed_threshold_is_amplitude_only():
    # speed_threshold=0 退回"只按幅度"的老行为：慢拖会被切成台阶。
    gate = DeadbandGate(0.2, speed_threshold=0.0)
    gate.reset({1: 0.0})
    passed = [gate.update({1: 0.01 * index}, dt=0.02)[1]
              for index in range(1, 60)]
    assert passed[18] == pytest.approx(0.0), '老行为：前 19 拍被挡住'
    assert passed[19] == pytest.approx(0.2), '老行为：第 20 拍一次跳到 0.2'


def test_deadband_tracks_each_joint_separately():
    gate = DeadbandGate(0.2, speed_threshold=0.8)
    gate.reset({1: 0.0, 2: 0.0})
    # j1 每周期 0.4 deg/s（低于门限）且幅度未超阈值 -> 保持；j2 2 deg/s -> 跟随
    assert gate.update({1: 0.1, 2: 0.5}, dt=0.25) == pytest.approx(
        {1: 0.0, 2: 0.5})


def test_deadband_rejects_negative_parameters():
    with pytest.raises(ValueError, match='threshold must not be negative'):
        DeadbandGate(-0.1)
    with pytest.raises(ValueError, match='speed threshold'):
        DeadbandGate(0.2, speed_threshold=-1.0)


def test_deadband_gates_a_wobble_around_a_new_position_symmetrically():
    # 拖出去 1 度后停住：之后手在这 1 度附近的小抖动（正反两个方向）都被挡住。
    gate = DeadbandGate(0.2, speed_threshold=0.8)
    gate.reset({1: 0.0})
    assert gate.update({1: 1.0}, dt=0.02)[1] == pytest.approx(1.0)
    # 快速移动之后速度估计还需一两个周期衰减，先静一下（真实操作也是先停住）
    for _ in range(3):
        gate.update({1: 1.0}, dt=0.5)
    for offset in (0.1, -0.15, 0.05, -0.19, 0.0):
        assert gate.update({1: 1.0 + offset}, dt=0.5)[1] == pytest.approx(1.0)


RATE = 50.0
PERIOD = 1.0 / RATE


def _smooth_run(smoother, positions, velocities=None):
    """Walk a reference through the smoother at 50 Hz."""
    smoother.reset({1: positions[0]})
    velocities = velocities or [0.0] * len(positions)
    out = [positions[0]]
    for goal, speed in zip(positions[1:], velocities[1:]):
        out.append(smoother.update({1: goal}, {1: speed}, PERIOD)[1])
    return out


def _band_rms(values, band=(5.0, 15.0), rate=RATE):
    """Zero-phase band-pass RMS, so edges cannot fake energy."""
    from scipy.signal import butter, sosfiltfilt
    signal = np.asarray(values, dtype=float)
    sos = butter(4, band, btype='bandpass', fs=rate, output='sos')
    return float(np.sqrt(np.mean(sosfiltfilt(sos, signal - signal.mean()) ** 2)))


def test_smoother_rejects_non_positive_limits():
    for name in ('bandwidth', 'max_velocity', 'max_acceleration', 'max_jerk'):
        kwargs = {name: 0.0}
        with pytest.raises(ValueError, match=name):
            MotionSmoother(**kwargs)


def test_smoother_respects_velocity_acceleration_and_jerk_limits():
    # 三个上限是硬约束：这是它相对"每周期重规划五次多项式"方案的关键优势
    # （那个方案会发散到 389 度滞后、或冲过目标 86 度）。
    positions = [0.0] + [90.0] * 250
    out = _smooth_run(MotionSmoother(), positions)
    speeds = [(b - a) / PERIOD for a, b in zip(out, out[1:])]
    accels = [(b - a) / PERIOD for a, b in zip(speeds, speeds[1:])]
    jerks = [(b - a) / PERIOD for a, b in zip(accels, accels[1:])]
    # jerk 限幅无法在一个周期内收掉已经建立的加速度，所以速度上限可能被超出
    # 千分之几（实测 0.35%）：这是安全网级别的偏差，不是发散。
    assert max(map(abs, speeds)) <= DEFAULT_SMOOTH_MAX_VELOCITY_DEG_S * 1.01
    assert max(map(abs, accels)) <= DEFAULT_SMOOTH_MAX_ACCELERATION_DEG_S2
    assert max(map(abs, jerks)) <= DEFAULT_SMOOTH_MAX_JERK_DEG_S3


def test_smoother_acceleration_is_continuous():
    # "限制加速度跳变"就是这一条：相邻周期的加速度变化不超过 jerk 上限×dt。
    positions = [0.0] + [60.0] * 200
    out = _smooth_run(MotionSmoother(), positions)
    speeds = [(b - a) / PERIOD for a, b in zip(out, out[1:])]
    accels = [(b - a) / PERIOD for a, b in zip(speeds, speeds[1:])]
    steps = [abs(b - a) for a, b in zip(accels, accels[1:])]
    assert max(steps) <= DEFAULT_SMOOTH_MAX_JERK_DEG_S3 * PERIOD + 1e-6


def test_smoother_settles_on_a_step_without_visible_overshoot():
    positions = [0.0] + [30.0] * 200
    out = _smooth_run(MotionSmoother(), positions)
    assert out[-1] == pytest.approx(30.0, abs=0.01)
    # 12 rad/s 配置下 30 度阶跃过冲约 2.85%。
    assert max(out) < 30.0 * 1.03, '阶跃过冲应小于 3%'


def test_smoother_tracks_a_steady_drag_with_velocity_feed_forward():
    # 前馈是"平滑不牺牲跟手"的关键：不加它时稳态滞后 = 速度/带宽增益。
    positions = [index * PERIOD * 100.0 for index in range(200)]
    out = _smooth_run(MotionSmoother(), positions, [100.0] * len(positions))
    tail = out[-100:]
    goals = positions[-100:]
    # 离散化带来的稳态偏差：100 deg/s 时约 2 度（无前馈时是 20 度量级）
    assert max(abs(o - g) for o, g in zip(tail, goals)) < 3.0


def test_smoother_lags_without_feed_forward():
    # 对照组：没有前馈时确实存在稳态滞后（说明上面那条不是自动成立的）。
    positions = [index * PERIOD * 100.0 for index in range(200)]
    out = _smooth_run(MotionSmoother(), positions)
    lag = positions[-1] - out[-1]
    assert lag > 3.0


def test_smoother_removes_the_tremor_band_that_one_euro_lets_through():
    # 用户报告的问题：大幅度移动时抖动严重。10 Hz、±0.3 度的手抖叠在 100 deg/s
    # 的拖动上——这一级把它衰减掉 95% 以上。
    positions = [index * PERIOD * 100.0
                 + 0.3 * math.sin(2 * math.pi * 10.0 * index * PERIOD)
                 for index in range(300)]
    velocities = [100.0] * len(positions)
    out = _smooth_run(MotionSmoother(), positions, velocities)
    tail = slice(100, None)
    before = _band_rms(positions[tail])
    after = _band_rms(out[tail])
    assert before > 0.15, '输入里应当先有可见的手抖'
    assert after < before * 0.1, '平滑级要把这段手抖压掉至少 90%'


def test_smoother_overshoot_on_an_abrupt_stop_stays_bounded():
    # 代价如实记录：参考突然停住时，机械臂会冲过它 v/带宽 的距离再回来。
    # 150 deg/s、带宽 15 rad/s 时约 7.6 度。
    positions = [index * PERIOD * 150.0 for index in range(200)] \
        + [200 * PERIOD * 150.0] * 200
    velocities = [150.0] * 200 + [0.0] * 200
    out = _smooth_run(MotionSmoother(), positions, velocities)
    overshoot = max(out) - max(positions)
    assert 4.0 < overshoot < 12.0


def test_smoother_holds_when_no_time_passed():
    smoother = MotionSmoother()
    smoother.reset({1: 5.0})
    assert smoother.update({1: 50.0}, {1: 0.0}, 0.0)[1] == pytest.approx(5.0)


def test_smoother_seeds_a_joint_that_appears_late():
    smoother = MotionSmoother()
    smoother.reset({1: 0.0})
    assert smoother.update({1: 0.0, 2: 7.0}, {}, PERIOD) == pytest.approx(
        {1: 0.0, 2: 7.0})


def test_one_euro_exposes_the_filtered_velocity_for_feed_forward():
    flt = OneEuroFilter()
    flt.reset({1: 0.0})
    assert flt.velocities()[1] == pytest.approx(0.0)
    flt.update({1: 1.0}, dt=PERIOD)
    # 原始微分是 1.0/0.02 = 50 deg/s，但速度估计自己也被 d_cutoff（1 Hz）低通，
    # 所以第一拍只留下约 11%（这正是它不会把手抖导数注回去的原因）。
    assert 5.0 < flt.velocities()[1] < 6.5


def test_filter_rejects_negative_tau():
    with pytest.raises(ValueError, match='tau must not be negative'):
        LowPassFilter(tau=-0.001)


# One Euro filters are driven at the teleop's 50 Hz unless a test says otherwise.
RATE = 50.0
PERIOD = 1.0 / RATE


def _tremor(speed_deg_s, seconds=3.0, amplitude=0.3, frequency=10.0):
    """Build a held arm's reading: a linear drag plus 10 Hz hand tremor."""
    return [
        speed_deg_s * index * PERIOD
        + amplitude * math.sin(2 * math.pi * frequency * index * PERIOD)
        for index in range(int(seconds * RATE))
    ]


def _wobble(signal, tail_seconds=1.0):
    """Peak-to-peak amplitude of the last seconds, halved into +-."""
    tail = signal[-int(tail_seconds * RATE):]
    return (max(tail) - min(tail)) / 2


def _lag(filtered, clean, tail_seconds=1.0):
    """How far the filtered output trails the true motion, in degrees."""
    count = int(tail_seconds * RATE)
    return sum(c - f for c, f in zip(clean[-count:], filtered[-count:])) / count


def _run_signals(smoother, clean):
    """Feed one clean signal through a filter, seeding it like the teleop."""
    smoother.reset({1: clean[0]})
    return [smoother.update({1: value}, PERIOD)[1] for value in clean]


def test_one_euro_rejects_bad_parameters():
    with pytest.raises(ValueError, match='min_cutoff must be positive'):
        OneEuroFilter(min_cutoff=0.0)
    with pytest.raises(ValueError, match='d_cutoff must be positive'):
        OneEuroFilter(d_cutoff=-1.0)
    with pytest.raises(ValueError, match='beta must not be negative'):
        OneEuroFilter(beta=-0.1)


def test_one_euro_leaves_a_steady_signal_alone():
    # 手臂不动、读数也没有抖动时，滤波器不应该自己产生运动。
    flt = OneEuroFilter()
    flt.reset({1: 12.5})
    for _ in range(50):
        assert flt.update({1: 12.5}, dt=PERIOD)[1] == pytest.approx(12.5)


def test_one_euro_suppresses_hand_tremor_while_held_still():
    # 用户报告的问题：--speed 100 之后 follower 忠实复现了手抖。手抖集中在
    # 8~12 Hz，One Euro 在静止时把截止频率压到 min_cutoff，正好压这一段。
    signal = _tremor(speed_deg_s=0.0)
    filtered = _run_signals(OneEuroFilter(), signal)
    # 10 Hz 在 50 Hz 采样下每周期 5 点，采样峰值本来就到不了正弦的 0.3。
    assert _wobble(signal) == pytest.approx(0.3, abs=0.02)
    assert _wobble(filtered) < 0.3 * 0.2, '静止时的残留抖动应低于输入的 20%'


def test_one_euro_barely_lags_at_slow_drag():
    # 慢拖时截止频率 ≈ min_cutoff + beta*v，滞后 v/(2*pi*fc) 必须落在 1 度内。
    clean = _tremor(speed_deg_s=20.0)
    filtered = _run_signals(OneEuroFilter(), clean)
    assert _lag(filtered, clean) < 1.0


def test_one_euro_barely_lags_at_fast_drag():
    clean = _tremor(speed_deg_s=100.0)
    filtered = _run_signals(OneEuroFilter(), clean)
    assert _lag(filtered, clean) < 0.6


def test_one_euro_opens_its_cutoff_with_speed():
    # 自适应性的定义：拖得越快，时间常数越小（滞后/speed 变小）。
    slow, fast = _tremor(speed_deg_s=20.0), _tremor(speed_deg_s=100.0)
    slow_lag = _lag(_run_signals(OneEuroFilter(), slow), slow) / 20.0
    fast_lag = _lag(_run_signals(OneEuroFilter(), fast), fast) / 100.0
    assert fast_lag < slow_lag / 2.0


def test_one_euro_beats_the_fixed_low_pass_on_both_axes():
    # 换方案的依据，两个方向都要更好（数据见 piper_feedback 里 beta 的注释）。
    low_pass = LowPassFilter(tau=DEFAULT_FILTER_TAU_S)
    held = _tremor(speed_deg_s=0.0)
    assert (_wobble(_run_signals(OneEuroFilter(), held))
            < _wobble(_run_signals(low_pass, held)))
    dragged = _tremor(speed_deg_s=100.0)
    assert (_lag(_run_signals(OneEuroFilter(), dragged), dragged)
            < _lag(_run_signals(low_pass, dragged), dragged))


def test_one_euro_holds_its_output_when_no_time_passed():
    flt = OneEuroFilter()
    flt.reset({1: 4.0})
    assert flt.update({1: 40.0}, dt=0.0)[1] == pytest.approx(4.0)
    assert flt.update({1: 40.0}, dt=-1.0)[1] == pytest.approx(4.0)


def test_one_euro_seeds_a_joint_that_appears_late():
    flt = OneEuroFilter()
    flt.reset({1: 0.0})
    assert flt.update({1: 0.0, 2: 7.0}, dt=PERIOD) == pytest.approx(
        {1: 0.0, 2: 7.0})


def test_one_euro_is_seeded_by_reset_so_there_is_no_startup_jump():
    flt = OneEuroFilter()
    flt.reset(_pose(33.0))
    assert flt.update(_pose(33.0), dt=PERIOD) == pytest.approx(_pose(33.0))


def test_one_euro_never_overshoots_a_ramp():
    # 任何超调都会变成一次真实的反向运动，滤波器不能制造它。
    clean = _tremor(speed_deg_s=30.0)
    filtered = _run_signals(OneEuroFilter(), clean)
    assert all(f <= c + 1e-9 for f, c in zip(filtered, clean))

