"""Tests for the limit read/write safeguards of the speed-limit tool."""

from piper.piper_feedback import JOINT_COUNT, JointLimit
from piper.piper_speed_limit import (
    _enabled_problem,
    _speed_text,
    _verify,
)


def _limits(speed=300, joint_count=JOINT_COUNT):
    """Build a full set of read-back limits for one arm."""
    return {
        joint: JointLimit(joint=joint, max_angle_deg=150.0,
                          min_angle_deg=-150.0, max_joint_spd=speed)
        for joint in range(1, joint_count + 1)
    }


def _disabled():
    return {joint: False for joint in range(1, JOINT_COUNT + 1)}


def test_all_disabled_is_the_only_state_that_allows_a_write():
    assert _enabled_problem(_disabled()) is None


def test_an_enabled_joint_blocks_the_write_and_is_named():
    enabled = _disabled()
    enabled[3] = True
    problem = _enabled_problem(enabled)
    assert problem is not None
    assert 'j3' in problem
    # 其余关节是失能的，提示里不应该把它们也报成使能。
    assert 'j1' not in problem


def test_a_missing_enable_bit_blocks_the_write():
    # 没收到某关节的低压反馈帧时，"失能"不是结论而是未知，未知不能放行。
    enabled = _disabled()
    del enabled[5]
    problem = _enabled_problem(enabled)
    assert problem is not None
    assert 'j5' in problem
    assert '无法确认' in problem


def test_verify_passes_when_only_the_speed_changed():
    before = _limits(speed=300)
    wanted = {joint: JointLimit(joint=joint, max_angle_deg=150.0,
                                min_angle_deg=-150.0, max_joint_spd=1500)
              for joint in range(1, JOINT_COUNT + 1)}
    assert _verify('can_fr', wanted, before, wanted) is True


def test_verify_rejects_a_speed_that_did_not_take():
    before = _limits(speed=300, joint_count=JOINT_COUNT)
    wanted = {joint: JointLimit(joint=joint, max_angle_deg=150.0,
                                min_angle_deg=-150.0, max_joint_spd=1500)
              for joint in range(1, JOINT_COUNT + 1)}
    after = dict(wanted)
    after[2] = JointLimit(joint=2, max_angle_deg=150.0, min_angle_deg=-150.0,
                          max_joint_spd=300)
    assert _verify('can_fr', wanted, before, after) is False


def test_verify_rejects_a_changed_angle_limit():
    # 未修改的字段必须原样保留：写入把角度限位改掉是最危险的失败方式，
    # 因为它会让越界指令钳到别的位置。
    before = _limits(speed=300)
    wanted = {joint: JointLimit(joint=joint, max_angle_deg=150.0,
                                min_angle_deg=-150.0, max_joint_spd=1500)
              for joint in range(1, JOINT_COUNT + 1)}
    after = dict(wanted)
    after[1] = JointLimit(joint=1, max_angle_deg=180.0, min_angle_deg=-150.0,
                          max_joint_spd=1500)
    assert _verify('can_fr', wanted, before, after) is False


def test_verify_rejects_a_joint_that_did_not_answer():
    before = _limits(speed=300)
    wanted = {joint: JointLimit(joint=joint, max_angle_deg=150.0,
                                min_angle_deg=-150.0, max_joint_spd=1500)
              for joint in range(1, JOINT_COUNT + 1)}
    after = dict(wanted)
    del after[6]
    assert _verify('can_fr', wanted, before, after) is False


def test_speed_text_reports_both_the_factory_and_the_widened_ceiling():
    # 出厂 300 与本次目标 1500 的换算必须和文档一致（0.3 / 1.5 rad/s）。
    assert '300' in _speed_text(300)
    assert '17.2 deg/s' in _speed_text(300)
    assert '1500' in _speed_text(1500)
    assert '85.9 deg/s' in _speed_text(1500)
    assert '1.500 rad/s' in _speed_text(1500)
