"""Tests for the offset master state publisher (no CAN traffic involved)."""

from pathlib import Path

import pytest

from piper.piper_feedback import DEG_TO_RAD, JOINT_COUNT, parse_offset
from piper.piper_master_state import build_joint_state, frame_filters

MODULE = Path(__file__).resolve().parents[1] / 'piper' / 'piper_master_state.py'


def _joints(overrides=None):
    angles = {joint: 0.0 for joint in range(1, JOINT_COUNT + 1)}
    angles.update(overrides or {})
    return angles


def test_state_carries_six_joints_in_radians_and_the_gripper():
    message = build_joint_state(_joints({1: 10.0, 6: -20.0}), 0.03)
    assert len(message.position) == JOINT_COUNT + 1
    assert message.position[0] == pytest.approx(10.0 * DEG_TO_RAD)
    assert message.position[5] == pytest.approx(-20.0 * DEG_TO_RAD)
    assert message.position[6] == pytest.approx(0.03)
    assert message.name[-1] == 'gripper'


def test_state_omits_the_gripper_when_it_has_no_fresh_frame():
    # teleop reads the gripper from position[6] only when it is present.
    message = build_joint_state(_joints(), None)
    assert len(message.position) == JOINT_COUNT
    assert message.name == [f'joint{joint}' for joint in range(1, 7)]


def test_joint_order_does_not_depend_on_dict_order():
    shuffled = {6: 6.0, 1: 1.0, 3: 3.0, 2: 2.0, 5: 5.0, 4: 4.0}
    message = build_joint_state(shuffled, None)
    assert message.position == pytest.approx(
        [value * DEG_TO_RAD for value in (1.0, 2.0, 3.0, 4.0, 5.0, 6.0)])


def test_radians_scale_uses_exact_si_conversion():
    import math
    assert DEG_TO_RAD == pytest.approx(math.pi / 180.0)
    message = build_joint_state(_joints({1: 90.0}), None)
    assert message.position[0] == pytest.approx(math.pi / 2.0)


@pytest.mark.parametrize('offset', (0x00, 0x10, 0x20))
def test_filters_follow_the_offset(offset):
    ids = {entry['can_id'] for entry in frame_filters(offset)}
    assert ids == {can_id + offset
                   for can_id in (0x2A5, 0x2A6, 0x2A7, 0x2A8)}


def test_parse_offset_rejects_unsupported_values():
    with pytest.raises(ValueError, match='unsupported offset'):
        parse_offset('0x40')


def test_module_has_no_can_transmit_path():
    # 本节点的安全属性：只收不发。出现任何发送调用都要在这里挡住。
    source = MODULE.read_text(encoding='utf-8')
    for forbidden in ('.send(', 'can.Message(', 'send_periodic',
                      'EnableArm', 'DisableArm', 'MotionCtrl'):
        assert forbidden not in source, forbidden
