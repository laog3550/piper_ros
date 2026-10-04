"""Tests for bus frame classification, angle decoding and offset detection."""

from collections import Counter

import pytest

from piper.piper_bus_probe import report
from piper.piper_feedback import (
    CORE_FEEDBACK_CAN_IDS,
    FEEDBACK_CAN_IDS,
    HIGH_SPEED_CAN_IDS,
    JOINT_ANGLE_IDS,
    LINKAGE_TEACHING_MODE,
    TEACHING_INPUT_OFFSETS,
    decode_ctrl_mode,
    decode_joint_angles,
    detect_feedback_offset,
    joint_for_can_id,
)


def _status_frame(ctrl_mode: int) -> bytes:
    """Build one arm status payload with the given control mode."""
    return bytes([ctrl_mode] + [0] * 7)


def _angle_frame(joint_a, joint_b, raw_a=None, raw_b=None):
    """Build a joint angle frame from degrees or raw counts."""
    first = int(raw_a) if raw_a is not None else int(round(joint_a * 1000))
    second = int(raw_b) if raw_b is not None else int(round(joint_b * 1000))
    return (first.to_bytes(4, 'big', signed=True)
            + second.to_bytes(4, 'big', signed=True))


@pytest.mark.parametrize('can_id,joints', sorted(JOINT_ANGLE_IDS.items()))
def test_each_angle_frame_carries_its_own_joint_pair(can_id, joints):
    data = _angle_frame(10.0, -20.0)
    assert decode_joint_angles(can_id, data) == {
        joints[0]: pytest.approx(10.0), joints[1]: pytest.approx(-20.0),
    }


def test_angle_decoding_handles_negative_and_fractional_values():
    can_id = 0x2A5
    decoded = decode_joint_angles(can_id, _angle_frame(-73.195, 1.574))
    assert decoded[1] == pytest.approx(-73.195)
    assert decoded[2] == pytest.approx(1.574)


def test_angle_decoding_uses_signed_values():
    # 0xFFFFFFFF is -1 count, i.e. -0.001 degree, not a huge positive angle.
    decoded = decode_joint_angles(0x2A7, _angle_frame(0, 0, raw_a=-1, raw_b=-1))
    assert decoded[5] == pytest.approx(-0.001)
    assert decoded[6] == pytest.approx(-0.001)


def test_angle_frames_of_other_ids_are_ignored():
    assert decode_joint_angles(0x2A1, _angle_frame(1.0, 2.0)) == {}
    assert decode_joint_angles(0x261, _angle_frame(1.0, 2.0)) == {}


def test_short_angle_frame_is_ignored():
    assert decode_joint_angles(0x2A5, _angle_frame(1.0, 2.0)[:7]) == {}


def test_core_ids_cover_the_whole_piper_feedback_layout():
    # 6 high-speed + 6 low-speed + status + 3 end pose + 3 angle + gripper
    assert len(CORE_FEEDBACK_CAN_IDS) == 20
    for can_id in (0x251, 0x256, 0x261, 0x266, 0x2A1, 0x2A8):
        assert can_id in CORE_FEEDBACK_CAN_IDS


def test_plain_layout_reports_no_offset():
    assert detect_feedback_offset(CORE_FEEDBACK_CAN_IDS) is None


def test_plain_layout_with_extra_ids_still_reports_no_offset():
    # The extra frames a master arm broadcasts must not look like a shift.
    observed = set(CORE_FEEDBACK_CAN_IDS) | {0x1C0, 0x1C1, 0x1C2, 0x1C3, 0x212}
    assert detect_feedback_offset(observed) is None


@pytest.mark.parametrize('offset', TEACHING_INPUT_OFFSETS)
def test_shifted_layout_is_detected_from_the_whole_set(offset):
    observed = {can_id + offset for can_id in CORE_FEEDBACK_CAN_IDS}
    assert detect_feedback_offset(observed) == offset


def test_offset_needs_the_whole_set_not_a_single_id():
    # 0x251 + 0x20 and 0x261 + 0x10 are both 0x271, so one ID cannot decide
    # the offset; the full set is what disambiguates it.
    assert 0x251 + 0x20 == 0x261 + 0x10
    assert detect_feedback_offset({0x271}) is None
    assert detect_feedback_offset(CORE_FEEDBACK_CAN_IDS + (0x271,)) is None


def test_offset_is_not_reported_for_an_unrelated_bus():
    assert detect_feedback_offset({0x1C0, 0x1C1, 0x212}) is None
    assert detect_feedback_offset(set()) is None


def test_low_speed_ids_map_to_their_joint():
    for index, can_id in enumerate(FEEDBACK_CAN_IDS, start=1):
        assert joint_for_can_id(can_id) == index
    assert joint_for_can_id(HIGH_SPEED_CAN_IDS[0]) is None


def test_ctrl_mode_comes_from_the_first_status_byte():
    frame = _status_frame(LINKAGE_TEACHING_MODE)
    assert decode_ctrl_mode(0x2A1, frame) == 0x06
    assert decode_ctrl_mode(0x2A1, _status_frame(0x01)) == 0x01


def test_ctrl_mode_ignores_other_frames_and_short_payloads():
    assert decode_ctrl_mode(0x2A5, _status_frame(0x06)) is None
    assert decode_ctrl_mode(0x2A1, b'') is None


def _survey_counter():
    return Counter({can_id: 20 for can_id in CORE_FEEDBACK_CAN_IDS})


def test_two_modes_on_one_bus_name_both_arms(capsys):
    # A shared bus after 0x470 on one arm: the follower still reports CAN
    # control mode while the master reports linkage teaching input.
    report('can_left', _survey_counter(), {0x2A1: _status_frame(0x01)},
           Counter({0x01: 300, LINKAGE_TEACHING_MODE: 300}), 1.0)
    out = capsys.readouterr().out
    assert '0x01 CAN 指令控制 x300' in out
    assert '0x06 联动示教输入 x300' in out
    assert '不止一台臂' in out
    assert '0x470 的 0xFA 生效' in out


def test_single_plain_mode_is_not_reported_as_two_arms(capsys):
    report('can_left', _survey_counter(), {0x2A1: _status_frame(0x01)},
           Counter({0x01: 600}), 1.0)
    out = capsys.readouterr().out
    assert '不止一台臂' not in out
    assert '联动示教输入模式' not in out


def test_linkage_mode_alone_warns_that_both_arms_may_share_it(capsys):
    report('can_left', _survey_counter(),
           {0x2A1: _status_frame(LINKAGE_TEACHING_MODE)},
           Counter({LINKAGE_TEACHING_MODE: 600}), 1.0)
    out = capsys.readouterr().out
    assert '两台都收到过 0x470' in out


def test_commanding_ids_catch_host_and_teaching_arm_frames():
    from piper.piper_feedback import commanding_can_ids
    # 主机的关节指令、使能、示教输入臂偏移后的指令都要被认出来
    assert commanding_can_ids({0x2A5, 0x155, 0x2A1}) == (0x155,)
    assert commanding_can_ids({0x151, 0x471}) == (0x151, 0x471)
    assert commanding_can_ids({0x175, 0x2C5}) == (0x175,)
    # 纯粹的反馈帧不算「有人在发指令」
    assert commanding_can_ids({0x2A5, 0x2A1, 0x251, 0x1C0, 0x212}) == ()


def test_report_flags_command_frames(capsys):
    counter = _survey_counter()
    counter[0x155] = 40  # 有人在给臂下发关节指令
    last = {0x2A1: _status_frame(0x01), 0x155: bytes(8)}
    report('can_left', counter, last, Counter({0x01: 300}), 1.0)
    out = capsys.readouterr().out
    assert '有控制指令帧' in out
    assert '0x155' in out
    assert '示教输入臂' in out


def test_report_says_when_nothing_is_commanding(capsys):
    report('can_left', _survey_counter(), {0x2A1: _status_frame(0x00)},
           Counter({0x00: 300}), 1.0)
    out = capsys.readouterr().out
    assert '没有指令帧' in out
    assert '有控制指令帧' not in out
