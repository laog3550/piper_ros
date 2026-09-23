"""Tests for bus frame classification, angle decoding and offset detection."""

import pytest

from piper.piper_feedback import (
    CORE_FEEDBACK_CAN_IDS,
    FEEDBACK_CAN_IDS,
    HIGH_SPEED_CAN_IDS,
    JOINT_ANGLE_IDS,
    TEACHING_INPUT_OFFSETS,
    decode_joint_angles,
    detect_feedback_offset,
    joint_for_can_id,
)


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
