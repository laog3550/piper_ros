"""Tests for raw feedback decoding and per-joint freshness tracking."""

import pytest

from piper.piper_feedback import (
    FEEDBACK_FIRST_CAN_ID,
    JOINT_COUNT,
    FeedbackTracker,
    decode,
    joint_for_can_id,
)

ALL_ON = (True,) * JOINT_COUNT
ALL_OFF = (False,) * JOINT_COUNT


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
