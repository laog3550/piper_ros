"""Tests for the read-only six-joint enable aggregation."""

from types import SimpleNamespace

import pytest

from piper.piper_enable_status import (
    EnableState,
    JOINT_COUNT,
    aggregate,
    describe,
    fill_enable_status,
    is_fully_enabled,
    read_joints,
)

ALL_ON = (True,) * JOINT_COUNT
ALL_OFF = (False,) * JOINT_COUNT


def _motor(enabled, reported=True):
    """Build a stand-in for one ArmMsgFeedbackLowSpd."""
    return SimpleNamespace(
        can_id=0x261 if reported else 0,
        foc_status=SimpleNamespace(driver_enable_status=enabled),
    )


def _piper(observations):
    """Build a stand-in C_PiperInterface, one motor per observation."""
    # None models a joint whose low-speed feedback has never arrived, which
    # the real driver reports as can_id == 0 with its enable bit cleared.
    low_spd = SimpleNamespace(**{
        f'motor_{index}': _motor(bool(enabled), reported=enabled is not None)
        for index, enabled in enumerate(observations, start=1)
    })
    return SimpleNamespace(GetArmLowSpdInfoMsgs=lambda: low_spd)


def _make_joint_unreadable(piper, index):
    """Keep a joint's frame arriving but drop the field we need to read."""
    low_spd = piper.GetArmLowSpdInfoMsgs()
    setattr(low_spd, f'motor_{index}', SimpleNamespace(
        can_id=0x260 + index, foc_status=SimpleNamespace()))


def test_all_six_enabled():
    assert aggregate(ALL_ON) is EnableState.ENABLED


def test_all_six_disabled():
    assert aggregate(ALL_OFF) is EnableState.DISABLED


@pytest.mark.parametrize('enabled_index', range(JOINT_COUNT))
def test_partial_single_joint_is_never_reported_as_enabled(enabled_index):
    joints = [False] * JOINT_COUNT
    joints[enabled_index] = True
    assert aggregate(joints) is EnableState.PARTIAL


def test_partial_majority_enabled_is_still_partial():
    assert aggregate((True, True, True, True, True, False)) is (
        EnableState.PARTIAL
    )


def test_unreported_joint_is_unknown_not_disabled():
    joints = [None] * JOINT_COUNT
    assert aggregate(joints) is EnableState.UNKNOWN


def test_unreported_joint_never_yields_enabled():
    # Five joints report enabled and one is silent: still not enabled.
    assert aggregate((True, True, True, True, True, None)) is (
        EnableState.UNKNOWN
    )


def test_invalid_joint_count_is_rejected():
    with pytest.raises(ValueError, match='expected 6 joints'):
        aggregate((True, False))


def test_read_joints_reports_each_joint_independently():
    assert read_joints(_piper((True, False, True, False, True, False))) == (
        (True, False, True, False, True, False)
    )


def test_read_joints_treats_unreported_frames_as_unknown():
    observations = read_joints(_piper((True, None, True, None, True, None)))
    assert observations == (True, None, True, None, True, None)
    assert aggregate(observations) is EnableState.UNKNOWN


def test_unreadable_joint_data_is_unknown():
    # The frame arrives but the enable bit cannot be read, which must not be
    # silently downgraded to "disabled".
    piper = _piper(ALL_ON)
    _make_joint_unreadable(piper, 3)
    joints = read_joints(piper)
    assert joints[2] is None
    assert aggregate(joints) is EnableState.UNKNOWN


def test_left_and_right_arms_do_not_share_state():
    left = _piper(ALL_ON)
    right = _piper(ALL_OFF)

    left_joints = read_joints(left)
    right_joints = read_joints(right)

    assert left_joints == ALL_ON
    assert right_joints == ALL_OFF
    assert aggregate(left_joints) is EnableState.ENABLED
    assert aggregate(right_joints) is EnableState.DISABLED

    # A fresh right-hand observation must not disturb the left-hand verdict.
    assert read_joints(right) == ALL_OFF
    assert aggregate(read_joints(left)) is EnableState.ENABLED


def test_fill_enable_status_keeps_joint_order_and_ports_apart():
    for can_port, joints, state in (
        ('can_fr', ALL_ON, EnableState.ENABLED),
        ('can_ml', (True, False, True, False, True, False),
         EnableState.PARTIAL),
    ):
        message = fill_enable_status(
            SimpleNamespace(), joints, state, can_port)
        assert message.can_port == can_port
        assert message.state == int(state)
        assert [getattr(message, f'joint_{index}_enabled')
                for index in range(1, JOINT_COUNT + 1)] == list(
                    bool(enabled) for enabled in joints)
        assert [getattr(message, f'joint_{index}_valid')
                for index in range(1, JOINT_COUNT + 1)] == list(
                    enabled is not None for enabled in joints)
        assert message.all_enabled is (state is EnableState.ENABLED)
        assert message.any_enabled is any(bool(e) for e in joints)


def test_partial_and_unknown_never_set_all_enabled():
    for joints in (
        ALL_OFF,
        (True, True, True, True, True, False),
        (True,) * (JOINT_COUNT - 1) + (None,),
        (None,) * JOINT_COUNT,
    ):
        state = aggregate(joints)
        message = fill_enable_status(SimpleNamespace(), joints, state, 'can0')
        assert state is not EnableState.ENABLED
        assert message.all_enabled is False


def test_describe_lists_every_joint():
    assert describe((True, False, None, True, False, None)) == (
        'j1=on j2=off j3=unknown j4=on j5=off j6=unknown'
    )


def test_motion_gate_opens_only_for_six_enabled_joints():
    assert is_fully_enabled(ALL_ON) is True


def test_motion_gate_stays_shut_for_every_incomplete_arm():
    # Each of these is a state in which motion must not reach the arm.
    for joints in (
        ALL_OFF,
        (True, True, True, True, True, False),
        (False, False, False, False, False, True),
        (True, True, True, True, True, None),
        (None,) * JOINT_COUNT,
        (),
        (True, True, True),
    ):
        assert is_fully_enabled(joints) is False


def test_motion_gate_rejects_a_joint_that_dropped_out():
    # The enable service confirmed six-of-six, then joint 4 dropped out; the
    # gate must close even though the node's enable flag is still latched.
    piper = _piper(ALL_ON)
    assert is_fully_enabled(read_joints(piper)) is True
    low_spd = piper.GetArmLowSpdInfoMsgs()
    low_spd.motor_4 = _motor(False, reported=True)
    assert read_joints(piper)[3] is False
    assert is_fully_enabled(read_joints(piper)) is False


def test_motion_gate_closes_when_a_joint_goes_silent():
    piper = _piper(ALL_ON)
    low_spd = piper.GetArmLowSpdInfoMsgs()
    low_spd.motor_5 = _motor(True, reported=False)
    assert read_joints(piper)[4] is None
    assert is_fully_enabled(read_joints(piper)) is False
