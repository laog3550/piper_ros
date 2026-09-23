#!/usr/bin/env python3
"""Read-only aggregation of one Piper arm's six joint enable states."""

# Everything in this module observes the arm.  Sampling the enable state reads
# the low-speed driver feedback that the arm already broadcasts, so it puts no
# frame on the CAN bus and cannot enable, disable or move the arm.

from enum import IntEnum
from typing import Optional, Sequence, Tuple

JOINT_COUNT = 6

# One observation per joint: True = enabled, False = disabled, None = this
# joint has not reported usable data, so its "disabled" bit carries no
# information.
JointObservations = Tuple[Optional[bool], ...]


class EnableState(IntEnum):
    """Aggregate enable state of the six joint drivers of one arm."""

    UNKNOWN = 0
    DISABLED = 1
    PARTIAL = 2
    ENABLED = 3


def aggregate(joints: Sequence[Optional[bool]]) -> EnableState:
    """Classify six per-joint observations into an arm-level enable state."""
    if len(joints) != JOINT_COUNT:
        raise ValueError(
            f'expected {JOINT_COUNT} joints, got {len(joints)}'
        )
    # A joint with no usable data must never be counted as an enabled one,
    # so an incomplete observation can only ever be UNKNOWN.
    if any(enabled is None for enabled in joints):
        return EnableState.UNKNOWN
    if all(joints):
        return EnableState.ENABLED
    if not any(joints):
        return EnableState.DISABLED
    return EnableState.PARTIAL


def _joint_flag(motor) -> Optional[bool]:
    """Read one motor's enable bit, or None if it has no usable data."""
    if motor is None:
        return None
    try:
        # can_id stays 0 until this joint's low-speed frame arrives, which is
        # what separates "not reported yet" from "reported disabled".
        if motor.can_id == 0:
            return None
        return bool(motor.foc_status.driver_enable_status)
    except AttributeError:
        return None


def read_joints(piper) -> JointObservations:
    """Sample the six driver enable flags from an open SDK interface."""
    low_spd = piper.GetArmLowSpdInfoMsgs()
    return tuple(
        _joint_flag(getattr(low_spd, f'motor_{index}', None))
        for index in range(1, JOINT_COUNT + 1)
    )


def is_fully_enabled(joints: Sequence[Optional[bool]]) -> bool:
    """Whether all six joints are reporting and enabled right now."""
    if len(joints) != JOINT_COUNT:
        return False
    # all() is False for a False or a None entry, so a joint that is disabled,
    # silent or stale can never let motion through.
    return all(joints)


def describe(joints: Sequence[Optional[bool]]) -> str:
    """Render a per-joint breakdown such as ``j1=on j2=off j3=unknown``."""
    labels = {True: 'on', False: 'off', None: 'unknown'}
    return ' '.join(
        f'j{index}={labels[enabled]}'
        for index, enabled in enumerate(joints, start=1)
    )


def fill_enable_status(message, joints: Sequence[Optional[bool]],
                       state: EnableState, can_port: str):
    """Copy an observation into a ``PiperEnableStatusMsg``-shaped object."""
    if len(joints) != JOINT_COUNT:
        raise ValueError(
            f'expected {JOINT_COUNT} joints, got {len(joints)}'
        )
    (message.joint_1_enabled, message.joint_2_enabled,
     message.joint_3_enabled, message.joint_4_enabled,
     message.joint_5_enabled, message.joint_6_enabled) = (
        bool(enabled) for enabled in joints
    )
    (message.joint_1_valid, message.joint_2_valid,
     message.joint_3_valid, message.joint_4_valid,
     message.joint_5_valid, message.joint_6_valid) = (
        enabled is not None for enabled in joints
    )
    message.all_enabled = state is EnableState.ENABLED
    message.any_enabled = any(bool(enabled) for enabled in joints)
    message.state = int(state)
    message.can_port = can_port
    return message
