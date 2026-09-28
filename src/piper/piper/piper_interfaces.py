"""Central ROS interface registry for Piper arm roles.

The deployed topic names are kept for compatibility.  New code should obtain
them through :func:`arm_interface` instead of assembling names locally.  This
module is the migration seam for a future ``/piper/<side>/<role>`` namespace.
"""

from dataclasses import dataclass


SIDES = ('left', 'right')


@dataclass(frozen=True)
class ArmInterface:
    """All ROS endpoints owned by one follower/master side."""

    side: str
    command_topic: str
    feedback_topic: str
    enable_status_topic: str
    enable_service: str
    default_master_topic: str
    display_name: str


_ARM_INTERFACES = {
    side: ArmInterface(
        side=side,
        command_topic=f'/joint_ctrl_cmd_{side}',
        feedback_topic=f'/joint_states_{side}',
        enable_status_topic=f'/arm_enable_status_{side}',
        enable_service=f'/enable_srv_{side}',
        default_master_topic=f'/joint_states_master_{side}',
        display_name=f'follower_{side} (can_f{side[0]})',
    )
    for side in SIDES
}


def arm_interface(side: str) -> ArmInterface:
    """Return the immutable interface definition for ``side``."""
    try:
        return _ARM_INTERFACES[side]
    except KeyError as error:
        raise ValueError(
            f'unsupported side {side!r}; expected one of {SIDES}') from error


def interface_matrix():
    """Return interfaces in stable side order for launch/config generation."""
    return tuple(_ARM_INTERFACES[side] for side in SIDES)
