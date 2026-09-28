#!/usr/bin/env python3
"""Decode Piper CAN feedback frames and track observation freshness."""

# Frames are decoded straight from the CAN bus, so this module depends on
# neither piper_sdk nor ROS.  Callers feed it frames the arm already
# broadcasts; nothing here transmits.

import math
import time
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

JOINT_COUNT = 6
# Low-speed driver information feedback, one frame per joint.
FEEDBACK_FIRST_CAN_ID = 0x261
FEEDBACK_CAN_IDS = tuple(
    range(FEEDBACK_FIRST_CAN_ID, FEEDBACK_FIRST_CAN_ID + JOINT_COUNT)
)
FEEDBACK_DATA_LENGTH = 8
# Byte 5 of the frame is the driver status word; bit 6 is driver_enable_status.
STATUS_BYTE = 5
ENABLE_BIT = 0x40

# High-speed driver information feedback, one frame per joint.
HIGH_SPEED_CAN_IDS = tuple(range(0x251, 0x257))
# Joint angles are packed two per frame.
JOINT_ANGLE_IDS = {0x2A5: (1, 2), 0x2A6: (3, 4), 0x2A7: (5, 6)}
ARM_STATUS_CAN_ID = 0x2A1
END_POSE_CAN_IDS = (0x2A2, 0x2A3, 0x2A4)
GRIPPER_FEEDBACK_CAN_ID = 0x2A8
# Every frame a healthy Piper arm broadcasts on its own, in any arm mode.
CORE_FEEDBACK_CAN_IDS = (
    HIGH_SPEED_CAN_IDS
    + FEEDBACK_CAN_IDS
    + (ARM_STATUS_CAN_ID,)
    + END_POSE_CAN_IDS
    + tuple(JOINT_ANGLE_IDS)
    + (GRIPPER_FEEDBACK_CAN_ID,)
)
# Making an arm the teaching input arm shifts its feedback IDs by one of these
# amounts (CAN 0x470); see MasterSlaveConfig in the SDK.  An arm that never
# received that command stays a motion output arm and keeps the plain IDs.
TEACHING_INPUT_OFFSETS = (0x10, 0x20)
_ANGLE_SCALE = 0.001  # raw joint angles are 0.001 degree per count

# True = enabled, False = disabled, None = no usable recent frame.
JointObservations = Tuple[Optional[bool], ...]


def joint_for_can_id(can_id: int) -> Optional[int]:
    """Map a low-speed feedback CAN ID to its 1-based joint number."""
    if FEEDBACK_FIRST_CAN_ID <= can_id <= FEEDBACK_CAN_IDS[-1]:
        return can_id - FEEDBACK_FIRST_CAN_ID + 1
    return None


def detect_feedback_offset(observed_can_ids) -> Optional[int]:
    """Return the whole-set feedback offset, or None for the plain layout."""
    # The offset cannot be decided from a single ID: shifting a high-speed
    # frame by 0x20 lands on the same ID as shifting a low-speed frame by
    # 0x10, so only the whole set disambiguates the layout being used.  A
    # plain arm yields None because no offset maps all its core IDs back.
    unique = set(observed_can_ids)
    for offset in TEACHING_INPUT_OFFSETS:
        matched = sum(
            1 for can_id in unique
            if can_id - offset in CORE_FEEDBACK_CAN_IDS
        )
        if matched >= len(CORE_FEEDBACK_CAN_IDS):
            return offset
    return None


def decode_joint_angles(can_id: int, data) -> Dict[int, float]:
    """Decode one joint angle frame into degrees, keyed by joint number."""
    joints = JOINT_ANGLE_IDS.get(can_id)
    if joints is None or len(data) < 8:
        return {}
    return {
        joints[0]: int.from_bytes(data[0:4], 'big', signed=True) * _ANGLE_SCALE,
        joints[1]: int.from_bytes(data[4:8], 'big', signed=True) * _ANGLE_SCALE,
    }


def _status_byte(data) -> Optional[int]:
    if len(data) < FEEDBACK_DATA_LENGTH:
        return None
    return data[STATUS_BYTE]


@dataclass(frozen=True)
class DriverFeedback:
    """One decoded low-speed driver feedback frame."""

    joint: int
    enabled: bool
    status_code: int
    voltage: float
    foc_temperature: int


def decode(can_id: int, data) -> Optional[DriverFeedback]:
    """Decode one feedback frame, or None if it is not one."""
    joint = joint_for_can_id(can_id)
    if joint is None:
        return None
    status = _status_byte(data)
    if status is None:
        return None
    raw_voltage = int.from_bytes(data[:2], 'big')
    raw_temperature = int.from_bytes(data[2:4], 'big', signed=True)
    return DriverFeedback(
        joint=joint,
        enabled=bool(status & ENABLE_BIT),
        status_code=status,
        voltage=raw_voltage * 0.1,
        foc_temperature=raw_temperature,
    )


class FeedbackTracker:
    """Keep the newest feedback frame per joint and how old it is."""

    def __init__(self, timeout: float = 0.5):
        if timeout <= 0:
            raise ValueError('timeout must be positive')
        self.timeout = timeout
        self._feedback: Dict[int, DriverFeedback] = {}
        self._last_seen: Dict[int, float] = {}

    def update(self, can_id: int, data, now: Optional[float] = None):
        """Record one frame and return the decoded feedback it carried."""
        feedback = decode(can_id, data)
        if feedback is None:
            return None
        self._feedback[feedback.joint] = feedback
        self._last_seen[feedback.joint] = (
            time.monotonic() if now is None else now
        )
        return feedback

    def age(self, joint: int, now: Optional[float] = None):
        """Seconds since this joint's newest frame, or None if never seen."""
        seen = self._last_seen.get(joint)
        if seen is None:
            return None
        current = time.monotonic() if now is None else now
        return max(0.0, current - seen)

    def known_joints(self) -> Tuple[int, ...]:
        """Joints that have sent at least one feedback frame."""
        return tuple(sorted(self._feedback))

    def is_fresh(self, joint: int, now: Optional[float] = None) -> bool:
        """Whether this joint's newest frame is within the timeout."""
        age = self.age(joint, now)
        return age is not None and age <= self.timeout

    def last_known(self, joint: int):
        """Return the newest decoded frame for this joint, fresh or not."""
        return self._feedback.get(joint)

    def observations(self, now: Optional[float] = None) -> JointObservations:
        """Per-joint enable bits, with None for missing or stale joints."""
        result = []
        for joint in range(1, JOINT_COUNT + 1):
            if not self.is_fresh(joint, now):
                # A joint that stopped reporting must not keep contributing
                # its last enable bit as if it were current.
                result.append(None)
                continue
            result.append(self._feedback[joint].enabled)
        return tuple(result)

    def stale_joints(self, now: Optional[float] = None) -> Tuple[int, ...]:
        """Joints already seen at least once but silent past the timeout."""
        return tuple(
            joint for joint in self.known_joints()
            if not self.is_fresh(joint, now)
        )

    def missing_joints(self) -> Tuple[int, ...]:
        """Joints that have never sent a feedback frame."""
        return tuple(
            joint for joint in range(1, JOINT_COUNT + 1)
            if joint not in self._feedback
        )


class ArmAngleTracker:
    """Track one arm's six joint angles against a baseline pose."""

    def __init__(self):
        self._angles: Dict[int, float] = {}
        self._baseline: Dict[int, float] = {}
        self._max_offset: Dict[int, float] = {}
        self._last_angle: Dict[int, float] = {}
        self._updated: Optional[float] = None
        self._last_change: Optional[float] = None

    def update(self, can_id: int, data, now: float = None,
               change_threshold: float = 0.05) -> Tuple[int, ...]:
        """Store the angles carried by one frame; return the joints it moved."""
        decoded = decode_joint_angles(can_id, data)
        if not decoded:
            return ()
        current = time.monotonic() if now is None else now
        moved = []
        for joint, angle in decoded.items():
            previous = self._last_angle.get(joint)
            if previous is None or abs(angle - previous) >= change_threshold:
                moved.append(joint)
                self._last_change = current
            self._last_angle[joint] = angle
            self._angles[joint] = angle
            if joint not in self._baseline:
                self._baseline[joint] = angle
            offset = abs(angle - self._baseline[joint])
            if joint not in self._max_offset or offset > self._max_offset[joint]:
                self._max_offset[joint] = offset
        self._updated = current
        return tuple(moved)

    def set_baseline(self) -> None:
        """Re-anchor every joint to its current angle and forget the peaks."""
        self._baseline = dict(self._angles)
        # Seed a zero peak per known joint, so a joint that simply has not
        # moved is shown as 0.0 rather than as missing data.
        self._max_offset = {joint: 0.0 for joint in self._angles}

    def angles(self) -> Dict[int, float]:
        """Newest angle of each joint that has reported."""
        return dict(self._angles)

    def offsets(self) -> Dict[int, float]:
        """Signed displacement of each joint from its baseline."""
        return {
            joint: angle - self._baseline[joint]
            for joint, angle in self._angles.items()
            if joint in self._baseline
        }

    def max_offsets(self) -> Dict[int, float]:
        """Largest absolute displacement each joint reached since baseline."""
        return dict(self._max_offset)

    def age(self, now: Optional[float] = None) -> Optional[float]:
        """Seconds since the newest angle frame, or None if none arrived."""
        if self._updated is None:
            return None
        current = time.monotonic() if now is None else now
        return max(0.0, current - self._updated)

    def is_moving(self, window: float = 0.5,
                  now: Optional[float] = None) -> bool:
        """Whether a joint changed within the last ``window`` seconds."""
        if self._last_change is None:
            return False
        current = time.monotonic() if now is None else now
        return (current - self._last_change) <= window

    def moved_joints(self, threshold: float) -> Tuple[int, ...]:
        """Joints displaced from the baseline by at least ``threshold``."""
        return tuple(sorted(
            joint for joint, offset in self._max_offset.items()
            if offset >= threshold
        ))

__all__ = [
    'JOINT_COUNT',
    'FEEDBACK_FIRST_CAN_ID',
    'FEEDBACK_CAN_IDS',
    'FEEDBACK_DATA_LENGTH',
    'STATUS_BYTE',
    'ENABLE_BIT',
    'HIGH_SPEED_CAN_IDS',
    'JOINT_ANGLE_IDS',
    'ARM_STATUS_CAN_ID',
    'END_POSE_CAN_IDS',
    'GRIPPER_FEEDBACK_CAN_ID',
    'CORE_FEEDBACK_CAN_IDS',
    'TEACHING_INPUT_OFFSETS',
    'JointObservations',
    'joint_for_can_id',
    'detect_feedback_offset',
    'decode_joint_angles',
    'DriverFeedback',
    'decode',
    'FeedbackTracker',
    'ArmAngleTracker',
]
