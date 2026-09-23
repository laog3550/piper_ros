#!/usr/bin/env python3
"""Decode Piper driver feedback frames and track how fresh they are."""

# Frames are decoded straight from the CAN bus, so this module depends on
# neither piper_sdk nor ROS.  Callers feed it frames the arm already
# broadcasts; nothing here transmits.

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

# True = enabled, False = disabled, None = no usable recent frame.
JointObservations = Tuple[Optional[bool], ...]


def joint_for_can_id(can_id: int) -> Optional[int]:
    """Map a feedback CAN ID to its 1-based joint number."""
    if FEEDBACK_FIRST_CAN_ID <= can_id <= FEEDBACK_CAN_IDS[-1]:
        return can_id - FEEDBACK_FIRST_CAN_ID + 1
    return None


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
