#!/usr/bin/env python3
# 用途：把只读直连 CAN 逐帧消息组装为可追溯的七维主臂目标或从臂反馈。
# 运行：ros2 run piper piper_arm_vector；离线验证：python3 -m pytest src/piper/test/test_piper_arm_vector.py
"""Pure frame assembly; the ROS wrapper is in piper_arm_vector_node.py."""

import math


JOINT_NAMES = ('joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper')
FRAME_IDS = {
    'master_target': (0x155, 0x156, 0x157, 0x159),
    'slave_feedback': (0x2A5, 0x2A6, 0x2A7, 0x2A8),
}


def stamp_ns(stamp):
    """Read a ROS stamp without losing its nanosecond component."""
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
        return 0
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class ArmVectorAssembler:
    def __init__(self, side, kind, max_age_ns=100_000_000, max_skew_ns=30_000_000,
                 future_tolerance_ns=10_000_000):
        if side not in ('left', 'right') or kind not in FRAME_IDS:
            raise ValueError('unknown side or source kind')
        if min(max_age_ns, max_skew_ns, future_tolerance_ns) < 0:
            raise ValueError('negative time threshold')
        self.side = side
        self.kind = kind
        self.channel = 'can_' + side
        self.source_topic = '/piper_direct/{}/{}'.format(side, kind)
        self.ids = FRAME_IDS[kind]
        self.max_age_ns = max_age_ns
        self.max_skew_ns = max_skew_ns
        self.future_tolerance_ns = future_tolerance_ns
        self._frames = [None] * 4

    def add(self, frame):
        """Accept only the expected source, frame identity and finite decoded values."""
        if frame.can_id not in self.ids or frame.header.frame_id != self.channel:
            return False
        index = self.ids.index(frame.can_id)
        expected_joint = (1, 3, 5, 0)[index]
        if frame.joint_index != expected_joint:
            return False
        values = (frame.joint_position_rad if index < 3
                  else (frame.gripper_position_m,))
        if len(values) != (2 if index < 3 else 1) or not all(
                math.isfinite(value) for value in values):
            return False
        timestamp = stamp_ns(frame.header.stamp)
        if timestamp <= 0:
            return False
        previous = self._frames[index]
        if previous is not None and timestamp < previous[0]:
            return False
        self._frames[index] = (timestamp, tuple(values))
        return True

    def snapshot(self, now_ns):
        """Return source stamps and explicit rejection flags at the requested Unix time."""
        stamps = [item[0] if item else 0 for item in self._frames]
        missing = [item is None for item in self._frames]
        stale = [bool(stamp and now_ns - stamp > self.max_age_ns)
                 for stamp in stamps]
        future = any(stamp > now_ns + self.future_tolerance_ns for stamp in stamps)
        present_stamps = [stamp for stamp in stamps if stamp]
        skew = (max(present_stamps) - min(present_stamps)
                if len(present_stamps) == 4 else 0)
        time_inconsistent = future or skew > self.max_skew_ns
        position = [0.0] * 7
        for index, item in enumerate(self._frames):
            if item:
                start = index * 2 if index < 3 else 6
                position[start:start + len(item[1])] = item[1]
        return {
            'stamp_ns': max(stamps),
            'source_stamps_ns': stamps,
            'source_can_ids': self.ids,
            'source_topic': self.source_topic,
            'frame_id': self.channel,
            'joint_names': JOINT_NAMES,
            'position': position,
            'missing': missing,
            'stale': stale,
            'time_inconsistent': time_inconsistent,
            'valid': not any(missing) and not any(stale) and not time_inconsistent,
        }
