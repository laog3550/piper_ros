#!/usr/bin/env python3
# 用途：用 piper_sdk V2 解码直连摇操的目标帧与实测反馈帧。
# 运行：python3 -m pytest src/piper/test/test_direct_can.py
import math

from piper_sdk.protocol import C_PiperParserV2
from piper_sdk.piper_msgs.msg_v2 import PiperMessage


TARGET_IDS = frozenset((0x155, 0x156, 0x157, 0x159))
FEEDBACK_IDS = frozenset((0x2A5, 0x2A6, 0x2A7, 0x2A8))
FRAME_IDS = TARGET_IDS | FEEDBACK_IDS
RAD_PER_MILLI_DEGREE = math.pi / 180000.0


def decode_frame(frame, parser=None):
    """Return one independently timestamped frame, or None for unrelated/invalid traffic."""
    can_id = frame.arbitration_id
    if (can_id not in FRAME_IDS or frame.is_extended_id or frame.is_remote_frame
            or frame.is_error_frame or frame.is_fd or len(frame.data) != 8
            or frame.timestamp <= 0):
        return None

    parser = parser or C_PiperParserV2()
    decoded = PiperMessage()
    if not parser.DecodeMessage(frame, decoded):
        return None

    result = {
        'stamp': frame.timestamp,
        'can_id': can_id,
        'raw_data': list(frame.data),
        'kind': 'master_target' if can_id in TARGET_IDS else 'slave_feedback',
        'joint_index': 0,
        'joint_position_rad': [0.0, 0.0],
        'gripper_position_m': 0.0,
        'gripper_effort_nm': 0.0,
        'gripper_status': 0,
        'gripper_set_zero': 0,
    }
    if can_id in (0x155, 0x156, 0x157, 0x2A5, 0x2A6, 0x2A7):
        index = (can_id - 0x155) if can_id in TARGET_IDS else (can_id - 0x2A5)
        first_joint = index * 2 + 1
        joints = decoded.arm_joint_ctrl if can_id in TARGET_IDS else decoded.arm_joint_feedback
        result['joint_index'] = first_joint
        result['joint_position_rad'] = [
            getattr(joints, 'joint_{}'.format(j)) * RAD_PER_MILLI_DEGREE
            for j in (first_joint, first_joint + 1)
        ]
    else:
        gripper = decoded.arm_gripper_ctrl if can_id in TARGET_IDS else decoded.gripper_feedback
        result['gripper_position_m'] = gripper.grippers_angle * 1e-6
        result['gripper_effort_nm'] = gripper.grippers_effort * 1e-3
        result['gripper_status'] = gripper.status_code
        if can_id in TARGET_IDS:
            result['gripper_set_zero'] = gripper.set_zero
    return result
