# 用途：用合成 CAN 数据验证左右臂、两种帧来源及七维向量的时间和拒绝条件。
# 运行：python3 -m pytest src/piper/test/test_piper_arm_vector.py
import math
import pathlib
import sys
from types import SimpleNamespace

import can
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from piper.piper_arm_vector import ArmVectorAssembler, FRAME_IDS  # noqa: E402
from piper.piper_direct_can_decode import decode_frame  # noqa: E402


BASE_NS = 1_720_000_000_000_000_000


def message(side, kind, index, timestamp_ns, count):
    can_id = FRAME_IDS[kind][index]
    raw = (count.to_bytes(4, 'big', signed=True)
           + (count + 1000).to_bytes(4, 'big', signed=True)) if index < 3 else (
               count.to_bytes(4, 'big', signed=True) + bytes(4))
    decoded = decode_frame(can.Message(
        arbitration_id=can_id, data=raw, timestamp=timestamp_ns / 1e9,
        is_extended_id=False))
    assert decoded is not None
    stamp = SimpleNamespace(sec=timestamp_ns // 1_000_000_000,
                            nanosec=timestamp_ns % 1_000_000_000)
    return SimpleNamespace(
        can_id=decoded['can_id'],
        header=SimpleNamespace(stamp=stamp, frame_id='can_' + side),
        joint_index=decoded['joint_index'],
        joint_position_rad=decoded['joint_position_rad'],
        gripper_position_m=decoded['gripper_position_m'])


@pytest.mark.parametrize('side', ('left', 'right'))
@pytest.mark.parametrize('kind', ('master_target', 'slave_feedback'))
def test_four_can_frames_form_ordered_vector_with_provenance(side, kind):
    assembler = ArmVectorAssembler(side, kind)
    timestamps = [BASE_NS + i * 1_000_000 for i in range(4)]
    frames = [message(side, kind, i, timestamps[i], (i + 1) * 1000)
              for i in range(4)]
    for frame in reversed(frames):
        assert assembler.add(frame)
    result = assembler.snapshot(BASE_NS + 5_000_000)
    assert result['valid']
    assert result['source_topic'] == '/piper_direct/{}/{}'.format(side, kind)
    assert result['frame_id'] == 'can_' + side
    assert result['source_can_ids'] == FRAME_IDS[kind]
    assert result['source_stamps_ns'] == timestamps
    assert result['stamp_ns'] == timestamps[-1]
    assert result['joint_names'] == ('joint1', 'joint2', 'joint3', 'joint4',
                                      'joint5', 'joint6', 'gripper')
    for pair in range(3):
        assert result['position'][pair * 2] == pytest.approx(
            (pair + 1) * 1000 * math.pi / 180000)
        assert result['position'][pair * 2 + 1] == pytest.approx(
            ((pair + 1) * 1000 + 1000) * math.pi / 180000)
    assert result['position'][6] == pytest.approx(0.004)
    assert result['missing'] == [False] * 4
    assert result['stale'] == [False] * 4


def test_missing_stale_skew_and_future_are_rejected():
    assembler = ArmVectorAssembler('left', 'slave_feedback', max_age_ns=10_000_000,
                                   max_skew_ns=5_000_000)
    for index in range(3):
        assert assembler.add(message('left', 'slave_feedback', index,
                                     BASE_NS + index * 1_000_000, 1000))
    partial = assembler.snapshot(BASE_NS + 3_000_000)
    assert not partial['valid'] and partial['missing'] == [False, False, False, True]
    assert assembler.add(message('left', 'slave_feedback', 3,
                                 BASE_NS + 8_000_000, 1000))
    skewed = assembler.snapshot(BASE_NS + 9_000_000)
    assert not skewed['valid'] and skewed['time_inconsistent']
    for index in range(3):
        assert assembler.add(message('left', 'slave_feedback', index,
                                     BASE_NS + 8_000_000, 1000))
    assert assembler.snapshot(BASE_NS + 9_000_000)['valid']
    stale = assembler.snapshot(BASE_NS + 20_000_001)
    assert not stale['valid'] and stale['stale'] == [True] * 4
    assert not assembler.snapshot(BASE_NS - 10_000_001)['valid']


def test_wrong_source_and_older_frame_cannot_replace_valid_values():
    assembler = ArmVectorAssembler('right', 'master_target')
    wrong_side = message('left', 'master_target', 0, BASE_NS, 1000)
    wrong_role = message('right', 'slave_feedback', 0, BASE_NS, 1000)
    assert not assembler.add(wrong_side)
    assert not assembler.add(wrong_role)
    for index in range(4):
        assert assembler.add(message('right', 'master_target', index,
                                     BASE_NS + 5_000_000, 1000))
    before = assembler.snapshot(BASE_NS + 6_000_000)
    assert not assembler.add(message('right', 'master_target', 0, BASE_NS, 9000))
    assert assembler.snapshot(BASE_NS + 6_000_000) == before
