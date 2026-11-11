"""验证异相位 RGB 对齐的因果性、固定网格和完成帧的失效筛选。"""

import json
from pathlib import Path
import sys

import numpy as np
import pytest
from rosbags.rosbag2 import Writer

sys.path.insert(0, str(Path(__file__).parent))
import to_lerobot as converter
from completion_validation import RosSqliteReader, validate_completion

BASE = 1_800_000_100_000_000_000


def phased_episode(tmp_path, invalid=None):
    path = tmp_path / 'episode_000001'
    path.mkdir()
    meta = {'schema_version': 2, 'state': 'saved', 'mode': 'skill', 'episode_id': 'phase-test',
            'task': '香蕉入盒', 'skill': converter.SKILLS[0], 'outcome': 'success',
            'started_at_unix_ns': BASE - 20_000_000, 'ready_at_unix_ns': BASE,
            'stopped_at_unix_ns': BASE + 3_600_000_000,
            'task_metadata': {'arm_mode': 'LEFT', 'time_domain': 'unix_ns',
                              'action_space': 'absolute_joint_position_7',
                              'action_provenance': 'human_demonstration',
                              'calibration_version': 'synthetic-calibration',
                              'data_convention_version': 'synthetic-data-convention',
                              'normalization_version': 'synthetic-normalization'}}
    store = converter.TYPESTORE
    Time = store.types['builtin_interfaces/msg/Time']
    Header = store.types['std_msgs/msg/Header']
    Vector = store.types[converter.VECTOR_TYPE]
    Image = store.types['sensor_msgs/msg/Image']
    def stamp(ns):
        return Time(ns // 1_000_000_000, ns % 1_000_000_000)
    topics = {converter.CAN_ACTIONS['left']: converter.VECTOR_TYPE,
              converter.CAN_STATES['left']: converter.VECTOR_TYPE,
              converter.CAMERAS['top_rgb']: 'sensor_msgs/msg/Image',
              converter.CAMERAS['left_wrist_rgb']: 'sensor_msgs/msg/Image'}
    meta['topics'] = list(topics)
    meta['frame_topics'] = list(converter.CAMERAS.values())[:2]
    (path / 'episode.json').write_text(json.dumps(meta), encoding='utf-8')
    events = []
    for i in range(176):
        t = BASE + i * 20_000_000
        for topic in list(topics)[:2]:
            kind = topic.rsplit('/', 1)[-1]
            bad = bool(invalid and kind == 'master_target' and invalid(i))
            msg = Vector(Header(stamp(t), 'can_left'), f'/piper_direct/left/{kind}',
                         converter.JOINTS, np.array([(t - BASE) / 1e9] * 6 + [.01]),
                         np.array(converter.CAN_IDS[kind], dtype=np.uint32),
                         [stamp(t - n * 1_000_000) for n in (3, 2, 1, 0)],
                         np.zeros(4, dtype=np.bool_), np.full(4, bad, dtype=np.bool_),
                         False, not bad, True)
            events.append((t, topic, msg))
    for topic, phase in [(converter.CAMERAS['top_rgb'], 25_000_000),
                         (converter.CAMERAS['left_wrist_rgb'], 0)]:
        for i in range(53):
            t = BASE + phase + round(i * 1_000_000_000 / 15)
            msg = Image(Header(stamp(t), 'synthetic_camera'), 480, 640, 'rgb8', 0, 1920,
                        np.zeros(480 * 640 * 3, dtype=np.uint8))
            events.append((t, topic, msg))
    with Writer(path / 'bag', version=Writer.VERSION_LATEST) as writer:
        connections = {t: writer.add_connection(t, typ, typestore=store) for t, typ in topics.items()}
        for t, topic, msg in sorted(events, key=lambda row: row[0]):
            writer.write(connections[topic], t + 1000, store.serialize_cdr(msg, topics[topic]))
    return path, meta


def read(path):
    return converter.read_episode(path, ['left'], 10, 100_000_000, 2_000_000,
                                  input_format='can-vector', max_source_skew_ns=30_000_000,
                                  extract_valid_segments=True)


def test_15fps_rgb_phase_keeps_10fps_grid_and_real_causal_sources(tmp_path):
    path, _ = phased_episode(tmp_path)
    before = converter.source_digest(path)
    meta, frames = read(path)
    assert len(frames) >= 30 and max(s['frames'] for s in meta['_quality']['segments']) >= 30
    assert all(b['__target_timestamp_ns__'] - a['__target_timestamp_ns__'] == 100_000_000
               for a, b in zip(frames, frames[1:]))
    for frame in frames:
        grid = frame['__target_timestamp_ns__']
        anchor = frame['__alignment__']['reference_timestamp_ns']
        stamps = frame['__source_header_ns__']
        assert all(stamp <= anchor <= grid and grid - stamp <= 100_000_000 for stamp in stamps.values())
        assert max(stamps.values()) - min(stamps.values()) <= 30_000_000
        assert frame['action'][0] == pytest.approx((stamps[converter.CAN_ACTIONS['left']] - BASE) / 1e9)
    assert any(f['__alignment__']['grid_lag_ns'] > 30_000_000 for f in frames)
    assert converter.source_digest(path) == before


def test_completion_native_sqlite_adapter_matches_converter_and_passes_long_segment(tmp_path):
    path, meta = phased_episode(tmp_path)
    checked, frames = read(path)
    topic = converter.CAMERAS['top_rgb']
    timestamp = checked['_camera_header_stamps'][topic][-2]
    result = validate_completion(path, meta, topic, timestamp, exact_ns=timestamp,
                                 deserialize=converter.TYPESTORE.deserialize_cdr)
    assert result['contiguous_frames'] >= 16 and result['header_stamp_ns'] == timestamp
    native, native_frames = converter.read_episode(
        path, ['left'], 10, 100_000_000, 2_000_000, input_format='can-vector',
        max_source_skew_ns=30_000_000, extract_valid_segments=True,
        _reader_factory=RosSqliteReader, _deserialize=converter.TYPESTORE.deserialize_cdr)
    assert native['_quality'] == checked['_quality']
    assert [f['__source_header_ns__'] for f in native_frames] == [f['__source_header_ns__'] for f in frames]


def test_completion_rejects_stale_hole_and_cannot_borrow_earlier_success(tmp_path):
    path, meta = phased_episode(tmp_path, invalid=lambda i: 140 <= i < 170)
    before = converter.source_digest(path)
    checked, _ = read(path)
    assert max(s['frames'] for s in checked['_quality']['segments']) >= 16
    with pytest.raises(ValueError, match='完成帧来源校验未通过'):
        validate_completion(path, meta, converter.CAMERAS['top_rgb'], BASE + 3_400_000_000,
                            strictly_before=True, deserialize=converter.TYPESTORE.deserialize_cdr)
    assert converter.source_digest(path) == before


def test_source_fault_between_grid_points_still_splits_camera_bundles(tmp_path):
    path, _ = phased_episode(tmp_path, invalid=lambda i: i == 77)
    meta, frames = read(path)
    assert len(meta['_quality']['segments']) >= 2
    for segment in meta['_quality']['segments']:
        times = [f['__target_timestamp_ns__'] for f in frames if f['__segment_index__'] == segment['segment_index']]
        assert not (min(times) < BASE + 1_540_000_000 < max(times))


def test_training_gate_checks_real_grid_age_and_future_sources(tmp_path):
    sys.path.insert(0, str(Path(__file__).parents[1] / 'vla_contract'))
    from atomic_skill_sft import check_source_timestamps
    stamps = {converter.CAMERAS['top_rgb']: BASE + 60_000_000,
              converter.CAMERAS['left_wrist_rgb']: BASE + 35_000_000,
              converter.CAN_ACTIONS['left']: BASE + 40_000_000,
              converter.CAN_STATES['left']: BASE + 40_000_000}
    row = {'split': 'train', 'episode_id': 'x', 'frame_index': 0,
           'target_timestamp_ns': BASE + 100_000_000, 'header_timestamp_ns': stamps,
           'alignment': {'policy': converter.ALIGNMENT_POLICY,
                         'reference_timestamp_ns': BASE + 60_000_000, 'grid_lag_ns': 40_000_000},
           'source_can_stamps_ns': {t: [stamps[t]] * 4
                                    for t in [converter.CAN_ACTIONS['left'], converter.CAN_STATES['left']]}}
    file = tmp_path / 'sources.jsonl'
    def check():
        file.write_text(json.dumps(row) + '\n')
        return check_source_timestamps(file, [{'split': 'train', 'episode_id': 'x', 'frames': 1}],
                                       'LEFT', 'can-vector', 30_000_000, 100_000_000,
                                       converter.ALIGNMENT_POLICY)
    assert check()['train', 'x'] == [BASE + 100_000_000]
    stamps[converter.CAN_ACTIONS['left']] = BASE + 65_000_000
    with pytest.raises(ValueError, match='未来来源'):
        check()
    stamps[converter.CAN_ACTIONS['left']] = BASE + 40_000_000
    row['target_timestamp_ns'] += 80_000_000
    row['alignment']['grid_lag_ns'] += 80_000_000
    with pytest.raises(ValueError, match='RGB 或状态超窗'):
        check()
    row['target_timestamp_ns'] -= 80_000_000
    row['alignment'] = None
    with pytest.raises(ValueError, match='缺少图像组对齐依据'):
        check()
