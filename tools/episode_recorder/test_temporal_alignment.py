"""验证异相位 RGB 对齐的因果性、固定网格和完成帧的失效筛选。"""

import json
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest
from rosbags.rosbag2 import Writer

sys.path.insert(0, str(Path(__file__).parent))
import to_lerobot as converter
from completion_validation import RosSqliteReader, validate_completion

BASE = 1_800_000_100_000_000_000


def phased_episode(tmp_path, invalid=None, stop_index=None, transport_fault=None, raw_gripper=.01,
                   top_phase=25_000_000, wrist_phase=0, receive_delay_ns=1000):
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
    if transport_fault is not None:
        topics['/piper_direct/left/diagnostics'] = 'std_msgs/msg/String'
    meta['topics'] = list(topics)
    meta['frame_topics'] = list(converter.CAMERAS.values())[:2]
    (path / 'episode.json').write_text(json.dumps(meta), encoding='utf-8')
    events = []
    if transport_fault is not None:
        counters = {'overflow': 0, 'publish_errors': 0, 'shutdown_discarded': 0}
        target = dict(counters)
        if transport_fault in target:
            target[transport_fault] = 1
        data = {'schema_version': 'piper_direct_can_diagnostics/1', 'stamp_ns': BASE + 1_000_000_000,
                'channel': 'can_left', 'capture_failed': transport_fault == 'capture_failed',
                'streams': {'master_target': target, 'slave_feedback': counters}}
        events.append((BASE + 1_000_000_000, '/piper_direct/left/diagnostics',
                       store.types['std_msgs/msg/String'](json.dumps(data))))
    for i in range(176):
        t = BASE + i * 20_000_000
        for topic in list(topics)[:2]:
            kind = topic.rsplit('/', 1)[-1]
            paused = kind == 'master_target' and stop_index is not None and i >= stop_index
            source_t = BASE + (stop_index - 1) * 20_000_000 if paused else t
            bad = paused or bool(invalid and kind == 'master_target' and invalid(i))
            msg = Vector(Header(stamp(source_t), 'can_left'), f'/piper_direct/left/{kind}',
                         converter.JOINTS, np.array([(source_t - BASE) / 1e9] * 6 + [raw_gripper]),
                         np.array(converter.CAN_IDS[kind], dtype=np.uint32),
                         [stamp(source_t - n * 1_000_000) for n in (3, 2, 1, 0)],
                         np.zeros(4, dtype=np.bool_), np.full(4, bad, dtype=np.bool_),
                         False, not bad, True)
            events.append((t, topic, msg))
    for topic, phase in [(converter.CAMERAS['top_rgb'], top_phase),
                         (converter.CAMERAS['left_wrist_rgb'], wrist_phase)]:
        for i in range(53):
            t = BASE + phase + round(i * 1_000_000_000 / 15)
            msg = Image(Header(stamp(t), 'synthetic_camera'), 480, 640, 'rgb8', 0, 1920,
                        np.zeros(480 * 640 * 3, dtype=np.uint8))
            events.append((t, topic, msg))
    with Writer(path / 'bag', version=Writer.VERSION_LATEST) as writer:
        connections = {t: writer.add_connection(t, typ, typestore=store) for t, typ in topics.items()}
        for t, topic, msg in sorted(events, key=lambda row: row[0]):
            writer.write(connections[topic], t + receive_delay_ns, store.serialize_cdr(msg, topics[topic]))
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


def test_clean_terminal_pause_keeps_qualified_pre_stale_completion(tmp_path):
    path, meta = phased_episode(tmp_path, stop_index=170)
    before = converter.source_digest(path)
    boundary = BASE + 169 * 20_000_000
    result = validate_completion(path, meta, converter.CAMERAS['top_rgb'], boundary,
                                 strictly_before=True, deserialize=converter.TYPESTORE.deserialize_cdr)
    assert 0 < boundary - result['header_stamp_ns'] <= 100_000_000
    assert result['contiguous_frames'] >= 16
    assert converter.source_digest(path) == before


@pytest.mark.parametrize('fault', ['overflow', 'publish_errors', 'shutdown_discarded', 'capture_failed'])
def test_transport_loss_diagnostics_cannot_be_filtered_into_valid_sft(tmp_path, fault):
    path, _ = phased_episode(tmp_path, transport_fault=fault)
    with pytest.raises(ValueError, match='拒绝不完整来源|标记采集失败'):
        read(path)


def test_healthy_transport_diagnostics_are_kept_with_source_quality(tmp_path):
    path, _ = phased_episode(tmp_path, transport_fault='healthy')
    meta, frames = read(path)
    assert len(frames) >= 30
    diagnostics = meta['_quality']['transport_diagnostics']['/piper_direct/left/diagnostics']
    assert diagnostics['observations'] == 1 and not diagnostics['last']['capture_failed']


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


@pytest.mark.parametrize('invalid_tail', [False, True])
def test_recorder_filters_auto_and_manual_completion_with_shared_validator(tmp_path, monkeypatch, invalid_tail):
    from test_recorder_annotations import load_recorder_module
    path, meta = phased_episode(tmp_path, invalid=(lambda i: i >= 140) if invalid_tail else None)
    module = load_recorder_module()
    recorder = module.Recorder.__new__(module.Recorder)
    recorder.current = path
    recorder.metadata = {**meta, 'state': 'recording', 'annotations': [],
                         'required_topics': meta['topics'], 'active_skill': converter.SKILLS[0]}
    recorder.topics, recorder.frame_topics = meta['topics'], meta['frame_topics']
    recorder.expected_fps = {}
    recorder.ready_monotonic_ns = None
    recorder.process = SimpleNamespace(poll=lambda: None)
    recorder._sample_clock = lambda: None
    recorder._event = lambda *a, **kw: None
    recorder._terminate = lambda: setattr(recorder, 'process', None)
    recorder.status = lambda: {'state': recorder.metadata['state']}
    recorder.event_publisher = SimpleNamespace(publish=lambda message: None)
    monkeypatch.setattr(module.time, 'time_ns', lambda: BASE + 3_520_000_000)
    monkeypatch.setattr(module.time, 'sleep', lambda _: None)
    monkeypatch.setattr(module, 'inspect_bag', lambda *a: {})
    monkeypatch.setattr(module, 'terminal_stale_master_target', lambda *a: None)
    monkeypatch.setattr(module, 'last_valid_master_target', lambda *a: (BASE + 3_500_000_000, BASE + 3_500_001_000))
    monkeypatch.setattr(module, 'validate_completion', lambda *a, **kw:
                        validate_completion(*a, **kw, deserialize=converter.TYPESTORE.deserialize_cdr))
    assert recorder.stop(reason='duration_limit')['state'] == 'stopped'
    candidate = recorder.metadata['auto_completion_candidate']
    assert recorder.metadata['annotations'] == []
    assert recorder.metadata['completion_review_required'] is True
    if invalid_tail:
        assert candidate.get('source_validation_error') or candidate.get('source_validation') is None
        with pytest.raises(ValueError, match='唯一且已在 rosbag 核验'):
            module.require_verified_skill_completion(recorder.metadata, 'success')
        recorder.annotate({'kind': 'skill_complete', 'value': converter.SKILLS[0],
                           'review_source_topic': candidate['source_topic'],
                           'header_stamp_ns': candidate['header_stamp_ns'],
                           'candidate': True})
        complete, = recorder.metadata['annotations']
        assert complete['source_validation_error']
        assert complete['annotation_origin'] == 'operator_confirmed_candidate'
        with pytest.raises(ValueError, match='完成帧须通过来源时效'):
            module.require_verified_skill_completion(recorder.metadata, 'success')
    else:
        assert candidate['source_validation']['contiguous_frames'] >= 16
        recorder.annotate({'kind': 'skill_complete', 'value': converter.SKILLS[0],
                           'review_source_topic': candidate['source_topic'],
                           'header_stamp_ns': candidate['header_stamp_ns'],
                           'candidate': True})
        complete, = recorder.metadata['annotations']
        assert complete['source_validation']['contiguous_frames'] >= 16
        assert complete['verified_in_bag'] is True
        module.require_verified_skill_completion(recorder.metadata, 'success')


@pytest.mark.parametrize('invalid_tail', [False, True])
def test_manual_completion_survives_stop_and_blocks_later_candidate(tmp_path, monkeypatch, invalid_tail):
    from test_recorder_annotations import load_recorder_module
    path, meta = phased_episode(tmp_path, invalid=(lambda i: i >= 140) if invalid_tail else None)
    module = load_recorder_module()
    rec = module.Recorder.__new__(module.Recorder)
    timestamp = BASE + 25_000_000 + round(50 * 1_000_000_000 / 15)
    marker = {'kind': 'skill_complete', 'value': converter.SKILLS[0],
              'skill_result': 'success', 'source_topic': converter.CAMERAS['top_rgb'],
              'header_stamp_ns': timestamp, 'recorded_at_unix_ns': timestamp + 1000}
    rec.current = path
    rec.metadata = {**meta, 'state': 'recording', 'annotations': [marker],
                    'required_topics': meta['topics']}
    rec.topics, rec.frame_topics, rec.expected_fps = meta['topics'], meta['frame_topics'], {}
    rec.ready_monotonic_ns = None
    rec.process = SimpleNamespace(poll=lambda: None)
    rec._sample_clock = lambda: None
    rec._event = lambda *a, **kw: None
    rec._terminate = lambda: setattr(rec, 'process', None)
    rec.status = lambda: {'state': rec.metadata['state']}
    monkeypatch.setattr(module.time, 'time_ns', lambda: BASE + 3_520_000_000)
    monkeypatch.setattr(module.time, 'sleep', lambda _: None)
    monkeypatch.setattr(module, 'inspect_bag', lambda *a: {})
    monkeypatch.setattr(module, 'validate_completion', lambda *a, **kw:
                        validate_completion(*a, **kw, deserialize=converter.TYPESTORE.deserialize_cdr))
    rec.stop(reason='duration_limit')
    assert rec.metadata['annotations'] == [marker]
    assert marker['header_stamp_ns'] == timestamp and marker['verified_in_bag'] is True
    assert ('source_validation_error' in marker) == invalid_tail
    assert 'auto_completion_candidate' not in rec.metadata
    status = module.assess_training_eligibility(rec.metadata, 'success')
    assert status['eligible'] is False
    assert (status['status'] == 'sft_frame_gate_passed') != invalid_tail


def test_failed_completion_reports_counts_without_qualifying_source(tmp_path):
    from completion_validation import CompletionValidationError
    path, meta = phased_episode(tmp_path, invalid=lambda i: 140 <= i < 170)
    with pytest.raises(CompletionValidationError) as caught:
        validate_completion(path, meta, converter.CAMERAS['top_rgb'], BASE + 3_450_000_000,
                            exact_ns=BASE + 3_425_000_000,
                            deserialize=converter.TYPESTORE.deserialize_cdr)
    detail = caught.value.details
    assert detail['header_stamp_ns'] == BASE + 3_425_000_000
    assert detail['contiguous_frames'] < 16 and detail['window_count'] == 0
    assert detail['required_frames'] == 16



def test_signed_raw_gripper_completion_does_not_enable_uncalibrated_export(tmp_path):
    path, meta = phased_episode(tmp_path, raw_gripper=-.0027)
    before = converter.source_digest(path)
    timestamp = BASE + 25_000_000 + round(50 * 1_000_000_000 / 15)
    proof = validate_completion(path, meta, converter.CAMERAS['top_rgb'], timestamp,
                                exact_ns=timestamp, deserialize=converter.TYPESTORE.deserialize_cdr)
    assert proof['contiguous_frames'] >= 16
    assert proof['gripper_value_space'] == 'raw_signed_m_pending_calibration'
    with pytest.raises(ValueError, match='夹爪开口为负'):
        read(path)
    with pytest.raises(ValueError, match='仅可用于'):
        converter.read_episode(path, ['left'], 10, 100_000_000, 2_000_000,
                               input_format='can-vector', max_source_skew_ns=30_000_000,
                               extract_valid_segments=True, _raw_signed_gripper=True)
    assert converter.source_digest(path) == before


@pytest.mark.parametrize('raw_gripper', [float('nan'), float('inf')])
def test_raw_completion_still_rejects_non_finite_gripper(tmp_path, raw_gripper):
    path, meta = phased_episode(tmp_path, raw_gripper=raw_gripper)
    with pytest.raises(ValueError, match='非有限'):
        validate_completion(path, meta, converter.CAMERAS['top_rgb'], BASE + 3_425_000_000,
                            exact_ns=BASE + 3_425_000_000,
                            deserialize=converter.TYPESTORE.deserialize_cdr)


def test_signed_raw_gripper_does_not_bridge_real_source_holes(tmp_path):
    path, meta = phased_episode(tmp_path, raw_gripper=-.0027, invalid=lambda i: 140 <= i < 170)
    with pytest.raises(ValueError, match='完成帧来源校验未通过'):
        validate_completion(path, meta, converter.CAMERAS['top_rgb'], BASE + 3_425_000_000,
                            exact_ns=BASE + 3_425_000_000,
                            deserialize=converter.TYPESTORE.deserialize_cdr)


def test_recheck_updates_only_validation_and_preserves_completion_event(tmp_path, monkeypatch):
    from test_recorder_annotations import load_recorder_module
    path, meta = phased_episode(tmp_path, raw_gripper=-.0027)
    module = load_recorder_module()
    stamp = BASE + 3_225_000_000
    marker = {'kind': 'skill_complete', 'value': converter.SKILLS[0],
              'source_topic': converter.CAMERAS['top_rgb'], 'header_stamp_ns': stamp,
              'reviewed_at_unix_ns': BASE + 4_000_000_000,
              'source_validation_error': '旧版本误将有符号原值判为负开口'}
    rec = module.Recorder.__new__(module.Recorder)
    rec.current = path
    rec.metadata = {**meta, 'state': 'stopped', 'annotations': [marker],
                    'stop_requested_at_unix_ns': BASE + 3_600_000_000}
    rec.frame_topics = meta['frame_topics']
    monkeypatch.setattr(module, 'validate_completion', lambda *a, **kw:
                        validate_completion(*a, **kw, deserialize=converter.TYPESTORE.deserialize_cdr))
    before = {f.name: f.read_bytes() for f in (path / 'bag').glob('*.db3')}
    rec.recheck_completion()
    assert marker['header_stamp_ns'] == stamp
    assert marker['reviewed_at_unix_ns'] == BASE + 4_000_000_000
    assert marker['source_validation']['contiguous_frames'] >= 16
    assert 'source_validation_error' not in marker
    assert marker['validation_reviews'][0]['previous_validation']['source_validation_error']
    assert before == {f.name: f.read_bytes() for f in (path / 'bag').glob('*.db3')}


def test_auto_candidate_uses_latest_valid_camera_bundle_in_one_read(tmp_path):
    path, meta = phased_episode(tmp_path, top_phase=0, wrist_phase=12_000_000)
    top, wrist = converter.CAMERAS['top_rgb'], converter.CAMERAS['left_wrist_rgb']
    boundary = BASE + 3_490_000_000
    before = converter.source_digest(path)
    with pytest.raises(ValueError, match='RGB 之间'):
        validate_completion(path, meta, top, boundary, strictly_before=True,
                            max_camera_skew_ms=40, max_source_skew_ms=40,
                            deserialize=converter.TYPESTORE.deserialize_cdr)
    calls = []
    def reader(path):
        calls.append(path)
        return RosSqliteReader(path)
    proof = validate_completion(path, meta, top, boundary, strictly_before=True,
                                candidate_topics=[top, wrist, converter.CAMERAS['right_wrist_rgb']],
                                max_camera_skew_ms=40, max_source_skew_ms=40,
                                reader_factory=reader, deserialize=converter.TYPESTORE.deserialize_cdr)
    assert len(calls) == 1
    assert proof['source_topic'] == wrist and proof['contiguous_frames'] >= 16
    assert proof['selection_policy'] == 'latest_valid_causal_camera_bundle_v1'
    bundle = proof['camera_bundle']
    assert bundle['camera_skew_ns'] == 12_000_000
    assert all(h <= proof['header_stamp_ns'] < boundary for h in bundle['source_header_stamps_ns'].values())
    assert converter.source_digest(path) == before


def test_multi_camera_candidate_still_cannot_cross_a_source_hole(tmp_path):
    path, meta = phased_episode(tmp_path, invalid=lambda i: 140 <= i < 170,
                                top_phase=0, wrist_phase=12_000_000)
    with pytest.raises(ValueError, match='完成帧来源校验未通过'):
        validate_completion(path, meta, converter.CAMERAS['top_rgb'], BASE + 3_400_000_000,
                            strictly_before=True, candidate_topics=list(converter.CAMERAS.values()),
                            max_camera_skew_ms=40, max_source_skew_ms=40,
                            deserialize=converter.TYPESTORE.deserialize_cdr)


def test_recorded_completion_preview_reads_exact_pixels_without_relabeling(tmp_path):
    import base64
    from completion_validation import recorded_image_preview
    path, meta = phased_episode(tmp_path)
    before = converter.source_digest(path)
    timestamp = BASE + 25_000_000 + round(50 * 10**9 / 15)
    preview = recorded_image_preview(path/'bag', converter.CAMERAS['top_rgb'], timestamp,
                                      deserialize=converter.TYPESTORE.deserialize_cdr)
    assert preview['header_stamp_ns'] == timestamp
    assert base64.b64decode(preview['data_base64']) == bytes(480 * 640 * 3)
    assert preview['width'] == 640 and preview['height'] == 480
    with pytest.raises(ValueError, match='Header 未在实际图像'):
        recorded_image_preview(path/'bag', converter.CAMERAS['top_rgb'], timestamp + 1,
                               deserialize=converter.TYPESTORE.deserialize_cdr)
    assert converter.source_digest(path) == before


def test_bimanual_candidate_fallback_respects_shared_trim_boundary(tmp_path, monkeypatch):
    from test_recorder_annotations import load_recorder_module
    path, meta = phased_episode(tmp_path)
    meta['task_metadata']['arm_mode'] = 'BIMANUAL'
    module = load_recorder_module();recorder = module.Recorder.__new__(module.Recorder)
    recorder.current = path;recorder.metadata = {**meta, 'state':'recording', 'annotations':[],
        'required_topics':meta['topics'], 'active_skill':converter.SKILLS[0]}
    recorder.topics = meta['topics'];recorder.frame_topics = [converter.CAMERAS['top_rgb']]
    recorder.expected_fps = {};recorder.ready_monotonic_ns = None
    recorder.auto_trim_stale_tail = True;recorder.process = SimpleNamespace(poll=lambda:None)
    recorder._sample_clock = lambda:None;recorder._event = lambda *a,**kw:None
    recorder._terminate = lambda:setattr(recorder,'process',None)
    recorder.status = lambda:{'state':recorder.metadata['state']}
    recorder.event_publisher = SimpleNamespace(publish=lambda message:None)
    boundary = BASE + 3_380_000_000
    monkeypatch.setattr(module.time,'time_ns',lambda:BASE+3_520_000_000)
    monkeypatch.setattr(module.time,'sleep',lambda _:None)
    monkeypatch.setattr(module,'inspect_bag',lambda *a:{})
    monkeypatch.setattr(module,'terminal_stale_master_target',lambda bag,arm,*a:
        {'last_valid_header_ns':boundary+(20_000_000 if arm=='right' else 0)})
    assert recorder.stop(reason='stale_tail')['state']=='stopped'
    candidate = recorder.metadata['auto_completion_candidate']
    assert recorder.metadata['trim_boundary_ns']==boundary
    assert 0 < boundary-candidate['header_stamp_ns'] <= 100_000_000
    assert not recorder.metadata['annotations']


def test_candidate_confirmation_accepts_late_receipt_before_physical_stop(tmp_path, monkeypatch):
    from test_recorder_annotations import load_recorder_module
    path, meta = phased_episode(tmp_path, receive_delay_ns=40_000_000)
    topic = converter.CAMERAS['top_rgb'];timestamp=BASE+3_425_000_000
    logical_stop = BASE+3_440_000_000;physical_stop=BASE+3_600_000_000
    module=load_recorder_module();recorder=module.Recorder.__new__(module.Recorder)
    recorder.current=path;recorder.frame_topics=[topic,converter.CAMERAS['left_wrist_rgb']]
    recorder.metadata={**meta,'state':'stopped','active_skill':converter.SKILLS[0],'annotations':[],
        'stopped_at_unix_ns':logical_stop,'raw_stopped_at_unix_ns':physical_stop,
        'auto_completion_candidate':{'source_topic':topic,'header_stamp_ns':timestamp}}
    recorder._write_current=lambda:None
    recorder.event_publisher=SimpleNamespace(publish=lambda _:None)
    monkeypatch.setattr(module,'validate_completion',lambda *a,**kw:
        validate_completion(*a,**kw,deserialize=converter.TYPESTORE.deserialize_cdr))
    assert not module.recorded_header_exists(path/'bag',topic,timestamp,BASE,logical_stop)
    assert module.recorded_header_exists(path/'bag',topic,timestamp,BASE,logical_stop,
                                         receive_stop_ns=physical_stop)
    assert not module.recorded_header_exists(path/'bag',topic,timestamp,BASE,timestamp-1,
                                             receive_stop_ns=physical_stop)
    assert not module.recorded_header_exists(path/'bag',topic,timestamp,BASE,logical_stop,
                                             receive_stop_ns=timestamp+1)
    before=converter.source_digest(path)
    result=recorder.annotate({'kind':'skill_complete','value':converter.SKILLS[0],
        'candidate':True,'review_source_topic':topic,'header_stamp_ns':timestamp})
    annotation=result['annotation']
    assert annotation['verified_in_bag'] and annotation['header_stamp_ns']==timestamp
    assert annotation['annotation_origin']=='operator_confirmed_candidate'
    assert annotation['source_validation']['contiguous_frames']>=16
    assert 'source_validation_error' not in annotation
    assert converter.source_digest(path)==before
