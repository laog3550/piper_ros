"""停止后只读验证完成帧，复用转换器的对齐、切段和 16 步规则。"""

import heapq
from pathlib import Path
import sqlite3
import struct
from types import SimpleNamespace


class RosSqliteReader:
    """Adapt stopped SQLite bags to the converter using native ROS decoding."""

    def __init__(self, path):
        self.path = Path(path)
        self.databases = []
        self.connections = []

    def __enter__(self):
        try:
            for index, path in enumerate(sorted(self.path.glob('*.db3'))):
                db = sqlite3.connect(path.resolve().as_uri() + '?mode=ro&immutable=1', uri=True)
                self.databases.append(db)
                for tid, topic, msgtype, serialization in db.execute(
                        'SELECT id,name,type,serialization_format FROM topics'):
                    self.connections.append(SimpleNamespace(
                        id=tid, database=index, topic=topic, msgtype=msgtype,
                        ext=SimpleNamespace(serialization_format=serialization)))
            return self
        except BaseException:
            self.__exit__(None, None, None)
            raise

    def messages(self, connections):
        def rows(index, db):
            lookup = {c.id: c for c in connections if c.database == index}
            if not lookup:
                return
            placeholders = ','.join('?' for _ in lookup)
            query = ('SELECT topic_id,timestamp,data FROM messages '
                     f'WHERE topic_id IN ({placeholders}) ORDER BY timestamp,id')
            for tid, timestamp, blob in db.execute(query, tuple(lookup)):
                yield lookup[tid], timestamp, blob
        yield from heapq.merge(*(rows(index, db) for index, db in enumerate(self.databases)),
                               key=lambda row: row[1])

    def __exit__(self, *_):
        for db in self.databases:
            db.close()


def native_deserializer():
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    types = {}
    def deserialize(blob, msgtype):
        if msgtype not in types:
            types[msgtype] = get_message(msgtype)
        return deserialize_message(blob, types[msgtype])
    return deserialize


def supports_source_validation(metadata):
    mode = metadata.get('task_metadata', {}).get('arm_mode')
    arms = ('left', 'right') if mode == 'BIMANUAL' else (mode.lower(),) if mode in ('LEFT', 'RIGHT') else ()
    action_source = metadata.get('task_metadata', {}).get('action_source', 'master_target')
    kinds = (('slave_feedback',) if action_source == 'follower_next_state'
             else ('master_target', 'slave_feedback'))
    return metadata.get('mode') == 'skill' and bool(arms) and all(
                              f'/piper_vector/{arm}/{kind}' in metadata.get('topics', [])
                              for arm in arms for kind in kinds)


class CompletionValidationError(ValueError):
    """Keep failed candidate measurements inspectable without qualifying it."""

    def __init__(self, reason, details):
        super().__init__('完成帧来源校验未通过，须复核或重采：' + reason)
        self.details = details


def validate_completion(path, metadata, topic, boundary_ns, *, strictly_before=False, exact_ns=None,
                        max_camera_skew_ms=30, max_source_skew_ms=30,
                        reader_factory=RosSqliteReader, deserialize=None, candidate_topics=()):
    from to_lerobot import (ALIGNMENT_POLICY, CAMERAS, CAN_ACTIONS, CAN_STATES, FOLLOWER_ALIGNMENT_POLICY,
                            read_episode, select_sft_frames)
    mode = metadata.get('task_metadata', {}).get('arm_mode')
    arms = ['left', 'right'] if mode == 'BIMANUAL' else [mode.lower()]
    action_source = metadata.get('task_metadata', {}).get('action_source', 'master_target')
    action_topics = CAN_STATES if action_source == 'follower_next_state' else CAN_ACTIONS
    topics = [topic]
    if candidate_topics and exact_ns is None:
        relevant = {CAMERAS['top_rgb'], *(CAMERAS[f'{arm}_wrist_rgb'] for arm in arms)}
        topics = sorted(set(candidate_topics) & relevant)
        if not topics:
            raise CompletionValidationError('没有配置执行侧与顶部彩色相机话题', {})
    edge = exact_ns if exact_ns is not None else boundary_ns - int(strictly_before)
    first = exact_ns if exact_ns is not None else boundary_ns - 100_000_000
    snapshot = {**metadata, 'state': 'saved'}
    validated, frames = read_episode(
        Path(path), arms, 10, 100_000_000, 2_000_000,
        input_format='can-vector', max_can_skew_ns=30_000_000,
        max_camera_gap_ns=100_000_000,
        max_camera_skew_ns=int(max_camera_skew_ms * 1_000_000),
        max_source_skew_ns=int(max_source_skew_ms * 1_000_000), extract_valid_segments=True,
        require_training_contract=False, extra_frame_topics=tuple(topics),
        action_source=action_source,
        _metadata_override=snapshot, _reader_factory=reader_factory,
        _deserialize=deserialize or native_deserializer(), _raw_signed_gripper=True,
        completion_header_range=(first, edge))
    ready = metadata.get('ready_at_unix_ns', metadata['started_at_unix_ns'])
    candidates = sorted(((stamp, source_topic) for source_topic in topics
                         for stamp in validated['_camera_header_stamps'].get(source_topic, ())
                         if max(ready, first) <= stamp <= edge), reverse=True)
    errors = []
    failed_details = []
    for timestamp, topic in candidates:
        segment = next((s for s in validated['_quality']['segments']
                        if s['valid_start_ns'] <= timestamp <= s['valid_stop_ns']), None)
        before = [f for f in frames if segment is not None
                  and f['__segment_index__'] == segment['segment_index']
                  and f['__target_timestamp_ns__'] <= timestamp
                  and max((f.get('__action_source_header_ns__')
                           or f['__source_header_ns__'])[action_topics[arm]]
                          for arm in arms) <= timestamp]
        failed_details.append({'header_stamp_ns': timestamp, 'source_topic': topic,
                               'segment_index': segment['segment_index'] if segment else None,
                               'segment_frames': segment['frames'] if segment else 0,
                               'contiguous_frames': len(before),
                               'window_count': max(0, len(before) - 15),
                               'required_frames': 16,
                               'gripper_value_space': 'raw_signed_m_pending_calibration'})
        if exact_ns is None:
            # Do not roll back across a source interruption to an earlier skill
            # phase merely because that earlier segment has enough frames.
            if not any(begin <= timestamp <= edge <= end
                       for begin, end in validated['_continuous_source_domains']):
                errors.append('候选完成帧与结束边界之间存在来源中断')
                continue
        try:
            _, coverage = select_sft_frames(validated, frames,
                                           {'completion_frame': {'source_topic': topic,
                                                                 'header_stamp_ns': timestamp}},
                                           arms, 'can-vector')
        except ValueError as error:
            detail = validated.get('_completion_errors', {}).get((topic, timestamp))
            errors.append(detail or str(error))
            continue
        return {'header_stamp_ns': timestamp, 'source_topic': topic,
                'alignment_policy': (FOLLOWER_ALIGNMENT_POLICY if action_source == 'follower_next_state'
                                     else ALIGNMENT_POLICY), 'fps': 10,
                'gripper_value_space': 'raw_signed_m_pending_calibration',
                'boundary_ns': boundary_ns, 'selected_lookback_ns': boundary_ns - timestamp,
                'max_age_ms': 100,
                'max_camera_skew_ms': max_camera_skew_ms,
                'max_source_skew_ms': max_source_skew_ms,
                'selection_policy': ('latest_valid_causal_camera_bundle_v1' if candidate_topics
                                     else 'exact_or_single_camera_v1'),
                'camera_bundle': validated['_completion_proofs'].get((topic, timestamp)),
                'segment_index': coverage['segment_index'],
                'contiguous_frames': coverage['annotated_frames'],
                'window_count': coverage['window_count']}
    reason = errors[0] if errors else '边界前 100 ms 内没有可用的实际相机 Header'
    raise CompletionValidationError(reason, failed_details[0] if failed_details else {})


def header_receive_ns(bag_dir, topic, header_ns):
    for path in sorted(Path(bag_dir).glob('*.db3')):
        with sqlite3.connect(path.resolve().as_uri() + '?mode=ro&immutable=1', uri=True) as db:
            for received, prefix in db.execute(
                    'SELECT messages.timestamp,substr(messages.data,1,12) FROM messages '
                    'JOIN topics ON topics.id=messages.topic_id WHERE topics.name=?', (topic,)):
                if len(prefix) >= 12 and prefix[:1] == b'\0' and prefix[1] in (0, 1):
                    sec, ns = struct.unpack_from('<iI' if prefix[1] == 1 else '>iI', prefix, 4)
                    if sec * 1_000_000_000 + ns == header_ns:
                        return received
    raise ValueError('完成帧 Header 未在实际图像中找到')


def recorded_image_preview(bag_dir, topic, header_ns, *, deserialize=None):
    """Read one exact recorded image on demand; never attach pixels to status."""
    import base64
    decode = deserialize or native_deserializer()
    received = header_receive_ns(bag_dir, topic, header_ns)
    for path in sorted(Path(bag_dir).glob('*.db3')):
        with sqlite3.connect(path.resolve().as_uri() + '?mode=ro&immutable=1', uri=True) as db:
            rows = db.execute(
                'SELECT topics.type,messages.data FROM messages JOIN topics '
                'ON topics.id=messages.topic_id WHERE topics.name=? AND messages.timestamp=?',
                (topic, received))
            for msgtype, raw in rows:
                if msgtype != 'sensor_msgs/msg/Image':
                    raise ValueError('完成帧必须为原始 Image')
                message = decode(raw, msgtype)
                if message.header.stamp.sec * 10**9 + message.header.stamp.nanosec != header_ns:
                    continue
                if (message.encoding != 'rgb8' or message.width <= 0 or message.height <= 0
                        or message.step < message.width * 3
                        or len(message.data) != message.step * message.height):
                    raise ValueError('完成图像不是有效的 rgb8 图像')
                return {'source_topic': topic, 'header_stamp_ns': header_ns,
                        'width': message.width, 'height': message.height,
                        'step': message.step, 'encoding': message.encoding,
                        'data_base64': base64.b64encode(bytes(message.data)).decode('ascii')}
    raise ValueError('找不到对应的已录制完成图像')
