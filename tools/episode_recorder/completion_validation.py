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
    return metadata.get('mode') == 'skill' and bool(arms) and all(
                              f'/piper_vector/{arm}/{kind}' in metadata.get('topics', [])
                              for arm in arms for kind in ('master_target', 'slave_feedback'))


def validate_completion(path, metadata, topic, boundary_ns, *, strictly_before=False, exact_ns=None,
                        reader_factory=RosSqliteReader, deserialize=None):
    from to_lerobot import ALIGNMENT_POLICY, read_episode, select_sft_frames
    mode = metadata.get('task_metadata', {}).get('arm_mode')
    arms = ['left', 'right'] if mode == 'BIMANUAL' else [mode.lower()]
    snapshot = {**metadata, 'state': 'saved'}
    validated, frames = read_episode(
        Path(path), arms, 10, 100_000_000, 2_000_000,
        input_format='can-vector', max_can_skew_ns=30_000_000,
        max_camera_gap_ns=100_000_000, max_camera_skew_ns=30_000_000,
        max_source_skew_ns=30_000_000, extract_valid_segments=True,
        require_training_contract=False, extra_frame_topics=(topic,),
        _metadata_override=snapshot, _reader_factory=reader_factory,
        _deserialize=deserialize or native_deserializer())
    headers = validated['_camera_header_stamps'].get(topic, [])
    ready = metadata.get('ready_at_unix_ns', metadata['started_at_unix_ns'])
    if exact_ns is not None:
        candidates = [exact_ns] if exact_ns in headers else []
    else:
        candidates = [stamp for stamp in reversed(headers)
                      if ready <= stamp and 0 <= boundary_ns - stamp <= 100_000_000
                      and (stamp < boundary_ns if strictly_before else stamp <= boundary_ns)]
    errors = []
    for timestamp in candidates:
        if exact_ns is None:
            # Do not roll back across a source interruption to an earlier skill
            # phase merely because that earlier segment has enough frames.
            edge = boundary_ns - 1 if strictly_before else boundary_ns
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
                'alignment_policy': ALIGNMENT_POLICY, 'fps': 10,
                'boundary_ns': boundary_ns, 'selected_lookback_ns': boundary_ns - timestamp,
                'max_age_ms': 100, 'max_source_skew_ms': 30,
                'segment_index': coverage['segment_index'],
                'contiguous_frames': coverage['annotated_frames'],
                'window_count': coverage['window_count']}
    reason = errors[0] if errors else '边界前 100 ms 内没有可用的实际相机 Header'
    raise ValueError('完成帧来源校验未通过，须复核或重采：' + reason)


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
