"""启动时核对真实来源与首批落盘覆盖；只读取 CDR Header，不解码图像像素。"""

import sqlite3
import struct
import time
from contextlib import closing


def header_ns(raw):
    if len(raw) < 12 or raw[0] != 0 or raw[1] not in (0, 1):
        raise ValueError('无效 CDR Header')
    sec, ns = struct.unpack_from('<iI' if raw[1] == 1 else '>iI', raw, 4)
    if sec < 0 or not 0 <= ns < 1_000_000_000 or sec * 1_000_000_000 + ns <= 0:
        raise ValueError('无效来源时间戳')
    return sec * 1_000_000_000 + ns


class StartupReadiness:
    def __init__(self, topics, vector_validator=None):
        self.topics = tuple(topics)
        self.vector_validator = vector_validator
        self.observations = {}
        self.persisted = {}
        self.last_error = '等待实际来源数据'

    def observe(self, topic, raw, *, wall_ns=None, monotonic_ns=None):
        wall_ns = time.time_ns() if wall_ns is None else wall_ns
        monotonic_ns = time.monotonic_ns() if monotonic_ns is None else monotonic_ns
        previous = self.observations.get(topic, {})
        try:
            stamp = header_ns(raw)
            max_age = 100_000_000 if topic.startswith('/piper_vector/') else 500_000_000
            if not -10_000_000 <= wall_ns - stamp <= max_age:
                raise ValueError('来源 Header 已过期或晚于主机时间')
            if self.vector_validator and topic.startswith('/piper_vector/'):
                self.vector_validator(raw)
            if stamp <= previous.get('last_header_ns', 0):
                raise ValueError('来源 Header 未前进')
        except ValueError as error:
            self.observations[topic] = {'count': 0, 'error': str(error)}
            return
        self.observations[topic] = {
            'count': previous.get('count', 0) + 1,
            'first_header_ns': previous.get('first_header_ns', stamp),
            'last_header_ns': stamp, 'received_monotonic_ns': monotonic_ns}

    def ready(self, bag_dir, *, wall_ns=None, monotonic_ns=None):
        wall_ns = time.time_ns() if wall_ns is None else wall_ns
        monotonic_ns = time.monotonic_ns() if monotonic_ns is None else monotonic_ns
        pending = []
        for topic in self.topics:
            row = self.observations.get(topic, {})
            limit = 100_000_000 if topic.startswith('/piper_vector/') else 500_000_000
            if (row.get('count', 0) < 2 or
                    not 0 <= monotonic_ns - row.get('received_monotonic_ns', 0) <= limit or
                    not -10_000_000 <= wall_ns - row.get('last_header_ns', 0) <= limit):
                pending.append(topic + '：' + row.get('error', '等待连续新鲜数据'))
        # rosbag uses a write cache. Check actual committed rows as well as
        # fresh live sources, without demanding the whole cache be flushed.
        missing = set(self.topics) - self.persisted.keys()
        if missing:
            for path in sorted(bag_dir.glob('*.db3')):
                try:
                    with closing(sqlite3.connect(path.resolve().as_uri() + '?mode=ro', uri=True,
                                                 timeout=0.02)) as db:
                        topic_ids = dict(db.execute('SELECT name,id FROM topics'))
                        for topic in sorted(missing):
                            if topic not in topic_ids:
                                continue
                            rows = db.execute(
                                'SELECT timestamp,substr(data,1,12) FROM messages '
                                'WHERE topic_id=? ORDER BY timestamp LIMIT 2', (topic_ids[topic],)).fetchall()
                            if len(rows) != 2:
                                continue
                            stamps = [header_ns(raw) for _, raw in rows]
                            if stamps[1] <= stamps[0]:
                                continue
                            self.persisted[topic] = {
                                'first_header_ns': stamps[0], 'second_header_ns': stamps[1],
                                'first_receive_ns': rows[0][0], 'second_receive_ns': rows[1][0]}
                except (sqlite3.Error, ValueError):
                    continue
            missing = set(self.topics) - self.persisted.keys()
        pending.extend(topic + '：等待首批数据落盘' for topic in sorted(missing))
        self.last_error = '；'.join(pending)
        return not pending

    def evidence(self):
        return {'policy': 'live_fresh_sources_and_committed_preroll_v1',
                'live_sources': self.observations, 'recorded_sources': self.persisted,
                'master_freshness_required': False}
