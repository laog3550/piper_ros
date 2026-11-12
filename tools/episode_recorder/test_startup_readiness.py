"""验证启动门禁拒绝空发布者、旧帧、重复时间戳和未落盘来源。"""
import sqlite3
import struct
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).parent))
from startup_readiness import StartupReadiness

BASE = 1_800_000_000_000_000_000
IMAGE = '/camera_top/D455/color/image_raw'
VECTOR = '/piper_vector/left/slave_feedback'


def cdr(stamp):
    return b'\x00\x01\x00\x00' + struct.pack('<iI', *divmod(stamp, 10**9))


def bag(tmp_path, rows):
    directory = tmp_path / 'bag'
    directory.mkdir(exist_ok=True)
    with sqlite3.connect(directory / 'bag_0.db3') as db:
        db.execute('CREATE TABLE IF NOT EXISTS topics(id INTEGER PRIMARY KEY,name TEXT)')
        db.execute('CREATE TABLE IF NOT EXISTS messages(timestamp INTEGER,topic_id INTEGER,data BLOB)')
        db.executemany('INSERT OR IGNORE INTO topics VALUES(?,?)', [(1, IMAGE), (2, VECTOR)])
        db.executemany('INSERT INTO messages VALUES(?,?,?)', rows)
    return directory


def feed(monitor):
    for topic in monitor.topics:
        for i in (0, 1):
            monitor.observe(topic, cdr(BASE + i * 20_000_000),
                            wall_ns=BASE + 20_000_000, monotonic_ns=100 + i)


def test_fresh_live_messages_are_insufficient_without_committed_bag(tmp_path):
    monitor = StartupReadiness([IMAGE, VECTOR]);feed(monitor)
    directory = bag(tmp_path, [])
    assert not monitor.ready(directory, wall_ns=BASE + 30_000_000, monotonic_ns=110)
    assert '落盘' in monitor.last_error
    bag(tmp_path, [(BASE+i*20_000_000, tid, cdr(BASE+i*20_000_000)) for tid in (1,2) for i in (0,1)])
    assert monitor.ready(directory, wall_ns=BASE + 30_000_000, monotonic_ns=110)
    assert not monitor.evidence()['master_freshness_required']
    assert len(monitor.persisted) == 2


def test_committed_rows_do_not_hide_stopped_or_old_live_sources(tmp_path):
    monitor = StartupReadiness([IMAGE]);feed(monitor)
    directory = bag(tmp_path, [(BASE+i*20_000_000, 1, cdr(BASE+i*20_000_000)) for i in (0,1)])
    assert not monitor.ready(directory, wall_ns=BASE + 2_000_000_000, monotonic_ns=2_000_000_000)
    monitor.observe(IMAGE, cdr(BASE), wall_ns=BASE + 2_000_000_000, monotonic_ns=2_000_000_001)
    assert monitor.observations[IMAGE]['count'] == 0
    assert not monitor.ready(directory, wall_ns=BASE + 2_000_000_000, monotonic_ns=2_000_000_002)


def test_repeated_or_invalid_feedback_cannot_establish_readiness():
    monitor = StartupReadiness([VECTOR]);feed(monitor)
    monitor.observe(VECTOR, cdr(BASE + 20_000_000), wall_ns=BASE + 30_000_000, monotonic_ns=110)
    assert monitor.observations[VECTOR]['count'] == 0
    def invalid(_):
        raise ValueError('反馈无效')
    monitor = StartupReadiness([VECTOR], invalid);feed(monitor)
    assert monitor.observations[VECTOR]['count'] == 0
    assert monitor.observations[VECTOR]['error'] == '反馈无效'
