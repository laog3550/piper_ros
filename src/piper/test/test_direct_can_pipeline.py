"""验证 ROS 发布阻塞时 CAN 接收继续、来源不改写、溢出明确失败。"""

import pathlib
import queue
import sys
import threading
import time
from types import SimpleNamespace
from unittest import mock

import can
import pytest
import rclpy

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from piper.piper_direct_can_node import PiperDirectCanNode  # noqa: E402
from piper.piper_arm_vector_node import PiperArmVectorNode  # noqa: E402
from rclpy.qos import ReliabilityPolicy  # noqa: E402


def wait_for(check, timeout=3):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if check():
            return
        time.sleep(.002)
    assert check(), '线程未在预期时间内完成'


def frame(cid, value):
    return can.Message(arbitration_id=cid, is_extended_id=False,
                       data=value.to_bytes(4, 'big', signed=True) + bytes(4),
                       timestamp=1_790_000_000.125 + value / 10000)


class Inbox:
    def __init__(self):
        self.frames = queue.Queue()
        self.socket = mock.Mock()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        pass

    def recv(self, timeout):
        try:
            return self.frames.get(timeout=min(timeout, .02))
        except queue.Empty:
            return None

    def send(self, *_):
        raise AssertionError('只读节点禁止发送 CAN')


@pytest.fixture
def pipeline(monkeypatch, request):
    monkeypatch.setenv('ROS_DOMAIN_ID', '178')
    depth = getattr(request, 'param', 4096)
    rclpy.init(args=['--ros-args', '-p', 'side:=left', '-p', f'receive_queue_depth:={depth}'])
    bus = Inbox()
    factory = mock.Mock(return_value=bus)
    monkeypatch.setattr(can, 'Bus', factory)
    with mock.patch.object(threading.Thread, 'start'):
        node = PiperDirectCanNode()
    releases = []
    state = SimpleNamespace(node=node, bus=bus, releases=releases, factory=factory)
    try:
        yield state
    finally:
        for release in releases:
            release.set()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def start(pipeline, block=True, fail=False):
    entered, release = threading.Event(), threading.Event()
    pipeline.releases.append(release)
    master, feedback = [], []
    def publish(message):
        entered.set()
        if fail:
            raise RuntimeError('注入发布错误')
        if block:
            assert release.wait(5), '阻塞测试未及时释放'
        master.append(message)
    pipeline.node._frame_publishers['can_left']['master_target'] = SimpleNamespace(publish=publish)
    pipeline.node._frame_publishers['can_left']['slave_feedback'] = SimpleNamespace(publish=feedback.append)
    for thread in pipeline.node._publish_threads + pipeline.node._receive_threads:
        thread.start()
    return entered, release, master, feedback


def test_blocked_master_publish_does_not_stop_can_rx_or_feedback(pipeline):
    entered, release, master, feedback = start(pipeline)
    first = frame(0x155, 1)
    pipeline.bus.frames.put(first)
    assert entered.wait(2)
    incoming = [first]
    for i in range(2, 202):
        target = frame(0x155, i)
        incoming.append(target)
        pipeline.bus.frames.put(target)
        pipeline.bus.frames.put(frame(0x2A5, i))
    def received_all():
        stats = pipeline.node.diagnostics('can_left')['streams']
        return stats['master_target']['received'] == 201 and stats['slave_feedback']['received'] == 200
    wait_for(received_all)
    wait_for(lambda: len(feedback) == 200)
    snapshot = pipeline.node.diagnostics('can_left')
    assert snapshot['streams']['master_target']['queue_size'] == 200
    assert snapshot['streams']['master_target']['published'] == 0
    assert not snapshot['capture_failed']
    release.set()
    wait_for(lambda: len(master) == 201)
    for original, emitted in zip(incoming, master):
        assert bytes(emitted.raw_data) == bytes(original.data)
        assert emitted.header.stamp.sec * 1_000_000_000 + emitted.header.stamp.nanosec == pipeline.node._header_ns(original)
        assert emitted.can_id == original.arbitration_id
    assert pipeline.node.diagnostics('can_left')['streams']['master_target']['overflow'] == 0
    assert pipeline.factory.call_count == 1


@pytest.mark.parametrize('pipeline', [2], indirect=True)
def test_queue_overflow_is_bounded_and_capture_is_marked_failed(pipeline):
    entered, release, _, _ = start(pipeline)
    pipeline.bus.frames.put(frame(0x155, 1))
    assert entered.wait(2)
    for i in range(2, 12):
        pipeline.bus.frames.put(frame(0x155, i))
    wait_for(lambda: pipeline.node.diagnostics('can_left')['streams']['master_target']['received'] == 11)
    snapshot = pipeline.node.diagnostics('can_left')
    assert snapshot['capture_failed']
    assert '队列溢出' in snapshot['error']
    assert snapshot['streams']['master_target']['queue_size'] == 2
    assert snapshot['streams']['master_target']['overflow'] == 8
    assert snapshot['streams']['master_target']['last_overflow_header_ns'] > 0
    release.set()


def test_publish_exception_propagates_to_capture_health(pipeline):
    entered, _, _, _ = start(pipeline, block=False, fail=True)
    pipeline.bus.frames.put(frame(0x155, 1))
    assert entered.wait(2)
    wait_for(lambda: pipeline.node.diagnostics('can_left')['capture_failed'])
    snapshot = pipeline.node.diagnostics('can_left')
    assert snapshot['streams']['master_target']['publish_errors'] == 1
    assert snapshot['streams']['master_target']['published'] == 0
    assert '注入发布错误' in snapshot['error']
    pipeline.node._report_health()
    assert pipeline.node._stop.is_set() and rclpy.ok()
    assert pipeline.node._fatal_shutdown_at > time.monotonic()
    pipeline.node._fatal_shutdown_at = time.monotonic() - 1
    pipeline.node._report_health()
    assert not rclpy.ok()


def test_live_input_qos_does_not_weaken_raw_archive_publisher(pipeline):
    vector = PiperArmVectorNode()
    try:
        assert vector._input_qos.reliability == ReliabilityPolicy.BEST_EFFORT
        assert vector._input_qos.depth == 32
        for publisher in pipeline.node._frame_publishers['can_left'].values():
            assert publisher.qos_profile.reliability == ReliabilityPolicy.RELIABLE
            assert publisher.qos_profile.depth == 1000
    finally:
        vector.destroy_node()


def test_interval_update_after_snapshot_is_reported_next_window(pipeline):
    node = pipeline.node
    pending = node._queues['can_left', 'master_target']
    original = pending.mutex
    class UpdateAfterSnapshot:
        injected = False
        def __enter__(self):
            if not self.injected:
                with node._stats_lock:
                    node._stats['can_left', 'master_target']['interval_max_publish_call_ms'] = 25.
                self.injected = True
            return original.__enter__()
        def __exit__(self, *args):
            return original.__exit__(*args)
    pending.mutex = UpdateAfterSnapshot()
    first = node.diagnostics('can_left')
    second = node.diagnostics('can_left')
    assert first['streams']['master_target']['interval_max_publish_call_ms'] == 0.
    assert second['streams']['master_target']['interval_max_publish_call_ms'] == 25.
