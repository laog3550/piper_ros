"""用合成 ROS 图像验证有界缓存、恢复触发和标准 rosbag2 回读。"""

from collections import Counter
import json
import os
from pathlib import Path
import threading
import time

os.environ.setdefault("ROS_DOMAIN_ID", "225")

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
import rosbag2_py

from recovery_cache import BoundedRing, RecoveryCache, Sample, load_config


IMAGE_TOPIC = "/synthetic/recovery/image"
STATE_TOPIC = "/synthetic/recovery/state"


def test_ring_enforces_time_and_bytes():
    ring = BoundedRing(1, 2600)
    def item(index, size=1256):
        return Sample(IMAGE_TOPIC, b"x" * (size - 256), index, index * 100_000_000,
                      index, size)
    assert ring.push(item(1))
    assert ring.push(item(2))
    assert ring.push(item(3))
    assert ring.bytes <= 2600
    assert ring.byte_evicted[IMAGE_TOPIC] == 1
    ring.prune(2_000_000_000)
    assert ring.bytes == 0
    assert ring.time_evicted[IMAGE_TOPIC] == 2
    assert not ring.push(item(6, 3000))
    assert ring.oversize_dropped[IMAGE_TOPIC] == 1


def test_disabled_config_needs_no_ros_topics(tmp_path):
    path = tmp_path / "config.json"
    path.write_text('{"enabled": false}')
    assert load_config(path) == {"enabled": False}


def test_synthetic_recovery_bag_roundtrip(tmp_path):
    config = {"enabled": True, "output_dir": str(tmp_path / "recoveries"),
              "pre_roll_sec": 0.5, "post_roll_sec": 1.0,
              "max_buffer_bytes": 100_000, "min_free_gb": 0,
              "topics": {IMAGE_TOPIC: "sensor_msgs/msg/Image", STATE_TOPIC: "std_msgs/msg/String"}}
    path = tmp_path / "config.json"
    path.write_text(json.dumps(config))
    config = load_config(path)
    rclpy.init()
    cache = RecoveryCache(config)
    source = rclpy.create_node("synthetic_recovery_source")
    image_pub = source.create_publisher(Image, IMAGE_TOPIC, qos_profile_sensor_data)
    state_pub = source.create_publisher(String, STATE_TOPIC, 10)
    event_pub = source.create_publisher(String, "/collection/events", 10)
    executor = SingleThreadedExecutor()
    executor.add_node(cache.node)
    executor.add_node(source)
    recovery_id = "a" * 32
    busy_id = "b" * 32
    image = Image()
    image.width, image.height, image.step, image.encoding = 16, 12, 48, "rgb8"
    image.data = bytes((10, 20, 30)) * (image.width * image.height)
    published_stamps = []
    count = 0

    def pump(duration):
        nonlocal count
        end = time.monotonic() + duration
        next_frame = time.monotonic()
        while time.monotonic() < end:
            now = time.monotonic()
            if now >= next_frame:
                image.header.stamp = source.get_clock().now().to_msg()
                published_stamps.append(image.header.stamp.sec * 1_000_000_000 + image.header.stamp.nanosec)
                image_pub.publish(image)
                state_pub.publish(String(data=f"synthetic state {count}"))
                count += 1
                next_frame += 0.05
            executor.spin_once(timeout_sec=0.005)

    def event(payload):
        event_pub.publish(String(data=json.dumps(payload)))
        for _ in range(3):
            executor.spin_once(timeout_sec=0.01)

    try:
        deadline = time.monotonic() + 5
        while (image_pub.get_subscription_count() == 0 or event_pub.get_subscription_count() == 0):
            assert time.monotonic() < deadline
            executor.spin_once(timeout_sec=0.05)
        event({"event": "started", "episode_id": "synthetic_episode"})
        assert cache.active_episode == "synthetic_episode"
        pump(1.0)
        trigger = {"event": "annotation", "kind": "recovery_trigger",
                   "recovery_id": recovery_id, "skill": "pick_banana",
                   "episode_id": "synthetic_episode", "value": "异常：测试滑脱",
                   "recorded_at_unix_ns": time.time_ns()}
        event(trigger)
        assert cache.active is not None
        capture = cache.active
        event({**trigger, "recovery_id": busy_id})
        pump(1.25)
        thread = capture.thread
        thread.join(timeout=10)
        assert not thread.is_alive()
        manifest = json.loads((cache.root / f"recovery_{recovery_id}" / "manifest.json").read_text())
        assert manifest["status"] == "complete", manifest["failure_reasons"]
        assert manifest["episode_id"] == "synthetic_episode"
        assert manifest["skill"] == "pick_banana"
        assert manifest["recovery_id"] == recovery_id
        assert manifest["training_approved"] is False
        assert manifest["actual_pre_roll_sec"] >= 0.45
        assert manifest["actual_post_roll_sec"] >= 0.9
        assert all(manifest["pre_messages_per_topic"][topic] > 0 for topic in config["topics"])
        assert all(manifest["post_messages_per_topic"][topic] > 0 for topic in config["topics"])
        assert all(manifest["actual_pre_roll_sec_by_topic"][topic] >= 0.45 for topic in config["topics"])
        assert all(manifest["actual_post_roll_sec_by_topic"][topic] >= 0.9 for topic in config["topics"])
        assert manifest["buffer_at_trigger"]["buffer_bytes"] <= config["max_buffer_bytes"]
        assert manifest["buffer_at_trigger"]["time_evicted"][IMAGE_TOPIC] > 0
        assert manifest["post_queue_highwater_bytes"] <= config["max_buffer_bytes"]
        busy = json.loads((cache.root / f"recovery_{busy_id}" / "manifest.json").read_text())
        assert busy["status"] == "failed"
        assert "已有恢复片段正在写入" in busy["failure_reasons"]
        bag = cache.root / f"recovery_{recovery_id}" / "bag"
        assert (bag / "metadata.yaml").is_file()
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
                    rosbag2_py.ConverterOptions(input_serialization_format="cdr",
                                               output_serialization_format="cdr"))
        topics = Counter()
        while reader.has_next():
            topic, payload, received_ns = reader.read_next()
            assert payload and received_ns > 0
            topics[topic] += 1
        assert topics == manifest["messages_per_topic"]
        assert all(topic in topics for topic in config["topics"])
        assert manifest["first_header_stamp_ns"][IMAGE_TOPIC] in published_stamps
    finally:
        cache.close()
        executor.remove_node(source)
        source.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


def test_trigger_boundary_never_writes_one_sample_twice(tmp_path):
    config = {"enabled": True, "output_dir": str(tmp_path / "recoveries"),
              "pre_roll_sec": 0.5, "post_roll_sec": 0.4,
              "max_buffer_bytes": 10_000, "min_free_gb": 0,
              "topics": {IMAGE_TOPIC: "sensor_msgs/msg/Image"}}
    rclpy.init()
    cache = RecoveryCache(config)
    cache.active_episode = "boundary_episode"
    pushed, release_push, release_writer = threading.Event(), threading.Event(), threading.Event()
    original_push = cache.ring.push

    def paused_push(item):
        result = original_push(item)
        pushed.set()
        assert release_push.wait(3)
        return result

    cache.ring.push = paused_push
    cache._write_capture = lambda capture: release_writer.wait(3)
    image = Image()
    image.width, image.height, image.step, image.encoding = 1, 1, 3, "rgb8"
    image.data = bytes((1, 2, 3))
    image.header.stamp.sec = 1
    trigger = {"event": "annotation", "kind": "recovery_trigger",
               "recovery_id": "d" * 32, "skill": "pick_banana",
               "episode_id": "boundary_episode", "value": "异常：边界测试"}
    message_thread = threading.Thread(target=cache.on_message, args=(IMAGE_TOPIC, image))
    trigger_thread = threading.Thread(target=cache.trigger, args=(trigger,))
    try:
        message_thread.start()
        assert pushed.wait(3)
        trigger_thread.start()
        time.sleep(0.05)
        assert trigger_thread.is_alive()
        release_push.set()
        message_thread.join(timeout=3)
        trigger_thread.join(timeout=3)
        assert not message_thread.is_alive() and not trigger_thread.is_alive()
        capture = cache.active
        assert capture is not None
        assert len(capture.pre) == 1
        assert len(capture.pending) == 0
    finally:
        release_push.set()
        release_writer.set()
        if cache.active and cache.active.thread:
            cache.active.thread.join(timeout=3)
        cache.close()
        rclpy.shutdown()


def test_disk_threshold_drop_is_failed(tmp_path):
    config = {"enabled": True, "output_dir": str(tmp_path / "recoveries"),
              "pre_roll_sec": 0.1, "post_roll_sec": 0.1,
              "max_buffer_bytes": 10_000, "min_free_gb": 0,
              "topics": {IMAGE_TOPIC: "sensor_msgs/msg/Image"}}
    rclpy.init()
    cache = RecoveryCache(config)
    cache.active_episode = "disk_episode"
    cache.min_free_bytes = 100
    calls = [0]

    def fake_free():
        calls[0] += 1
        return 1000 if calls[0] == 1 else 0

    cache._disk_free = fake_free
    trigger = {"event": "annotation", "kind": "recovery_trigger",
               "recovery_id": "e" * 32, "skill": "pick_banana",
               "episode_id": "disk_episode", "value": "异常：磁盘测试"}
    try:
        assert cache.trigger(trigger)
        capture = cache.active
        capture.thread.join(timeout=3)
        manifest = json.loads((capture.directory / "manifest.json").read_text())
        assert manifest["status"] == "failed"
        assert "写入期间剩余磁盘空间低于配置下限" in manifest["failure_reasons"]
        assert manifest["free_bytes_before"] == 1000
        assert manifest["free_bytes_after"] == 0
    finally:
        cache.close()
        rclpy.shutdown()
