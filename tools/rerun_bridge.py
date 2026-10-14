#!/usr/bin/env python3
"""只读显示 ROS 2 episode；运行: python3 tools/rerun_bridge.py --config bridge.json live|replay BAG。"""

import argparse
import json
import math
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np
import rerun as rr


MESSAGE_TYPES = {
    "Image": "sensor_msgs/msg/Image",
    "CompressedImage": "sensor_msgs/msg/CompressedImage",
    "JointState": "sensor_msgs/msg/JointState",
    "Float64MultiArray": "std_msgs/msg/Float64MultiArray",
    "String": "std_msgs/msg/String",
}


def load_config(path):
    config = json.loads(Path(path).read_text(encoding="utf-8"))
    channels = {}
    cameras = config.get("cameras", {})
    if len(cameras) != 3:
        raise ValueError("cameras 必须配置三路相机")
    for name, camera in cameras.items():
        for kind in ("rgb", "depth"):
            channels[f"camera/{name}/{kind}"] = camera[kind]
    for arm in ("left", "right"):
        for kind in ("target", "actual"):
            channels[f"arm/{arm}/{kind}"] = config["arms"][arm][kind]
    channels["episode/events"] = config["events"]
    topics = {}
    for channel, spec in channels.items():
        topic, kind = spec["topic"], spec["type"]
        if not topic.startswith("/") or kind not in MESSAGE_TYPES:
            raise ValueError(f"无效话题或消息类型: {channel}")
        if topic in topics:
            raise ValueError(f"话题重复映射: {topic}")
        if channel.endswith("/depth") and kind != "Image":
            raise ValueError("深度流须使用 sensor_msgs/Image")
        if channel.endswith("/rgb") and kind not in ("Image", "CompressedImage"):
            raise ValueError("RGB 流须使用 Image 或 CompressedImage")
        if channel.startswith("arm/") and kind not in ("JointState", "Float64MultiArray"):
            raise ValueError("机械臂曲线须使用 JointState 或 Float64MultiArray")
        if channel == "episode/events" and kind != "String":
            raise ValueError("事件须使用 std_msgs/String")
        topics[topic] = (channel, spec)
    config["image_hz"] = float(config.get("image_hz", 10))
    config["joint_hz"] = float(config.get("joint_hz", 30))
    if config["image_hz"] <= 0 or config["joint_hz"] <= 0:
        raise ValueError("显示频率须大于零")
    return config, topics


def stamp_ns(msg, fallback_ns):
    stamp = getattr(getattr(msg, "header", None), "stamp", None)
    if stamp is None or (stamp.sec == 0 and stamp.nanosec == 0):
        return fallback_ns
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def raw_image(msg):
    encoding = msg.encoding.lower()
    formats = {
        "rgb8": (np.uint8, 3), "bgr8": (np.uint8, 3),
        "rgba8": (np.uint8, 4), "bgra8": (np.uint8, 4),
        "mono8": (np.uint8, 1), "16uc1": (np.uint16, 1),
        "32fc1": (np.float32, 1),
    }
    if encoding not in formats:
        raise ValueError(f"不支持图像编码 {msg.encoding}")
    dtype, channels = formats[encoding]
    dtype = np.dtype(dtype).newbyteorder(">" if msg.is_bigendian else "<")
    width_bytes = msg.width * channels * dtype.itemsize
    if msg.step < width_bytes or len(msg.data) < msg.step * msg.height:
        raise ValueError("图像尺寸或步长无效")
    pixels = np.ndarray((msg.height, msg.width, channels), dtype=dtype,
                        buffer=bytes(msg.data), strides=(msg.step, channels * dtype.itemsize, dtype.itemsize))
    if channels == 1:
        pixels = pixels[:, :, 0]
    if encoding in ("bgr8", "bgra8"):
        pixels = pixels[..., [2, 1, 0] + ([3] if channels == 4 else [])]
    return np.ascontiguousarray(pixels.astype(pixels.dtype.newbyteorder("="), copy=False)), encoding


def joint_values(msg, spec):
    values = msg.position if spec["type"] == "JointState" else msg.data
    names = spec.get("joint_names") or list(getattr(msg, "name", []))
    if not names:
        names = [f"joint_{i + 1}" for i in range(len(values))]
    if len(names) != len(values):
        raise ValueError("joint_names 与数值长度不一致")
    return [(str(name).replace("/", "_"), float(value)) for name, value in zip(names, values)
            if math.isfinite(float(value))]


class Bridge:
    def __init__(self, config, topics):
        self.config = config
        self.topics = topics
        self.last_ns = {}
        self.latest = {}
        self.events = deque()
        self.lock = threading.Lock()
        self.active = True

    def accept(self, topic, msg, receive_ns=None, buffered=False):
        channel, spec = self.topics[topic]
        when = stamp_ns(msg, receive_ns or time.time_ns())
        rate = self.config["image_hz"] if channel.startswith("camera/") else self.config["joint_hz"]
        if channel != "episode/events":
            last = self.last_ns.get(channel)
            if last is not None and 0 <= when - last < 1_000_000_000 / rate:
                return
            self.last_ns[channel] = when
        item = (channel, spec, msg, when)
        if buffered:
            with self.lock:
                if channel == "episode/events":
                    self.events.append(item)
                else:
                    self.latest[channel] = item
        else:
            self.log(item)

    def flush(self):
        with self.lock:
            items = list(self.events) + list(self.latest.values())
            self.events.clear()
            self.latest.clear()
        for item in sorted(items, key=lambda entry: entry[3]):
            self.log(item)

    def log(self, item):
        channel, spec, msg, when = item
        try:
            rr.set_time("ros_time", timestamp=np.datetime64(when, "ns"))
            if channel.startswith("camera/"):
                if spec["type"] == "CompressedImage":
                    rr.log(channel, rr.EncodedImage(contents=bytes(msg.data)))
                else:
                    pixels, encoding = raw_image(msg)
                    if channel.endswith("/depth"):
                        rr.log(channel, rr.DepthImage(pixels, meter=float(spec.get("meter", 1000 if encoding == "16uc1" else 1))))
                    else:
                        rr.log(channel, rr.Image(pixels))
            elif channel.startswith("arm/"):
                for name, value in joint_values(msg, spec):
                    rr.log(f"{channel}/{name}", rr.Scalars(value))
            else:
                rr.log(channel, rr.TextLog(str(msg.data)))
        except Exception as exc:
            print(f"Rerun 跳过 {channel}: {exc}", flush=True)


def live(bridge):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rosidl_runtime_py.utilities import get_message

    rclpy.init()
    node = Node("rerun_bridge")
    image_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                           durability=DurabilityPolicy.VOLATILE)
    event_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.VOLATILE)
    for topic, (channel, spec) in bridge.topics.items():
        msg_type = get_message(MESSAGE_TYPES[spec["type"]])
        node.create_subscription(msg_type, topic,
                                 lambda msg, topic=topic: bridge.accept(topic, msg, buffered=True),
                                 event_qos if channel == "episode/events" else image_qos)

    def writer():
        while bridge.active:
            bridge.flush()
            time.sleep(0.02)

    thread = threading.Thread(target=writer, daemon=True)
    thread.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.active = False
        thread.join(timeout=1)
        bridge.flush()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def replay(bridge, bag, storage_id):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=storage_id),
                rosbag2_py.ConverterOptions("", ""))
    bag_types = {entry.name: entry.type for entry in reader.get_all_topics_and_types()}
    for topic, (_, spec) in bridge.topics.items():
        if topic in bag_types and bag_types[topic] != MESSAGE_TYPES[spec["type"]]:
            raise ValueError(f"rosbag2 消息类型不匹配: {topic}: {bag_types[topic]}")
    classes = {topic: get_message(bag_types[topic]) for topic in bridge.topics if topic in bag_types}
    while reader.has_next():
        topic, payload, when = reader.read_next()
        if topic in classes:
            bridge.accept(topic, deserialize_message(payload, classes[topic]), when)


def main():
    parser = argparse.ArgumentParser(description="只读 ROS 2 Rerun Bridge")
    parser.add_argument("--config", required=True, help="话题映射 JSON")
    parser.add_argument("--save", help="输出 .rrd；省略时打开 Viewer")
    commands = parser.add_subparsers(dest="mode", required=True)
    commands.add_parser("live")
    replay_parser = commands.add_parser("replay")
    replay_parser.add_argument("bag", help="rosbag2 episode 目录")
    replay_parser.add_argument("--storage-id", default="sqlite3", help="sqlite3 或 mcap")
    args = parser.parse_args()
    config, topics = load_config(args.config)
    rr.init("piper_episode", spawn=not args.save)
    if args.save:
        rr.save(args.save)
    bridge = Bridge(config, topics)
    if args.mode == "live":
        live(bridge)
    else:
        replay(bridge, args.bag, args.storage_id)


if __name__ == "__main__":
    main()
