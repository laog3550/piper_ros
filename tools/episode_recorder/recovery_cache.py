#!/usr/bin/env python3
"""可选的被动 ROS 恢复片段缓存；运行：python3 recovery_cache.py --config PATH。"""

import argparse
from collections import Counter, deque
from dataclasses import dataclass, field
import fcntl
import json
import os
from pathlib import Path
import re
import shutil
import threading
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import serialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String


RECOVERY_ID = re.compile(r"[0-9a-f]{32}\Z")
EVENT_TOPIC = "/collection/events"
SKILLS = ("pick_banana", "place_banana_in_box", "close_hinged_lid_bimanual")
SAMPLE_OVERHEAD_BYTES = 256


def atomic_json(path, value):
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    os.replace(temporary, path)


def require_number(value, name, minimum):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or value < minimum:
        raise ValueError(f"{name} 必须不小于 {minimum}")
    return value


def load_config(path):
    config = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(config, dict) or not isinstance(config.get("enabled"), bool):
        raise ValueError("enabled 必须为布尔值")
    if not config["enabled"]:
        return config
    topics = config.get("topics")
    if (not isinstance(topics, dict) or not topics or EVENT_TOPIC in topics
            or any(not isinstance(name, str) or not name.startswith("/")
                   or not isinstance(kind, str) or "/msg/" not in kind
                   for name, kind in topics.items())):
        raise ValueError("topics 必须是非空的绝对话题名到 ROS 消息类型的映射，且不含事件话题")
    for name in ("pre_roll_sec", "post_roll_sec"):
        require_number(config.get(name), name, 0.1)
    require_number(config.get("max_buffer_bytes"), "max_buffer_bytes", 1024)
    require_number(config.get("min_free_gb", 1), "min_free_gb", 0)
    if not isinstance(config.get("output_dir"), str) or not config["output_dir"]:
        raise ValueError("output_dir 必须为路径")
    rates = config.get("expected_fps", {})
    if not isinstance(rates, dict) or not set(rates).issubset(topics):
        raise ValueError("expected_fps 必须是 topics 的子集")
    for rate in rates.values():
        require_number(rate, "expected_fps", 0.001)
    return config


@dataclass(slots=True)
class Sample:
    topic: str
    payload: bytes
    received_at_ns: int
    received_mono_ns: int
    header_stamp_ns: int | None
    size_bytes: int


class BoundedRing:
    def __init__(self, pre_roll_sec, max_buffer_bytes):
        self.window_ns = int(pre_roll_sec * 1_000_000_000)
        self.limit = int(max_buffer_bytes)
        self.items = deque()
        self.bytes = 0
        self.time_evicted = Counter()
        self.byte_evicted = Counter()
        self.oversize_dropped = Counter()

    def prune(self, now_mono_ns):
        cutoff = now_mono_ns - self.window_ns
        while self.items and self.items[0].received_mono_ns < cutoff:
            old = self.items.popleft()
            self.bytes -= old.size_bytes
            self.time_evicted[old.topic] += 1

    def push(self, item):
        self.prune(item.received_mono_ns)
        if item.size_bytes > self.limit:
            self.oversize_dropped[item.topic] += 1
            return False
        self.items.append(item)
        self.bytes += item.size_bytes
        while self.bytes > self.limit:
            old = self.items.popleft()
            self.bytes -= old.size_bytes
            self.byte_evicted[old.topic] += 1
        return True

    def audit(self):
        return {"buffer_bytes": self.bytes, "buffer_messages": len(self.items),
                "time_evicted": dict(self.time_evicted), "byte_evicted": dict(self.byte_evicted),
                "oversize_dropped": dict(self.oversize_dropped)}


@dataclass
class Capture:
    directory: Path
    manifest: dict
    trigger_mono_ns: int
    deadline_mono_ns: int
    pre: list[Sample]
    pending: deque = field(default_factory=deque)
    pending_bytes: int = 0
    pending_highwater_bytes: int = 0
    pending_dropped: Counter = field(default_factory=Counter)
    condition: threading.Condition = field(default_factory=threading.Condition)
    interrupted: bool = False
    thread: threading.Thread | None = None


class RecoveryCache:
    def __init__(self, config):
        self.config = config
        self.topics = dict(config["topics"])
        self.message_classes = {topic: get_message(kind) for topic, kind in self.topics.items()}
        self.root = Path(config["output_dir"]).expanduser().resolve()
        self.root.mkdir(parents=True, exist_ok=True)
        self.lock_file = (self.root / ".recovery_cache.lock").open("w")
        fcntl.flock(self.lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
        self._recover_incomplete()
        self.pre_roll_ns = int(config["pre_roll_sec"] * 1_000_000_000)
        self.post_roll_ns = int(config["post_roll_sec"] * 1_000_000_000)
        self.max_buffer_bytes = int(config["max_buffer_bytes"])
        self.min_free_bytes = int(config.get("min_free_gb", 1) * 1_000_000_000)
        self.ring = BoundedRing(config["pre_roll_sec"], self.max_buffer_bytes)
        self.active_episode = None
        self.active = None
        self.lock = threading.Lock()
        self.node = rclpy.create_node("piper_recovery_cache")
        for topic, message_class in self.message_classes.items():
            self.node.create_subscription(message_class, topic,
                                          lambda msg, name=topic: self.on_message(name, msg),
                                          qos_profile_sensor_data)
        self.node.create_subscription(String, EVENT_TOPIC, self.on_event, 10)

    def _recover_incomplete(self):
        for path in self.root.glob("recovery_*/manifest.json"):
            try:
                manifest = json.loads(path.read_text(encoding="utf-8"))
                if manifest.get("status") == "recording":
                    manifest["status"] = "failed"
                    manifest.setdefault("failure_reasons", []).append("缓存进程异常退出，片段未完成")
                    atomic_json(path, manifest)
            except (OSError, ValueError, json.JSONDecodeError):
                continue

    def on_message(self, topic, message):
        mono_ns = time.monotonic_ns()
        received_ns = time.time_ns()
        payload = serialize_message(message)
        stamp = getattr(getattr(message, "header", None), "stamp", None)
        header_ns = None if stamp is None else stamp.sec * 1_000_000_000 + stamp.nanosec
        item = Sample(topic, payload, received_ns, mono_ns,
                      header_ns if header_ns and header_ns > 0 else None,
                      len(payload) + SAMPLE_OVERHEAD_BYTES)
        with self.lock:
            if not self.ring.push(item):
                return
            capture = self.active
            if capture and mono_ns <= capture.deadline_mono_ns:
                self._queue_post(capture, item)

    def _queue_post(self, capture, item):
        with capture.condition:
            if capture.pending_bytes + item.size_bytes > self.max_buffer_bytes:
                capture.pending_dropped[item.topic] += 1
            else:
                capture.pending.append(item)
                capture.pending_bytes += item.size_bytes
                capture.pending_highwater_bytes = max(capture.pending_highwater_bytes,
                                                      capture.pending_bytes)
                capture.condition.notify()

    def on_event(self, message):
        received_ns, mono_ns = time.time_ns(), time.monotonic_ns()
        try:
            event = json.loads(message.data)
        except (TypeError, json.JSONDecodeError):
            return
        if not isinstance(event, dict):
            return
        episode_id = event.get("episode_id")
        if event.get("event") == "started":
            with self.lock:
                self.active_episode = episode_id
        elif event.get("event") in ("stopped", "discarded", "failed", "interrupted"):
            with self.lock:
                if episode_id == self.active_episode:
                    self.active_episode = None
        elif event.get("event") == "annotation" and event.get("kind") == "recovery_trigger":
            self.trigger(event, received_ns, mono_ns)

    def trigger(self, event, received_ns=None, mono_ns=None):
        received_ns = received_ns or time.time_ns()
        mono_ns = mono_ns or time.monotonic_ns()
        recovery_id = event.get("recovery_id")
        if not isinstance(recovery_id, str) or not RECOVERY_ID.fullmatch(recovery_id):
            return False
        directory = self.root / f"recovery_{recovery_id}"
        try:
            directory.mkdir()
        except FileExistsError:
            return False
        free_before = self._disk_free()
        manifest = {"schema_version": 1, "status": "recording", "training_approved": False,
                    "recovery_id": recovery_id,
                    "episode_id": event.get("episode_id"), "skill": event.get("skill"),
                    "trigger_reason": event.get("value"),
                    "trigger_recorded_at_unix_ns": event.get("recorded_at_unix_ns"),
                    "trigger_received_at_unix_ns": received_ns,
                    "pre_roll_sec": self.pre_roll_ns / 1e9, "post_roll_sec": self.post_roll_ns / 1e9,
                    "max_buffer_bytes": self.max_buffer_bytes, "free_bytes_before": free_before,
                    "bag_relative_path": "bag", "topics": self.topics,
                    "failure_reasons": []}
        reasons = manifest["failure_reasons"]
        with self.lock:
            if event.get("episode_id") != self.active_episode or not self.active_episode:
                reasons.append("触发事件不属于当前正在录制的 Episode")
            if event.get("skill") not in SKILLS:
                reasons.append("触发事件没有有效的当前技能")
            if self.active is not None:
                reasons.append("已有恢复片段正在写入")
            if free_before < self.min_free_bytes:
                reasons.append("剩余磁盘空间低于配置下限")
            if reasons:
                manifest["status"] = "failed"
                manifest["free_bytes_after"] = free_before
                atomic_json(directory / "manifest.json", manifest)
                return False
            self.ring.prune(mono_ns)
            pre = [item for item in self.ring.items if item.received_mono_ns <= mono_ns]
            future = [item for item in self.ring.items if item.received_mono_ns > mono_ns]
            manifest["buffer_at_trigger"] = self.ring.audit()
            manifest["pre_snapshot_messages"] = len(pre)
            manifest["pre_snapshot_bytes"] = sum(item.size_bytes for item in pre)
            atomic_json(directory / "manifest.json", manifest)
            capture = Capture(directory, manifest, mono_ns, mono_ns + self.post_roll_ns, pre)
            capture.thread = threading.Thread(target=self._write_capture, args=(capture,), daemon=True)
            self.active = capture
            for item in future:
                self._queue_post(capture, item)
            capture.thread.start()
        return True

    def _disk_free(self):
        return shutil.disk_usage(self.root).free

    def _write_capture(self, capture):
        manifest = capture.manifest
        counts, pre_counts, post_counts = Counter(), Counter(), Counter()
        first_header, last_header = {}, {}
        first_received, last_received = {}, {}
        first_pre_mono, last_post_mono = {}, {}
        missing, non_increasing = Counter(), Counter()
        first_mono, last_mono = None, None
        written_bytes = 0
        checked_bytes, checked_at = 0, time.monotonic()
        writer = None
        error = None
        try:
            writer = rosbag2_py.SequentialWriter()
            writer.open(rosbag2_py.StorageOptions(uri=str(capture.directory / "bag"),
                                                  storage_id="sqlite3", max_cache_size=0),
                        rosbag2_py.ConverterOptions(input_serialization_format="cdr",
                                                   output_serialization_format="cdr"))
            for topic, kind in self.topics.items():
                writer.create_topic(rosbag2_py.TopicMetadata(name=topic, type=kind,
                                                              serialization_format="cdr"))

            def write_one(item, phase):
                nonlocal first_mono, last_mono, written_bytes, checked_bytes, checked_at
                now = time.monotonic()
                if (self.min_free_bytes and
                        (written_bytes - checked_bytes >= 16_000_000 or now - checked_at >= 0.5)):
                    if self._disk_free() < self.min_free_bytes:
                        raise OSError("写入期间剩余磁盘空间低于配置下限")
                    checked_bytes, checked_at = written_bytes, now
                writer.write(item.topic, item.payload, item.received_at_ns)
                counts[item.topic] += 1
                (pre_counts if phase == "pre" else post_counts)[item.topic] += 1
                first_received.setdefault(item.topic, item.received_at_ns)
                last_received[item.topic] = item.received_at_ns
                if phase == "pre":
                    first_pre_mono.setdefault(item.topic, item.received_mono_ns)
                else:
                    last_post_mono[item.topic] = item.received_mono_ns
                first_mono = item.received_mono_ns if first_mono is None else min(first_mono, item.received_mono_ns)
                last_mono = item.received_mono_ns if last_mono is None else max(last_mono, item.received_mono_ns)
                written_bytes += len(item.payload)
                if item.header_stamp_ns:
                    first_header.setdefault(item.topic, item.header_stamp_ns)
                    previous = last_header.get(item.topic)
                    if previous is not None:
                        delta = item.header_stamp_ns - previous
                        rate = self.config.get("expected_fps", {}).get(item.topic)
                        if delta <= 0:
                            non_increasing[item.topic] += 1
                        elif rate and delta > 1.5 * 1e9 / rate:
                            missing[item.topic] += max(0, round(delta * rate / 1e9) - 1)
                    last_header[item.topic] = item.header_stamp_ns

            for item in capture.pre:
                write_one(item, "pre")
            capture.pre.clear()
            while True:
                with capture.condition:
                    while not capture.pending and not capture.interrupted:
                        remaining = (capture.deadline_mono_ns - time.monotonic_ns()) / 1e9
                        if remaining <= 0:
                            break
                        capture.condition.wait(timeout=remaining)
                    if capture.interrupted or not capture.pending:
                        break
                    item = capture.pending.popleft()
                    capture.pending_bytes -= item.size_bytes
                write_one(item, "post")
        except Exception as exc:
            error = str(exc)
        finally:
            if writer is not None:
                try:
                    writer.close()
                except Exception as exc:
                    error = error or str(exc)
        manifest["messages_per_topic"] = dict(counts)
        manifest["pre_messages_per_topic"] = dict(pre_counts)
        manifest["post_messages_per_topic"] = dict(post_counts)
        manifest["first_header_stamp_ns"] = first_header
        manifest["last_header_stamp_ns"] = last_header
        manifest["first_received_at_unix_ns"] = first_received
        manifest["last_received_at_unix_ns"] = last_received
        manifest["estimated_missing_frames"] = dict(missing)
        manifest["non_increasing_header_stamps"] = dict(non_increasing)
        manifest["post_queue_dropped"] = dict(capture.pending_dropped)
        manifest["post_queue_highwater_bytes"] = capture.pending_highwater_bytes
        manifest["serialized_payload_bytes"] = written_bytes
        with self.lock:
            manifest["buffer_after_capture"] = self.ring.audit()
        before = manifest["buffer_at_trigger"]["oversize_dropped"]
        after = manifest["buffer_after_capture"]["oversize_dropped"]
        manifest["oversize_dropped_during_capture"] = {
            topic: count - before.get(topic, 0) for topic, count in after.items()
            if count > before.get(topic, 0)}
        manifest["actual_pre_roll_sec"] = max(0, (capture.trigger_mono_ns - first_mono) / 1e9) if first_mono else 0
        manifest["actual_post_roll_sec"] = max(0, (last_mono - capture.trigger_mono_ns) / 1e9) if last_mono else 0
        manifest["actual_pre_roll_sec_by_topic"] = {
            topic: max(0, (capture.trigger_mono_ns - first_pre_mono[topic]) / 1e9)
            for topic in first_pre_mono}
        manifest["actual_post_roll_sec_by_topic"] = {
            topic: max(0, (last_post_mono[topic] - capture.trigger_mono_ns) / 1e9)
            for topic in last_post_mono}
        manifest["free_bytes_after"] = self._disk_free()
        reasons = manifest["failure_reasons"]
        if error:
            reasons.append(f"rosbag2 写入失败：{error}")
        if manifest["free_bytes_after"] < self.min_free_bytes:
            reasons.append("写入期间剩余磁盘空间低于配置下限")
        if capture.interrupted:
            reasons.append("缓存进程在后录窗口结束前退出")
        if capture.pending_dropped:
            reasons.append("后录写队列溢出，消息已丢失")
        if manifest["oversize_dropped_during_capture"]:
            reasons.append("消息超过缓存字节上限，片段不完整")
        if missing:
            reasons.append("消息 Header 间隙超过预期周期，估计存在缺帧")
        if non_increasing:
            reasons.append("消息 Header 时间戳未单调递增")
        for topic in self.topics:
            if not pre_counts[topic] or not post_counts[topic]:
                reasons.append(f"{topic} 的前录或后录消息缺失")
            elif (manifest["actual_pre_roll_sec_by_topic"][topic] < 0.9 * self.pre_roll_ns / 1e9
                  or manifest["actual_post_roll_sec_by_topic"][topic] < 0.9 * self.post_roll_ns / 1e9):
                reasons.append(f"{topic} 的前录或后录覆盖时长不足配置的 90%")
        if manifest["actual_pre_roll_sec"] < 0.9 * self.pre_roll_ns / 1e9:
            reasons.append("实际前录时长不足配置的 90%")
        if manifest["actual_post_roll_sec"] < 0.9 * self.post_roll_ns / 1e9:
            reasons.append("实际后录时长不足配置的 90%")
        manifest["status"] = "failed" if reasons else "complete"
        manifest["training_approved"] = False
        try:
            atomic_json(capture.directory / "manifest.json", manifest)
        finally:
            with self.lock:
                if self.active is capture:
                    self.active = None

    def close(self):
        with self.lock:
            capture = self.active
        if capture:
            with capture.condition:
                capture.interrupted = True
                capture.condition.notify_all()
            capture.thread.join(timeout=30)
        self.node.destroy_node()
        self.lock_file.close()


def main():
    parser = argparse.ArgumentParser(description="可选的被动 ROS 恢复片段缓存")
    parser.add_argument("--config", required=True, help="恢复缓存 JSON 配置")
    args = parser.parse_args()
    config = load_config(args.config)
    if not config["enabled"]:
        print("恢复缓存未启用；没有订阅或写入数据")
        return
    rclpy.init()
    cache = RecoveryCache(config)
    try:
        rclpy.spin(cache.node)
    except KeyboardInterrupt:
        pass
    finally:
        cache.close()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
