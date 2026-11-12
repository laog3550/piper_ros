#!/usr/bin/env python3
"""只读 Episode 录制服务与 CLI。运行：python3 episode_recorder.py serve --config config.json；另见 --help。"""

import argparse
import copy
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import signal
import socket
import sqlite3
import statistics
import struct
import subprocess
import sys
import time
import uuid

import rclpy
from piper_msgs.srv import StartEpisode
from std_msgs.msg import String
from std_srvs.srv import Trigger
from completion_validation import (supports_source_validation, validate_completion,
                                   header_receive_ns, recorded_image_preview)
from startup_readiness import StartupReadiness
from vlm_annotations import (required_text, validate_annotation, validate_plan,
                             validate_plan_link, validate_skill_context)


SKILLS = ("pick_and_place_banana_in_box", "flip_box_lid_bimanual", "latch_box_lid_bimanual")
ANNOTATION_KINDS = ("skill_start", "skill_complete", "outcome", "recovery_trigger",
                    "hold_start", "hold_end")
CLOCK_STEP_THRESHOLD_NS = 50_000_000
# The socket loop is also the only place that services the live master-target
# subscription and the terminal-stale check.  Keep the accept timeout well
# below the freshness window: at 0.2 s the last stale callback is already
# older than the 0.1 s window when status() runs, so auto-trim never fires.
LIVE_STALE_FRESHNESS_NS = 100_000_000
RECORDER_STATUS_POLL_SEC = 0.05


def drain_ros_callbacks(node, limit=32, budget_sec=0.005):
    """Bound callback work while draining more than one source per socket poll."""
    executor = rclpy.get_global_executor()
    executor.add_node(node)
    deadline = time.monotonic() + budget_sec
    for _ in range(limit):
        executor.spin_once(timeout_sec=0)
        if time.monotonic() >= deadline:
            break


def atomic_json(path, data):
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        json.dump(data, stream, ensure_ascii=False, indent=2)
        stream.flush()
        os.fsync(stream.fileno())
    temporary.replace(path)


def load_json(path):
    with open(path, encoding="utf-8") as stream:
        return json.load(stream)


def write_storage_config(path):
    """Allow nonblocking startup reads while cached rosbag batches are written."""
    path.write_text('write:\n  pragmas: ["busy_timeout=5000", "journal_mode=WAL", '
                    '"synchronous=NORMAL"]\n', encoding='utf-8')
    return path


def gripper_calibration_evidence(path):
    """Pin the exact calibration chosen by the recording configuration."""
    from to_lerobot import load_gripper_calibration
    path = Path(path).expanduser().resolve()
    raw = path.read_bytes()
    settings = load_gripper_calibration(path)
    if path.read_bytes() != raw:
        raise ValueError("读取期间夹爪标定发生变化，请重新开始")
    canonical = json.dumps(settings, sort_keys=True, separators=(",", ":")).encode()
    return {"path": str(path), "sha256": hashlib.sha256(raw).hexdigest(),
            "settings_sha256": hashlib.sha256(canonical).hexdigest(), "settings": settings}


def write_qos_overrides(path, topics, depth):
    """Write the rosbag2 QoS override file for high-rate CAN/vector topics.

    rosbag2 0.15.x subscribes with its own KEEP_LAST depth 10 regardless of
    the publisher's history.  At 800-1600 frames/s that is only 6-12 ms of
    headroom, so the recorder misses most raw CAN frames while images are
    written.  A deeper reader queue keeps the source frames; it does not
    change the original Header or stale flags.
    """
    lines = []
    for topic in topics:
        lines.extend([
            f"{topic}:",
            "  history: keep_last",
            f"  depth: {depth}",
            "  reliability: reliable",
            "  durability: volatile",
        ])
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


def add_save_outcome_annotation(metadata, outcome, reviewed_at_unix_ns):
    """Record a human save-time result without pretending it was marked during recording."""
    if metadata.get("schema_version") != 2 or metadata.get("mode") != "skill":
        return
    if outcome not in ("success", "failure"):
        return
    annotations = metadata.setdefault("annotations", [])
    if any(item.get("kind") == "outcome" for item in annotations):
        return
    annotations.append({"kind": "outcome", "value": outcome,
                        "annotation_origin": "save_selection",
                        "reviewed_at_unix_ns": reviewed_at_unix_ns})


def require_verified_skill_completion(metadata, outcome):
    """SFT-only gate. Raw saving records the status instead of calling this."""
    if (metadata.get("schema_version") != 2 or metadata.get("mode") != "skill"
            or outcome != "success"):
        return
    status = assess_training_eligibility(metadata, outcome)
    if status["status"] == "completion_pending":
        raise ValueError("成功保存前须有唯一且已在 rosbag 核验的当前技能完成帧；请补标")
    if status["status"] == "sft_frame_gate_failed":
        raise ValueError("成功保存前完成帧须通过来源时效、同步及连续 16 步检查；请复核完成帧")


def assess_training_eligibility(metadata, outcome):
    """Report the SFT frame gate without blocking raw episode preservation."""
    if metadata.get("schema_version") != 2 or metadata.get("mode") != "skill":
        return {"eligible": False, "status": "not_skill_episode", "reasons": []}
    if outcome != "success":
        return {"eligible": False, "status": "not_success", "reasons": []}
    completes = [item for item in metadata.get("annotations", [])
                 if item.get("kind") == "skill_complete"]
    if len(completes) != 1 or completes[0].get("value") != metadata.get("skill"):
        return {"eligible": False, "status": "completion_pending",
                "reasons": ["缺少唯一且与当前技能匹配的已确认完成帧"]}
    item = completes[0]
    reasons = []
    if item.get("verified_in_bag") is not True:
        reasons.append("完成帧未在 rosbag 核验")
    if item.get("frame_verification_error"):
        reasons.append(item["frame_verification_error"])
    if item.get("source_validation_error"):
        reasons.append(item["source_validation_error"])
    proof = item.get("source_validation")
    if not isinstance(proof, dict):
        reasons.append("完成帧缺少来源连续性及 16 步校验")
    else:
        if proof.get("alignment_policy") not in ("causal_rgb_bundle_v1",
                                                  "causal_follower_next_state_v1"):
            reasons.append("完成帧对齐策略不匹配")
        frames = proof.get("contiguous_frames")
        if type(frames) is not int or frames < 16:
            reasons.append(f"完成帧所在连续片段 {frames} 帧，少于 16")
    if reasons:
        return {"eligible": False, "status": "sft_frame_gate_failed", "reasons": reasons}
    return {"eligible": False, "status": "sft_frame_gate_passed",
            "frame_gate_passed": True, "scope": "completion_frame_only",
            "reasons": ["完成帧门禁通过，仍须独立来源验收、数据集划分和训练统计检查"]}


def required_topics_for_episode(topics, mode, metadata):
    """Require action/state from the executing arm; retain other streams as optional context."""
    arm_mode = metadata.get("arm_mode") if isinstance(metadata, dict) else None
    if mode != "skill" or arm_mode not in ("LEFT", "RIGHT"):
        return list(topics)
    inactive = "right" if arm_mode == "LEFT" else "left"
    return [topic for topic in topics if not topic.startswith((
        f"/piper_direct/{inactive}/", f"/piper_vector/{inactive}/"))]


def inspect_bag(bag_dir, topics, frame_topics=(), expected_fps=None, required_topics=None):
    files = sorted(bag_dir.glob("*.db3"))
    if not files or not (bag_dir / "metadata.yaml").is_file():
        raise ValueError("rosbag2 SQLite 文件或 metadata.yaml 缺失")
    counts = {topic: 0 for topic in topics}
    stamps = {topic: [] for topic in frame_topics}
    for path in files:
        connection = sqlite3.connect(f"file:{path}?mode=ro", uri=True)
        try:
            if connection.execute("PRAGMA quick_check").fetchone()[0] != "ok":
                raise ValueError(f"SQLite 完整性检查失败: {path.name}")
            rows = connection.execute(
                "SELECT topics.name, COUNT(messages.id) FROM topics "
                "LEFT JOIN messages ON messages.topic_id=topics.id GROUP BY topics.name"
            ).fetchall()
            for name, count in rows:
                if name in counts:
                    counts[name] += count
            if stamps:
                placeholders = ",".join("?" for _ in stamps)
                frames = connection.execute(
                    "SELECT topics.name, substr(messages.data, 1, 12) FROM messages "
                    "JOIN topics ON messages.topic_id=topics.id "
                    f"WHERE topics.name IN ({placeholders}) ORDER BY messages.timestamp, messages.id",
                    tuple(stamps),
                )
                for name, prefix in frames:
                    if len(prefix) < 12 or prefix[:1] != b"\0" or prefix[1] not in (0, 1):
                        raise ValueError(f"无法读取 {name} 的 CDR header.stamp")
                    sec, nanosec = struct.unpack_from("<iI" if prefix[1] == 1 else ">iI", prefix, 4)
                    if nanosec >= 1_000_000_000:
                        raise ValueError(f"{name} 的 header.stamp 无效")
                    stamps[name].append(sec * 1_000_000_000 + nanosec)
        finally:
            connection.close()
    required = topics if required_topics is None else required_topics
    if not set(required).issubset(topics):
        raise ValueError("required_topics 必须是 topics 的子集")
    missing = [topic for topic in required if counts[topic] == 0]
    if missing:
        raise ValueError("话题无消息: " + ", ".join(missing))
    frame_stats = {}
    for topic, values in stamps.items():
        deltas = [right - left for left, right in zip(values, values[1:])]
        positive = [delta for delta in deltas if delta > 0]
        fps = (expected_fps or {}).get(topic)
        period = round(1_000_000_000 / fps) if fps else (statistics.median(positive) if positive else None)
        frame_stats[topic] = {
            "frames": len(values),
            "first_header_stamp_ns": values[0] if values else None,
            "last_header_stamp_ns": values[-1] if values else None,
            "non_increasing_stamps": sum(delta <= 0 for delta in deltas),
            "reference_period_ns": period,
            "reference": "configured_fps" if fps else "median_positive_interval",
            "estimated_missing_frames": sum(max(0, round(delta / period) - 1) for delta in positive) if period else None,
            "gaps_over_1_5_periods": sum(delta > 1.5 * period for delta in positive) if period else None,
        }
    return {"messages_per_topic": counts, "sqlite_files": [p.name for p in files],
            "frame_stats_by_header_stamp": frame_stats}


def recorded_frame_near(bag_dir, topic, target_ns, start_ns, stop_ns):
    """Resolve a review choice to an actual image Header stored in rosbag2."""
    nearest = None
    for path in sorted(bag_dir.glob("*.db3")):
        with sqlite3.connect(f"file:{path}?mode=ro", uri=True) as connection:
            rows = connection.execute(
                "SELECT messages.timestamp, substr(messages.data, 1, 12) FROM messages "
                "JOIN topics ON messages.topic_id=topics.id "
                "WHERE topics.name=? AND messages.timestamp BETWEEN ? AND ?",
                (topic, start_ns, stop_ns))
            for received_ns, prefix in rows:
                if len(prefix) < 12 or prefix[:1] != b"\0" or prefix[1] not in (0, 1):
                    continue
                sec, nanosec = struct.unpack_from("<iI" if prefix[1] == 1 else ">iI", prefix, 4)
                header_ns = sec * 1_000_000_000 + nanosec
                if nanosec >= 1_000_000_000 or not start_ns <= header_ns <= stop_ns:
                    continue
                candidate = (abs(received_ns - target_ns), received_ns, header_ns)
                if nearest is None or candidate < nearest:
                    nearest = candidate
    if nearest is None:
        raise ValueError("所选相机在本条 Episode 中没有可核验的录制帧")
    if nearest[0] > 500_000_000:
        raise ValueError("所选完成时刻附近 0.5 秒内没有录制帧")
    return nearest[1], nearest[2]


def recorded_header_exists(bag_dir, topic, header_ns, start_ns, stop_ns, *, receive_stop_ns=None):
    """Bound the image Header by the task, and receipt by the physical recording."""
    if not start_ns <= header_ns <= stop_ns:
        return False
    receive_stop_ns = stop_ns if receive_stop_ns is None else receive_stop_ns
    for path in sorted(bag_dir.glob("*.db3")):
        with sqlite3.connect(f"file:{path}?mode=ro", uri=True) as connection:
            rows = connection.execute(
                "SELECT substr(messages.data, 1, 12) FROM messages "
                "JOIN topics ON messages.topic_id=topics.id "
                "WHERE topics.name=? AND messages.timestamp BETWEEN ? AND ?",
                (topic, start_ns, receive_stop_ns))
            for (prefix,) in rows:
                if len(prefix) < 12 or prefix[:1] != b"\0" or prefix[1] not in (0, 1):
                    continue
                sec, nanosec = struct.unpack_from("<iI" if prefix[1] == 1 else ">iI", prefix, 4)
                if nanosec < 1_000_000_000 and sec * 1_000_000_000 + nanosec == header_ns:
                    return True
    return False


def latest_topic_receive_before(bag_dir, topic, target_ns, start_ns):
    """Return the most recent original bag receive time; never synthesize source frames."""
    latest = None
    for path in sorted(bag_dir.glob("*.db3")):
        with sqlite3.connect(f"file:{path}?mode=ro", uri=True) as connection:
            row = connection.execute(
                "SELECT MAX(messages.timestamp) FROM messages "
                "JOIN topics ON messages.topic_id=topics.id "
                "WHERE topics.name=? AND messages.timestamp BETWEEN ? AND ?",
                (topic, start_ns, target_ns)).fetchone()
            if row and row[0] is not None:
                latest = row[0] if latest is None else max(latest, row[0])
    return latest


def last_valid_master_target(bag_dir, arm, start_ns, stop_ns):
    """Find the last original, quality-valid master vector without extending its Header."""
    from rclpy.serialization import deserialize_message
    from piper_msgs.msg import PiperArmVector

    topic = f"/piper_vector/{arm}/master_target"
    latest = None
    for path in sorted(bag_dir.glob("*.db3")):
        with sqlite3.connect(f"file:{path}?mode=ro", uri=True) as connection:
            rows = connection.execute(
                "SELECT messages.timestamp, messages.data FROM messages "
                "JOIN topics ON messages.topic_id=topics.id "
                "WHERE topics.name=? AND messages.timestamp BETWEEN ? AND ? "
                "ORDER BY messages.timestamp, messages.id",
                (topic, start_ns, stop_ns))
            for received_ns, raw in rows:
                message = deserialize_message(raw, PiperArmVector)
                header_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
                if (message.valid and message.gripper_mapping_verified
                        and not any(message.missing) and not any(message.stale)
                        and not message.time_inconsistent and start_ns <= header_ns <= stop_ns):
                    candidate = (header_ns, received_ns)
                    if latest is None or candidate > latest:
                        latest = candidate
    return latest


def terminal_stale_master_target(bag_dir, arm, start_ns, stop_ns):
    """Find the valid-to-stale transition of a terminal source pause, not an interior gap."""
    from rclpy.serialization import deserialize_message
    from piper_msgs.msg import PiperArmVector

    topic = f"/piper_vector/{arm}/master_target"
    observations = []
    for path in sorted(bag_dir.glob("*.db3")):
        with sqlite3.connect(f"file:{path}?mode=ro", uri=True) as connection:
            rows = connection.execute(
                "SELECT messages.timestamp, messages.data FROM messages "
                "JOIN topics ON messages.topic_id=topics.id "
                "WHERE topics.name=? AND messages.timestamp BETWEEN ? AND ? "
                "ORDER BY messages.timestamp, messages.id",
                (topic, start_ns, stop_ns))
            for received_ns, raw in rows:
                message = deserialize_message(raw, PiperArmVector)
                header_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
                quality_ok = (message.gripper_mapping_verified and not any(message.missing)
                              and not message.time_inconsistent and start_ns <= header_ns <= stop_ns
                              and message.source_topic == f"/piper_direct/{arm}/master_target"
                              and message.header.frame_id == f"can_{arm}"
                              and list(message.source_can_ids) == [0x155, 0x156, 0x157, 0x159])
                observations.append((received_ns, header_ns,
                                     bool(quality_ok and message.valid and not any(message.stale)),
                                     bool(quality_ok and not message.valid and any(message.stale))))
    observations.sort(key=lambda item: item[0])
    last_valid = tail = None
    previous_receive = None
    for received_ns, header_ns, valid, stale in observations:
        gap = previous_receive is not None and received_ns - previous_receive > 100_000_000
        if valid:
            last_valid, tail = (header_ns, received_ns), None
        elif stale and last_valid is not None and not gap and header_ns <= last_valid[0]:
            if tail is None:
                tail = {"last_valid_header_ns": last_valid[0], "last_valid_receive_ns": last_valid[1],
                        "first_stale_receive_ns": received_ns, "stale_message_count": 0}
            tail["last_stale_receive_ns"] = received_ns
            tail["stale_message_count"] += 1
        else:
            # A missing/inconsistent stream or recorder gap is not a confirmed
            # terminal stop under the operator's collection convention.
            last_valid = tail = None
        previous_receive = received_ns
    if (tail is None or tail["stale_message_count"] < 2
            or stop_ns - tail["last_stale_receive_ns"] > 100_000_000):
        return None
    return tail


def classify_master_target(message, arm):
    """Classify a decoded master vector for live terminal-stale detection."""
    quality_ok = (
        bool(message.gripper_mapping_verified) and not any(message.missing)
        and not bool(message.time_inconsistent)
        and message.source_topic == f"/piper_direct/{arm}/master_target"
        and message.header.frame_id == f"can_{arm}"
        and list(message.source_can_ids) == [0x155, 0x156, 0x157, 0x159])
    if not quality_ok:
        return "reset"
    if bool(message.valid) and not any(message.stale):
        return "valid"
    if not bool(message.valid) and any(message.stale):
        return "stale"
    return "reset"


def recorded_frame_before_header(bag_dir, topic, boundary_ns, start_ns, stop_ns):
    """Choose an actual RGB Header strictly before the final target's stale boundary."""
    latest = None
    for path in sorted(bag_dir.glob("*.db3")):
        with sqlite3.connect(f"file:{path}?mode=ro", uri=True) as connection:
            rows = connection.execute(
                "SELECT messages.timestamp, substr(messages.data, 1, 12) FROM messages "
                "JOIN topics ON messages.topic_id=topics.id "
                "WHERE topics.name=? AND messages.timestamp BETWEEN ? AND ?",
                (topic, start_ns, stop_ns))
            for received_ns, prefix in rows:
                if len(prefix) < 12 or prefix[:1] != b"\0" or prefix[1] not in (0, 1):
                    continue
                sec, nanosec = struct.unpack_from("<iI" if prefix[1] == 1 else ">iI", prefix, 4)
                header_ns = sec * 1_000_000_000 + nanosec
                if not (0 <= nanosec < 1_000_000_000 and start_ns <= header_ns < boundary_ns):
                    continue
                candidate = (header_ns, received_ns)
                if latest is None or candidate > latest:
                    latest = candidate
    if latest is None or boundary_ns - latest[0] > 100_000_000:
        raise ValueError("末尾 stale 前 100 ms 内没有可核验的相机 Header；请录制后补标")
    return latest[1], latest[0]


class Recorder:
    def __init__(self, config):
        self.gripper_calibration_path = config.get("gripper_calibration_path")
        self.root = Path(config["output_dir"]).expanduser().resolve()
        self.root.mkdir(parents=True, exist_ok=True)
        self.staging = self.root / ".staging"
        self.staging.mkdir(exist_ok=True)
        self.discarded = self.root / "discarded"
        self.discarded.mkdir(exist_ok=True)
        self.plans_dir = self.root / "plans"
        self.plans_dir.mkdir(exist_ok=True)
        self.events_topic = "/collection/events"
        self.topics = list(config["topics"])
        if self.events_topic not in self.topics:
            self.topics.append(self.events_topic)
        if not self.topics or len(set(self.topics)) != len(self.topics) or any(
            not isinstance(topic, str) or not topic.startswith("/") for topic in self.topics
        ):
            raise ValueError("topics 必须是非空、无重复的绝对 ROS 话题名列表")
        self.frame_topics = config.get("frame_topics", [])
        self.expected_fps = config.get("expected_fps", {})
        if (not isinstance(self.frame_topics, list) or len(set(self.frame_topics)) != len(self.frame_topics)
                or not set(self.frame_topics).issubset(self.topics)):
            raise ValueError("frame_topics 必须是 topics 的无重复子集")
        if not isinstance(self.expected_fps, dict) or not set(self.expected_fps).issubset(self.frame_topics) or any(
            not isinstance(rate, (int, float)) or rate <= 0 for rate in self.expected_fps.values()
        ):
            raise ValueError("expected_fps 只能为 frame_topics 指定正数帧率")
        self.min_free_bytes = int(float(config.get("min_free_gb", 1)) * 1_000_000_000)
        self.timeout = float(config.get("startup_timeout_sec", 15))
        self._last_clock_offset_ns = None
        self.rosbag_cache_bytes = config.get("rosbag_cache_bytes", 268_435_456)
        if (isinstance(self.rosbag_cache_bytes, bool)
                or not isinstance(self.rosbag_cache_bytes, int)
                or not 0 <= self.rosbag_cache_bytes <= 2_147_483_648):
            raise ValueError("rosbag_cache_bytes 必须为 0 至 2147483648 的整数")
        self.raw_can_qos_depth = config.get("raw_can_qos_depth", 5000)
        if (isinstance(self.raw_can_qos_depth, bool)
                or not isinstance(self.raw_can_qos_depth, int)
                or not 0 <= self.raw_can_qos_depth <= 100_000):
            raise ValueError("raw_can_qos_depth 必须为 0 至 100000 的整数")
        self.hold_max_sec = config.get("hold_max_sec", 10.0)
        self.hold_anchor_max_gap_sec = config.get("hold_anchor_max_gap_sec", 2.0)
        for name, value in (("hold_max_sec", self.hold_max_sec),
                            ("hold_anchor_max_gap_sec", self.hold_anchor_max_gap_sec)):
            if (isinstance(value, bool) or not isinstance(value, (int, float))
                    or not math.isfinite(value) or value <= 0):
                raise ValueError(f"{name} 必须为正数")
        self.auto_trim_stale_tail = config.get("auto_trim_stale_tail", False)
        if not isinstance(self.auto_trim_stale_tail, bool):
            raise ValueError("auto_trim_stale_tail 必须为布尔值")
        self.stale_stop_grace_sec = config.get("stale_stop_grace_sec", 1.0)
        if (isinstance(self.stale_stop_grace_sec, bool)
                or not isinstance(self.stale_stop_grace_sec, (int, float))
                or not math.isfinite(self.stale_stop_grace_sec)
                or not 0.2 <= self.stale_stop_grace_sec <= 10):
            raise ValueError("stale_stop_grace_sec 必须为 0.2 至 10 秒的有限数值")
        self.max_camera_skew_ms = config.get("max_camera_skew_ms", 40)
        self.max_source_skew_ms = config.get("max_source_skew_ms", 40)
        for name, value in (("max_camera_skew_ms", self.max_camera_skew_ms),
                            ("max_source_skew_ms", self.max_source_skew_ms)):
            if (isinstance(value, bool) or not isinstance(value, (int, float))
                    or not math.isfinite(value) or not 0 < value <= 100):
                raise ValueError(f"{name} 必须为 0 至 100 的正数")
        if self.max_source_skew_ms < self.max_camera_skew_ms:
            raise ValueError("max_source_skew_ms 不能小于 max_camera_skew_ms")
        self._master_monitor = {}
        self._master_subscriptions = []
        self.socket_path = Path(config.get("socket_path", str(self.root / "recorder.sock"))).expanduser().resolve()
        self.lock = (self.root / ".recorder.lock").open("w")
        fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        self.node = rclpy.create_node("piper_episode_manager")
        self.event_publisher = self.node.create_publisher(String, self.events_topic, 10)
        self.process = None
        self.current = None
        self.ready_monotonic_ns = None
        self.deadline_monotonic_ns = None
        self._recover()
        self.node.create_service(StartEpisode, "/collection/start", self._service_start)
        for action in ("stop", "save", "discard", "status", "list"):
            self.node.create_service(Trigger, f"/collection/{action}", self._trigger(action))

    def _service_start(self, request, response):
        try:
            fields = json.loads(request.metadata_json or "{}")
            duration = fields.pop("duration_limit_sec", None) if isinstance(fields, dict) else None
            result = self.start({"mode": request.mode, "task": request.task,
                                 "skill": request.skill, "metadata": fields,
                                 "duration_sec": duration})
            response.success = True
            response.result_json = json.dumps(result, ensure_ascii=False)
        except (ValueError, OSError, KeyError, sqlite3.Error) as error:
            response.success = False
            response.result_json = json.dumps({"error": str(error)}, ensure_ascii=False)
        return response

    def _trigger(self, action):
        def callback(_request, response):
            try:
                response.message = json.dumps(getattr(self, action)(), ensure_ascii=False)
                response.success = True
            except (ValueError, OSError, KeyError, sqlite3.Error) as error:
                response.message = json.dumps({"error": str(error)}, ensure_ascii=False)
                response.success = False
            return response
        return callback

    def _event(self, name, **details):
        event = {"episode_id": self.metadata["episode_id"], "event": name,
                 "time": time.time_ns(), "time_unit": "unix_ns", **details}
        self.metadata.setdefault("events", []).append(event)
        self._write_current()
        self.event_publisher.publish(String(data=json.dumps(event, ensure_ascii=False)))

    def _recover(self):
        pending = []
        for directory in sorted(self.staging.iterdir()):
            if not directory.is_dir():
                continue
            metadata_file = directory / "episode.json"
            metadata = load_json(metadata_file) if metadata_file.exists() else {
                "schema_version": 1, "episode_id": directory.name, "state": "starting",
                "error": "暂存目录无元数据，录制服务意外中断",
            }
            pid = metadata.get("recorder_pid")
            if pid:
                command_file = Path(f"/proc/{pid}/cmdline")
                try:
                    command = command_file.read_bytes().replace(b"\0", b" ").decode(errors="replace")
                    if str(directory / "bag") in command and "ros2 bag record" in command:
                        os.killpg(pid, signal.SIGINT)
                        deadline = time.monotonic() + 30
                        while command_file.exists() and time.monotonic() < deadline:
                            time.sleep(0.1)
                        if command_file.exists():
                            os.killpg(pid, signal.SIGTERM)
                except (OSError, ProcessLookupError):
                    pass
            if metadata.get("state") in ("recording", "starting"):
                metadata["state"] = "interrupted"
                metadata["error"] = "录制服务意外中断；请检查后丢弃或重新开始"
                event = {"episode_id": metadata.get("episode_id", directory.name),
                         "event": "interrupted", "time": time.time_ns(), "time_unit": "unix_ns"}
                metadata.setdefault("events", []).append(event)
                self.event_publisher.publish(String(data=json.dumps(event, ensure_ascii=False)))
                atomic_json(metadata_file, metadata)
            pending.append((directory, metadata))
        if len(pending) == 1 and pending[0][1].get("state") == "stopped":
            self.current, self.metadata = pending[0]

    def _write_current(self):
        atomic_json(self.current / "episode.json", self.metadata)

    def _available(self):
        return shutil.disk_usage(self.root).free

    def _publisher_snapshot(self):
        """Reject mixed publishers before recording, including optional streams."""
        snapshot = {}
        for topic in self.topics:
            publishers = self.node.get_publishers_info_by_topic(topic)
            snapshot[topic] = [{"node_name": info.node_name,
                                "node_namespace": info.node_namespace,
                                "gid": bytes(info.endpoint_gid).hex()}
                               for info in publishers]
            if len(publishers) > 1:
                names = ", ".join(info.node_namespace.rstrip("/") + "/" + info.node_name
                                  for info in publishers)
                raise ValueError(f"录制被拒绝：{topic} 有 {len(publishers)} 个发布者（{names}）；"
                                 "请先停止重复的采集实例，每个来源话题只能有一个发布者")
        return snapshot

    def _graph_ready(self, node_name):
        publishers = self._publisher_snapshot()
        for topic in self.metadata.get("required_topics", self.topics):
            subs = self.node.get_subscriptions_info_by_topic(topic)
            if not publishers[topic] or not any(info.node_name == node_name for info in subs):
                return False
        self.metadata["source_publishers_at_ready"] = publishers
        return True

    def _start_data_monitor(self):
        required = self.metadata['required_topics']
        topics = [topic for topic in required if topic in self.frame_topics
                  or topic.endswith('/color/camera_info')
                  or (topic.startswith('/piper_vector/') and topic.endswith('/slave_feedback'))]
        self._startup_subscriptions = []
        if not topics:
            return StartupReadiness(())
        from rclpy.serialization import deserialize_message
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from rosidl_runtime_py.utilities import get_message
        vector_type = get_message('piper_msgs/msg/PiperArmVector')

        def validate_vector(raw):
            message = deserialize_message(raw, vector_type)
            if (not message.valid or any(message.stale) or any(message.missing)
                    or message.time_inconsistent or len(message.position) != 7
                    or not all(math.isfinite(value) for value in message.position)):
                raise ValueError('从臂反馈不完整、过期或数值无效')

        monitor = StartupReadiness(topics, validate_vector)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        for topic in topics:
            msgtype = ('piper_msgs/msg/PiperArmVector' if topic.startswith('/piper_vector/') else
                       'sensor_msgs/msg/CameraInfo' if topic.endswith('/camera_info') else
                       'sensor_msgs/msg/Image')
            self._startup_subscriptions.append(self.node.create_subscription(
                get_message(msgtype), topic, lambda raw, topic=topic: monitor.observe(topic, raw),
                qos, raw=True))
        return monitor

    def _clear_startup_monitor(self):
        for subscription in getattr(self, '_startup_subscriptions', ()):
            self.node.destroy_subscription(subscription)
        self._startup_subscriptions = []

    def start(self, request):
        if self.current is not None:
            raise ValueError("已有活动 Episode")
        if any(self.staging.iterdir()):
            raise ValueError("存在未处理的暂存 Episode；请先 discard")
        mode = request.get("mode")
        task = request.get("task")
        skill = request.get("skill")
        fields = request.get("metadata", {})
        duration = request.get("duration_sec")
        if mode not in ("task", "skill") or not isinstance(task, str) or not task.strip():
            raise ValueError("mode 必须为 task 或 skill，且 task 不能为空")
        if mode == "skill" and (not isinstance(skill, str) or not skill.strip()):
            raise ValueError("原子技能录制需要 skill")
        if not isinstance(fields, dict):
            raise ValueError("metadata 必须是 JSON 对象")
        fields = dict(fields)
        if getattr(self, "gripper_calibration_path", None):
            fields["gripper_calibration_evidence"] = gripper_calibration_evidence(self.gripper_calibration_path)
        if mode == "skill" and "vlm" not in fields and skill not in SKILLS:
            raise ValueError("skill 不在固定技能集中")
        if "vlm" in fields:
            if mode != "skill":
                raise ValueError("VLM 技能上下文需要 mode=skill")
            skill = required_text(skill, "原子技能名", 120)
            context = validate_skill_context(fields["vlm"])
            if context["plan_id"]:
                plan_file = self.plans_dir / f'{context["plan_id"]}.json'
                if not plan_file.is_file():
                    raise ValueError("关联的 plan_id 不存在")
                plan = load_json(plan_file)
                if context["task_group_id"] and context["task_group_id"] != plan["task_group_id"]:
                    raise ValueError("task_group_id 与规划样本不一致")
                validate_plan_link(plan, context, skill)
                context["task_group_id"] = plan["task_group_id"]
            context["task_group_id"] = context["task_group_id"] or uuid.uuid4().hex
            fields["vlm"] = context
        if duration is not None and (isinstance(duration, bool)
                                     or not isinstance(duration, (int, float))
                                     or not math.isfinite(duration) or not 1 <= duration <= 86400):
            raise ValueError("duration_sec 必须为 1 至 86400 秒的有限数值")
        free = self._available()
        if free < self.min_free_bytes:
            raise ValueError(f"剩余磁盘空间不足: {free} 字节")
        self._publisher_snapshot()
        self.ready_monotonic_ns = None
        self.deadline_monotonic_ns = None
        self.current = self.staging / uuid.uuid4().hex
        self.current.mkdir()
        storage_path = write_storage_config(self.current / 'recording_storage.yaml')
        qos_topics = [topic for topic in self.topics
                      if topic.startswith(("/piper_direct/", "/piper_vector/"))
                      and topic.rsplit("/", 1)[-1] in ("master_target", "slave_feedback")]
        qos_path = None
        if self.raw_can_qos_depth and qos_topics:
            qos_path = write_qos_overrides(
                self.current / "recording_qos.yaml", qos_topics, self.raw_can_qos_depth)
        self.metadata = {
            "schema_version": 2 if "vlm" not in fields else 1, "state": "starting", "mode": mode,
            "episode_id": self.current.name,
            "task": task, "skill": skill if mode == "skill" else None,
            "active_skill": skill if mode == "skill" else None,
            "annotations": [],
            "task_metadata": fields, "topics": self.topics,
            "required_topics": required_topics_for_episode(self.topics, mode, fields),
            "rosbag_relative_path": "bag",
            "duration_limit_sec": duration,
            "frame_topics": self.frame_topics, "expected_fps": self.expected_fps,
            "rosbag_cache_bytes": self.rosbag_cache_bytes,
            "raw_can_qos_depth": self.raw_can_qos_depth,
            "qos_profile_overrides": qos_path.name if qos_path else None,
            "storage_config": storage_path.name,
            "storage_policy": "sqlite_wal_normal_busy_timeout_5000_v1",
            "hold_policy": {"source": "operator_annotation",
                            "max_hold_sec": self.hold_max_sec,
                            "anchor_max_gap_sec": self.hold_anchor_max_gap_sec},
            "auto_trim_stale_tail": getattr(self, "auto_trim_stale_tail", False),
            "stale_stop_grace_sec": getattr(self, "stale_stop_grace_sec", 1.0),
            "skew_thresholds": {
                "max_camera_skew_ms": getattr(self, "max_camera_skew_ms", 40),
                "max_source_skew_ms": getattr(self, "max_source_skew_ms", 40),
            },
            "started_at_unix_ns": time.time_ns(),
            "timestamp_note": "rosbag2 messages.timestamp 是接收时间；原始消息 header.stamp 保留在 CDR 消息中",
        }
        self._write_current()
        self._master_monitor = {}
        self._master_subscriptions = []
        if getattr(self, "auto_trim_stale_tail", False) and mode == "skill":
            from piper_msgs.msg import PiperArmVector
            from rclpy.qos import QoSProfile, ReliabilityPolicy
            # This is a latest-state monitor, not the archival subscriber.
            # Old queued flags delay stop detection; rosbag records every vector.
            monitor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            arm_mode = fields.get("arm_mode")
            arms = ("left", "right") if arm_mode == "BIMANUAL" else (
                (arm_mode.lower(),) if arm_mode in ("LEFT", "RIGHT") else ())
            for arm in arms:
                topic = f"/piper_vector/{arm}/master_target"
                if topic not in self.topics:
                    continue
                self._master_subscriptions.append(self.node.create_subscription(
                    PiperArmVector, topic,
                    lambda message, arm=arm: self._on_master_target(arm, message),
                    monitor_qos))
        node_name = f"piper_episode_{os.getpid()}"
        command = ["ros2", "bag", "record", "--storage", "sqlite3", "--max-cache-size",
                   str(self.rosbag_cache_bytes), "--storage-config-file", str(storage_path)]
        if qos_path is not None:
            command.extend(["--qos-profile-overrides-path", str(qos_path)])
        command.extend(["--node-name", node_name, "-o", str(self.current / "bag"), *self.topics])
        log = (self.current / "rosbag.log").open("w")
        try:
            self.process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            self.metadata["recorder_pid"] = self.process.pid
            self._write_current()
        except OSError as error:
            self.metadata["state"] = "failed"
            self.metadata["error"] = str(error)
            self._event("failed")
            raise
        finally:
            log.close()
        deadline = time.monotonic() + self.timeout
        startup_error = None
        readiness = None
        try:
            readiness = self._start_data_monitor()
            while time.monotonic() < deadline:
                if self.process.poll() is not None:
                    break
                time.sleep(0.02)
                drain_ros_callbacks(self.node)
                try:
                    ready_graph = self._graph_ready(node_name)
                except ValueError as error:
                    startup_error = str(error)
                    break
                if ready_graph and (self.current / "bag").is_dir() and readiness.ready(self.current / "bag"):
                    self.metadata["startup_data_readiness"] = readiness.evidence()
                    self.metadata["state"] = "recording"
                    ready = time.time_ns()
                    self.metadata["ready_at_unix_ns"] = ready
                    self._last_clock_offset_ns = ready - time.monotonic_ns()
                    self.metadata["clock_adjustments"] = []
                    if self.metadata.get("schema_version") == 2 and mode == "task":
                        self.metadata["active_skill"] = SKILLS[0]
                        self.metadata["annotations"].append({
                            "kind": "skill_start", "value": SKILLS[0],
                            "annotation_origin": "automatic_task_start",
                            "recorded_at_unix_ns": ready})
                    if duration is not None:
                        duration_ns = round(duration * 1_000_000_000)
                        self.ready_monotonic_ns = time.monotonic_ns()
                        self.deadline_monotonic_ns = self.ready_monotonic_ns + duration_ns
                        self.metadata["recording_deadline_unix_ns"] = ready + duration_ns
                    self._event("started")
                    return self.status()
        except (ValueError, OSError) as error:
            startup_error = str(error)
        finally:
            self._clear_startup_monitor()
        if not startup_error and readiness is not None and readiness.last_error:
            startup_error = "来源数据未就绪：" + readiness.last_error
        self._terminate()
        self.metadata["state"] = "failed"
        self.metadata["error"] = startup_error or "rosbag2 未在超时内订阅全部配置话题；见 rosbag.log"
        self._event("failed")
        raise ValueError(self.metadata["error"])

    def annotate(self, request):
        state = self.metadata.get("state")
        if self.current is None or state not in ("recording", "stopped"):
            raise ValueError("只能在录制中或停止后保存前标注")
        if self.metadata.get("schema_version") != 2:
            raise ValueError("旧版 VLM 技能片段不支持 v2 实时标注")
        kind = request.get("kind")
        value = request.get("value")
        if kind not in ANNOTATION_KINDS:
            raise ValueError("未知标注类型")
        if kind in ("hold_start", "hold_end") and state != "recording":
            raise ValueError("保持意图只能在录制中标注")
        post_review = state == "stopped"
        if post_review and (self.metadata.get("mode") != "skill" or kind != "skill_complete"):
            raise ValueError("停止后仅可复核原子技能完成帧；结果请在保存时选择")
        if state == "recording" and self.metadata.get("mode") == "skill" and kind == "outcome":
            raise ValueError("原子技能结果请在录制结束后选择")
        if state == "recording" and self.metadata.get("mode") == "skill" and kind == "skill_complete":
            if request.get("skill_result") != "success":
                raise ValueError("原子技能请点击“标记当前技能成功”以记录完成帧")
            if request.get("source_topic") is None or request.get("header_stamp_ns") is None:
                raise ValueError("技能成功标记必须关联当前彩色相机帧")
        if state == "recording" and self.metadata.get("mode") == "task" and kind == "skill_complete":
            if request.get("skill_result") != "success":
                raise ValueError("完整任务请点击“标记当前技能成功”以记录完成帧")
            if request.get("source_topic") is None or request.get("header_stamp_ns") is None:
                raise ValueError("技能成功标记必须关联当前彩色相机帧")
        active = self.metadata.get("active_skill")
        next_active = active
        if kind == "skill_start":
            if self.metadata["mode"] != "task" or active is not None or value not in SKILLS:
                raise ValueError("当前不能开始该技能")
            completed = [item["value"] for item in self.metadata["annotations"]
                         if item["kind"] == "skill_complete"]
            if len(completed) >= len(SKILLS) or value != SKILLS[len(completed)]:
                raise ValueError("技能顺序必须为单臂香蕉入盒、双臂翻盖、双臂扣盖")
            next_active = value
        elif kind == "skill_complete":
            if active is None or value != active:
                raise ValueError("完成标注与当前技能不一致")
            if self.metadata["mode"] == "skill" and any(
                item["kind"] == "skill_complete" for item in self.metadata["annotations"]
            ):
                raise ValueError("原子技能完成帧不能重复标注")
            if self.metadata["mode"] == "task":
                following = SKILLS.index(active) + 1
                next_active = SKILLS[following] if following < len(SKILLS) else None
            else:
                next_active = active
        elif kind == "outcome":
            if value not in ("success", "failure"):
                raise ValueError("outcome 必须为 success 或 failure")
            if any(item["kind"] == "outcome" for item in self.metadata["annotations"]):
                raise ValueError("结果标签不能重复标注")
        elif kind == "hold_start":
            if active is None:
                raise ValueError("保持意图必须属于当前技能")
            if any(item.get("end_unix_ns") is None
                   for item in self.metadata.get("hold_segments", [])):
                raise ValueError("已有未结束的保持段")
            if not isinstance(value, str) or not value.strip():
                value = "operator_keep_last_target"
        elif kind == "hold_end":
            if not any(item.get("end_unix_ns") is None
                       for item in self.metadata.get("hold_segments", [])):
                raise ValueError("没有可结束的保持段")
            if not isinstance(value, str) or not value.strip():
                value = "operator_release"
        elif not isinstance(value, str) or not value.strip():
            raise ValueError("恢复原因不能为空")
        topic = request.get("source_topic")
        stamp = request.get("header_stamp_ns")
        candidate_confirmation = bool(request.get("candidate"))
        if post_review:
            topic = request.get("review_source_topic") or request.get("source_topic")
            if candidate_confirmation:
                candidate = self.metadata.get("auto_completion_candidate") or {}
                stamp = request.get("header_stamp_ns")
                if (topic not in self.frame_topics or not isinstance(topic, str)
                        or not topic.endswith("/image_raw") or type(stamp) is not int
                        or candidate.get("source_topic") != topic
                        or candidate.get("header_stamp_ns") != stamp):
                    raise ValueError("候选完成帧与当前自动候选不一致")
                ready = self.metadata["ready_at_unix_ns"]
                if not recorded_header_exists(
                        self.current / "bag", topic, stamp, ready,
                        self.metadata["stopped_at_unix_ns"],
                        receive_stop_ns=self.metadata.get('raw_stopped_at_unix_ns',
                                                         self.metadata['stopped_at_unix_ns'])):
                    raise ValueError("候选完成帧未写入本条 rosbag")
                received = header_receive_ns(self.current / "bag", topic, stamp)
                offset = round((received - ready) / 1_000_000_000, 3)
            else:
                offset = request.get("review_offset_sec")
                if topic not in self.frame_topics or not topic.endswith("/image_raw"):
                    raise ValueError("请选择已录制的彩色相机")
                duration = self.metadata.get("recorded_duration_sec")
                if (isinstance(offset, bool) or not isinstance(offset, (int, float))
                        or not math.isfinite(offset) or offset < 0 or duration is None or offset > duration):
                    raise ValueError("完成时刻须在本条 Episode 的录制时长内")
                ready = self.metadata["ready_at_unix_ns"]
                received, stamp = recorded_frame_near(
                    self.current / "bag", topic, ready + round(offset * 1_000_000_000),
                    ready, self.metadata["stopped_at_unix_ns"])
        if (topic is None) != (stamp is None):
            raise ValueError("source_topic 与 header_stamp_ns 必须同时提供")
        if topic is not None and (topic not in self.frame_topics or
                                  not isinstance(stamp, int) or isinstance(stamp, bool) or stamp <= 0):
            raise ValueError("完成帧来源必须是配置中的图像话题及有效 Header 时间戳")
        annotation = {"kind": kind, "value": value}
        if kind == "skill_complete" and not post_review:
            annotation["skill_result"] = "success"
        if post_review:
            if supports_source_validation(self.metadata):
                try:
                    annotation["source_validation"] = validate_completion(
                        self.current, self.metadata, topic, stamp, exact_ns=stamp,
                        max_camera_skew_ms=getattr(self, "max_camera_skew_ms", 30),
                        max_source_skew_ms=getattr(self, "max_source_skew_ms", 30))
                except ValueError as error:
                    # The operator can still confirm the completion event; the
                    # separate SFT gate keeps rejecting an invalid segment.
                    annotation["source_validation_error"] = str(error)
                    annotation["source_validation_details"] = getattr(error, "details", {})
            self.metadata.pop("completion_review_required", None)
            self.metadata.pop("auto_completion_error", None)
            annotation.update(
                annotation_origin=("operator_confirmed_candidate"
                                   if candidate_confirmation else "post_recording_review"),
                reviewed_at_unix_ns=time.time_ns(),
                verified_in_bag=True,
                selected_offset_sec=offset,
                matched_offset_sec=round((received - ready) / 1_000_000_000, 3))
        else:
            annotation["recorded_at_unix_ns"] = time.time_ns()
        if kind == "recovery_trigger":
            annotation["recovery_id"] = uuid.uuid4().hex
            annotation["skill"] = active
        if kind == "hold_start":
            hold_id = uuid.uuid4().hex
            started = annotation["recorded_at_unix_ns"]
            annotation.update(hold_id=hold_id, source="operator_annotation", skill=active,
                              start_unix_ns=started)
            self.metadata.setdefault("hold_segments", []).append({
                "hold_id": hold_id, "start_unix_ns": started, "end_unix_ns": None,
                "source": "operator_annotation", "skill": active, "intent": value})
        elif kind == "hold_end":
            segment = next(item for item in reversed(self.metadata["hold_segments"])
                           if item.get("end_unix_ns") is None)
            ended = annotation["recorded_at_unix_ns"]
            segment["end_unix_ns"] = ended
            segment["duration_ns"] = ended - segment["start_unix_ns"]
            segment["end_reason"] = "operator_release"
            annotation.update(hold_id=segment["hold_id"], source="operator_annotation",
                              end_unix_ns=ended, duration_ns=segment["duration_ns"])
        if topic is not None:
            annotation.update(source_topic=topic, header_stamp_ns=stamp)
        else:
            annotation["frame_resolution_required"] = kind == "skill_complete"
        self.metadata["active_skill"] = next_active
        self.metadata["annotations"].append(annotation)
        if kind == "skill_complete" and self.metadata["mode"] == "task" and next_active is not None:
            self.metadata["annotations"].append({
                "kind": "skill_start", "value": next_active,
                "annotation_origin": "automatic_handoff",
                "recorded_at_unix_ns": annotation["recorded_at_unix_ns"]})
        self._write_current()
        self.event_publisher.publish(String(data=json.dumps({"episode_id": self.metadata["episode_id"],
            "event": "annotation", **annotation}, ensure_ascii=False)))
        return {"annotation": annotation, "active_skill": next_active}

    def recheck_completion(self):
        """Recheck the same human-confirmed image without moving its timestamp."""
        if self.current is None or self.metadata.get("state") != "stopped":
            raise ValueError("只能重新校验停止后待保存的完成帧")
        markers = [a for a in self.metadata.get("annotations", []) if a.get("kind") == "skill_complete"]
        if self.metadata.get("mode") != "skill" or len(markers) != 1:
            raise ValueError("须有唯一已确认的原子技能完成帧")
        item = markers[0]
        topic, stamp = item.get("source_topic"), item.get("header_stamp_ns")
        if topic not in self.frame_topics or type(stamp) is not int:
            raise ValueError("完成帧缺少有效图像引用")
        previous = {key: item.get(key) for key in ("verified_in_bag", "frame_verification_error",
                                                  "source_validation", "source_validation_error",
                                                  "source_validation_details")}
        exists = recorded_header_exists(self.current / "bag", topic, stamp,
                                        self.metadata["ready_at_unix_ns"],
                                        self.metadata["stopped_at_unix_ns"],
                                        receive_stop_ns=self.metadata.get('raw_stopped_at_unix_ns',
                                                                         self.metadata['stopped_at_unix_ns']))
        for key in ("source_validation", "source_validation_error", "source_validation_details",
                    "frame_verification_error"):
            item.pop(key, None)
        item["verified_in_bag"] = exists
        if not exists:
            item["frame_verification_error"] = "完成帧未在实际 rosbag 中找到"
        elif supports_source_validation(self.metadata):
            try:
                item["source_validation"] = validate_completion(
                    self.current, self.metadata, topic, stamp, exact_ns=stamp,
                    max_camera_skew_ms=getattr(self, "max_camera_skew_ms", 30),
                    max_source_skew_ms=getattr(self, "max_source_skew_ms", 30))
            except ValueError as error:
                item["source_validation_error"] = str(error)
                item["source_validation_details"] = getattr(error, "details", {})
        item.setdefault("validation_reviews", []).append({
            "reviewed_at_unix_ns": time.time_ns(), "previous_validation": previous})
        self._write_current()
        return {"annotation": item, "active_skill": self.metadata.get("active_skill")}

    def _terminate(self):
        if self.process and self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGINT)
            try:
                self.process.wait(timeout=30)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGTERM)
                self.process.wait(timeout=5)
        self.process = None
        for subscription in getattr(self, "_master_subscriptions", []):
            node = getattr(self, "node", None)
            if node is not None:
                try:
                    node.destroy_subscription(subscription)
                except Exception:
                    pass
        self._master_subscriptions = []

    def _close_open_holds(self, end_ns, reason):
        for segment in self.metadata.get("hold_segments", []):
            if segment.get("end_unix_ns") is None:
                segment["end_unix_ns"] = end_ns
                segment["duration_ns"] = end_ns - segment["start_unix_ns"]
                segment["end_reason"] = reason

    def stop(self, reason="manual"):
        if self.current is None or self.metadata["state"] != "recording":
            raise ValueError("没有正在录制的 Episode")
        self._sample_clock()
        self._close_open_holds(time.time_ns(), "recording_stopped")
        if self.process is None or self.process.poll() is not None:
            self.metadata["state"] = "interrupted"
            self.metadata["error"] = "rosbag2 已意外退出"
            self._event("interrupted")
            raise ValueError(self.metadata["error"])
        stopped = time.time_ns()
        self.metadata["stop_reason"] = reason
        if (getattr(self, "auto_trim_stale_tail", False)
                and self.metadata.get("mode") == "skill"
                and self.metadata.get("schema_version") == 2):
            now_ns = time.monotonic_ns()
            monitor = {}
            for arm, state in (getattr(self, "_master_monitor", {}) or {}).items():
                last_valid = state.get("last_valid_monotonic_ns")
                last_stale = state.get("last_stale_monotonic_ns")
                monitor[arm] = {
                    "last_valid_age_ns": int(now_ns - last_valid) if last_valid is not None else None,
                    "last_stale_age_ns": int(now_ns - last_stale) if last_stale is not None else None,
                    "stale_count": int(state.get("stale_count", 0)),
                }
            self.metadata["auto_trim_live_monitor"] = {
                "status_poll_sec": RECORDER_STATUS_POLL_SEC,
                "freshness_ns": LIVE_STALE_FRESHNESS_NS,
                "grace_ns": int(getattr(self, "stale_stop_grace_sec", 1.0) * 1_000_000_000),
                "elapsed_ns": self._terminal_stale_elapsed_ns(now_ns),
                "arms": monitor,
            }
        self.metadata["stop_requested_at_unix_ns"] = stopped
        self.metadata["recorded_duration_sec"] = round(
            ((time.monotonic_ns() - self.ready_monotonic_ns) if self.ready_monotonic_ns is not None
             else (stopped - self.metadata["ready_at_unix_ns"])) / 1_000_000_000, 3)
        self._event("stopped", reason=reason)
        time.sleep(0.15)
        self._terminate()
        self.deadline_monotonic_ns = None
        self.metadata["stopped_at_unix_ns"] = time.time_ns()
        try:
            self.metadata["bag_validation"] = inspect_bag(
                self.current / "bag", self.topics, self.frame_topics, self.expected_fps,
                self.metadata["required_topics"])
            self.metadata["state"] = "stopped"
            auto_trim = (reason == "stale_tail"
                         and getattr(self, "auto_trim_stale_tail", False)
                         and self.metadata.get("schema_version") == 2
                         and self.metadata.get("mode") == "skill"
                         and self.metadata.get("skill") in SKILLS)
            arm_mode = self.metadata.get("task_metadata", {}).get("arm_mode")
            stale_boundary = None
            action_anchor = None
            if auto_trim and arm_mode in ("LEFT", "RIGHT", "BIMANUAL"):
                ready = self.metadata["ready_at_unix_ns"]
                stop_requested = self.metadata["stop_requested_at_unix_ns"]
                arms = ("left", "right") if arm_mode == "BIMANUAL" else (arm_mode.lower(),)
                boundaries = {arm: terminal_stale_master_target(
                    self.current / "bag", arm, ready, stop_requested) for arm in arms}
                if all(boundaries.values()):
                    boundary = min(item["last_valid_header_ns"] for item in boundaries.values())
                    raw_stopped = self.metadata["stopped_at_unix_ns"]
                    self.metadata["raw_stopped_at_unix_ns"] = raw_stopped
                    self.metadata["trim_boundary_ns"] = boundary
                    self.metadata["trimmed_tail_ns"] = max(0, raw_stopped - boundary)
                    self.metadata["auto_trim_source"] = "terminal_stale_master_target"
                    self.metadata["stopped_at_unix_ns"] = boundary
                    self.metadata["recorded_duration_sec"] = round(
                        (boundary - ready) / 1_000_000_000, 3)
                    if len(arms) == 1:
                        stale_boundary = boundaries[arms[0]]
                        action_anchor = last_valid_master_target(
                            self.current / "bag", arms[0], ready, stop_requested)
                    else:
                        self.metadata["auto_trim_stale_boundaries"] = boundaries
                else:
                    self.metadata["auto_trim_error"] = "未找到可确认的末尾 stale 边界，未裁切"
            if self.metadata.get("schema_version") == 2:
                for item in list(self.metadata.get("annotations", [])):
                    if item.get("kind") != "skill_complete" or item.get("skill_result") != "success":
                        continue
                    try:
                        if not recorded_header_exists(
                                self.current / "bag", item["source_topic"], item["header_stamp_ns"],
                                self.metadata["ready_at_unix_ns"],
                                self.metadata["stop_requested_at_unix_ns"]):
                            raise ValueError("点击成功时的相机帧未写入本条 rosbag")
                        item["verified_in_bag"] = True
                        if supports_source_validation(self.metadata):
                            try:
                                item["source_validation"] = validate_completion(
                                    self.current, self.metadata, item["source_topic"],
                                    item["header_stamp_ns"], exact_ns=item["header_stamp_ns"],
                                    max_camera_skew_ms=getattr(self, "max_camera_skew_ms", 30),
                                    max_source_skew_ms=getattr(self, "max_source_skew_ms", 30))
                            except ValueError as error:
                                item.pop("source_validation", None)
                                item["source_validation_error"] = str(error)
                                item["source_validation_details"] = getattr(error, "details", {})
                    except (ValueError, KeyError, sqlite3.Error) as error:
                        if self.metadata.get("mode") == "skill":
                            item["verified_in_bag"] = False
                            item["frame_verification_error"] = str(error)
                            self.metadata["completion_review_required"] = True
                            self.metadata["auto_completion_error"] = str(error)
                            continue
                        item["frame_resolution_required"] = True
                        item["frame_verification_error"] = str(error)
                        item.pop("source_topic", None)
                        item.pop("header_stamp_ns", None)
            if (reason in ("duration_limit", "stale_tail")
                    and self.metadata.get("schema_version") == 2
                    and self.metadata.get("mode") == "skill"
                    and self.metadata.get("skill") in SKILLS
                    and not any(item.get("kind") == "skill_complete"
                                for item in self.metadata.get("annotations", []))):
                color_topics = [topic for topic in self.frame_topics
                                if topic.endswith("/color/image_raw")]
                color_topics.sort(key=lambda topic: ("camera_top" not in topic, topic))
                try:
                    if not color_topics:
                        raise ValueError("没有配置彩色相机帧话题")
                    ready = self.metadata["ready_at_unix_ns"]
                    stop_requested = self.metadata["stop_requested_at_unix_ns"]
                    if arm_mode in ("LEFT", "RIGHT") and stale_boundary is None:
                        stale_boundary = terminal_stale_master_target(
                            self.current / "bag", arm_mode.lower(), ready, stop_requested)
                        action_anchor = last_valid_master_target(
                            self.current / "bag", arm_mode.lower(), ready, stop_requested)
                    origin = "duration_limit_last_frame"
                    trimmed_boundary = self.metadata.get('trim_boundary_ns')
                    candidate_boundary = (trimmed_boundary or
                                          (stale_boundary['last_valid_header_ns'] if stale_boundary
                                           else stop_requested))
                    before_boundary = bool(trimmed_boundary or stale_boundary)
                    if before_boundary:
                        origin = ("stale_tail_before_terminal_stale" if reason == "stale_tail"
                                  else "duration_limit_before_terminal_stale")
                        self.metadata["auto_completion_stale_boundary"] = stale_boundary
                        self.metadata["auto_completion_boundary_policy"] = (
                            "auto_trim_terminal_stale_convention" if reason == "stale_tail"
                            else "operator_terminal_stale_convention")
                    source_validation = None
                    validation_error = None
                    validation_details = {}
                    if supports_source_validation(self.metadata):
                        try:
                            source_validation = validate_completion(
                                self.current, self.metadata, color_topics[0], candidate_boundary,
                                strictly_before=before_boundary,
                                candidate_topics=color_topics,
                                max_camera_skew_ms=getattr(self, "max_camera_skew_ms", 30),
                                max_source_skew_ms=getattr(self, "max_source_skew_ms", 30))
                        except ValueError as error:
                            validation_error = str(error)
                            validation_details = getattr(error, "details", {})
                    if source_validation is not None:
                        candidate_topic = source_validation["source_topic"]
                        stamp = source_validation["header_stamp_ns"]
                        received = header_receive_ns(self.current / "bag", candidate_topic, stamp)
                    elif before_boundary:
                        candidate_topic = color_topics[0]
                        received, stamp = recorded_frame_before_header(
                            self.current / "bag", color_topics[0], candidate_boundary,
                            ready, stop_requested)
                    else:
                        candidate_topic = color_topics[0]
                        received, stamp = recorded_frame_near(
                            self.current / "bag", color_topics[0], stop_requested, ready, stop_requested)
                    if validation_details.get("header_stamp_ns") != stamp:
                        validation_details = {}
                    candidate = {
                        "source_topic": candidate_topic,
                        "header_stamp_ns": stamp,
                        "matched_offset_sec": round((received - ready) / 1_000_000_000, 3),
                        "candidate_origin": origin,
                        "selection_policy": (source_validation or {}).get(
                            "selection_policy", "unqualified_single_camera_fallback"),
                        "stale_boundary": stale_boundary,
                        "source_validation": source_validation,
                        "source_validation_error": validation_error,
                        "source_validation_details": validation_details,
                        "contiguous_frames": (source_validation or validation_details).get("contiguous_frames"),
                        "window_count": (source_validation or validation_details).get("window_count"),
                        "segment_index": (source_validation or validation_details).get("segment_index"),
                        "reviewed_at_unix_ns": time.time_ns(),
                    }
                    self.metadata["auto_completion_candidate"] = candidate
                    self.metadata["completion_review_required"] = True
                    if action_anchor:
                        candidate["tail_gap_ms"] = round(
                            (stop_requested - action_anchor[0]) / 1_000_000, 1)
                        self.metadata["auto_completion_tail_gap_ms"] = candidate["tail_gap_ms"]
                        candidate["master_header_ns"] = action_anchor[0]
                        self.metadata["auto_completion_master_header_ns"] = action_anchor[0]
                    arms = ("left", "right") if arm_mode == "BIMANUAL" else (
                        arm_mode.lower(),) if arm_mode in ("LEFT", "RIGHT") else ()
                    source_gaps = {}
                    for arm in arms:
                        topic = f"/piper_direct/{arm}/master_target"
                        last_receive = latest_topic_receive_before(
                            self.current / "bag", topic, received, ready)
                        source_gaps[arm] = (None if last_receive is None else
                                            round((received - last_receive) / 1_000_000, 1))
                    candidate["master_gap_ms"] = source_gaps
                    self.metadata["auto_completion_master_gap_ms"] = source_gaps
                    if any(gap is None or gap > 100 for gap in source_gaps.values()):
                        self.metadata["auto_completion_source_warning"] = (
                            "自动完成候选附近未收到持续主端目标；可保存实际结果，但该帧需复核"
                        )
                    if reason == "stale_tail":
                        self.metadata["auto_completion_source_warning"] = (
                            "末尾 stale 后无动作输出，已自动停止并裁切到最后一个有效主目标；候选完成帧需操作员确认"
                        )
                    elif origin == "duration_limit_before_terminal_stale":
                        self.metadata["auto_completion_source_warning"] = (
                            "按采集约定选用末尾持续 stale 前的原始图像；仅作为候选，需操作员确认"
                        )
                except (ValueError, sqlite3.Error) as error:
                    self.metadata["auto_completion_error"] = str(error)
                    self.metadata["completion_review_required"] = True
            if (self.metadata.get("schema_version") == 2 and self.metadata.get("mode") == "skill"
                    and not any(item.get("kind") == "skill_complete"
                                for item in self.metadata.get("annotations", []))):
                self.metadata["completion_review_required"] = True
        except (ValueError, sqlite3.Error) as error:
            self.metadata["state"] = "failed"
            self.metadata["error"] = str(error)
            self._event("failed")
        self._write_current()
        return self.status()

    def save(self, outcome=None, save_dir=None, annotation=None):
        if self.current is None or self.metadata["state"] != "stopped":
            raise ValueError("只能保存已停止且通过完整性核验的 Episode")
        if outcome is not None and outcome not in ("success", "failure", "unknown"):
            raise ValueError("outcome 必须为 success、failure 或 unknown")
        labels = [item["value"] for item in self.metadata.get("annotations", [])
                  if item["kind"] == "outcome"]
        if labels and outcome is not None and outcome != labels[0]:
            raise ValueError("保存结果与录制中结果标签冲突")
        if labels and outcome is None:
            outcome = labels[0]
        context = self.metadata.get("task_metadata", {}).get("vlm")
        if context and annotation is None:
            raise ValueError("VLM 技能片段保存前须填写人工结果标注")
        if annotation is not None:
            if not context:
                raise ValueError("仅 VLM 技能片段可保存结果标注")
            annotation = validate_annotation(annotation)
            if outcome is not None and outcome != annotation["outcome"]:
                raise ValueError("结果标签与人工标注不一致")
            outcome = annotation["outcome"]
        self.metadata["completion_validation"] = assess_training_eligibility(self.metadata, outcome)
        self.metadata["training_eligibility"] = {
            "eligible": False, "status": "pending_independent_evidence_and_audit",
            "reasons": ["原始采集保存不授予 SFT 导出或训练资格"]}
        if save_dir is not None and (not isinstance(save_dir, str) or not save_dir.strip()
                                     or not Path(save_dir).expanduser().is_absolute()):
            raise ValueError("save_dir 必须为非空的绝对目录路径")
        destination_root = Path(save_dir).expanduser().resolve() if save_dir else self.root
        if destination_root == self.staging or self.staging in destination_root.parents:
            raise ValueError("保存目录不能位于录制暂存目录内")
        inspect_bag(self.current / "bag", self.metadata["topics"],
                    self.metadata.get("frame_topics", []), self.metadata.get("expected_fps", {}),
                    self.metadata.get("required_topics"))
        destination_root.mkdir(parents=True, exist_ok=True)
        counter = self.root / ".next_id"
        stored = int(counter.read_text()) if counter.exists() else 1
        existing = [int(p.name[8:]) for p in self._saved_paths() if p.name[8:].isdigit()]
        number = max([stored, *(value + 1 for value in existing)])
        destination = destination_root / f"episode_{number:06d}"
        if destination.exists() or destination.is_symlink():
            raise ValueError(f"保存目标已存在: {destination}")
        same_device = self.current.stat().st_dev == destination_root.stat().st_dev
        if not same_device:
            required = sum(path.stat().st_size for path in self.current.rglob("*") if path.is_file())
            if shutil.disk_usage(destination_root).free < required + self.min_free_bytes:
                raise ValueError("目标磁盘剩余空间不足以保存 Episode")
        previous = copy.deepcopy(self.metadata)
        self.metadata["state"] = "saved"
        self.metadata["episode_number"] = number
        self.metadata["outcome"] = outcome
        add_save_outcome_annotation(self.metadata, outcome, time.time_ns())
        if annotation is not None:
            annotation["annotated_at_unix_ns"] = time.time_ns()
            self.metadata["vlm_annotation"] = annotation
        self.metadata["saved_at_unix_ns"] = time.time_ns()
        self.metadata["save_directory"] = str(destination_root)
        event = {"episode_id": self.metadata["episode_id"], "event": "saved",
                 "time": time.time_ns(), "time_unit": "unix_ns"}
        self.metadata.setdefault("events", []).append(event)
        self._write_current()
        temporary = destination_root / f".{destination.name}.copy-{uuid.uuid4().hex}"
        try:
            if destination_root != self.root:
                index = self.root / ".episode_index.json"
                paths = load_json(index)["paths"] if index.exists() else []
                atomic_json(index, {"paths": [*paths, str(destination)]})
            atomic_json(counter, number + 1)
            if same_device:
                self.current.rename(destination)
            else:
                shutil.copytree(self.current, temporary)
                temporary.rename(destination)
                cleanup = self.root / f".saved_cleanup_{self.current.name}_{uuid.uuid4().hex}"
                self.current.rename(cleanup)
                shutil.rmtree(cleanup, ignore_errors=True)
        except (OSError, ValueError):
            if temporary.exists():
                shutil.rmtree(temporary, ignore_errors=True)
            if self.current.exists():
                if destination.exists():
                    shutil.rmtree(destination, ignore_errors=True)
                self.metadata = previous
                self._write_current()
            raise
        self.current = None
        self.event_publisher.publish(String(data=json.dumps(event, ensure_ascii=False)))
        return {"state": "saved", "path": str(destination), "episode_number": number}

    def discard(self):
        if self.current is not None:
            self._terminate()
            self.metadata["state"] = "discarded"
            self._event("discarded")
            metadata = self.metadata
            shutil.rmtree(self.current)
            atomic_json(self.discarded / f"{metadata['episode_id']}.json", metadata)
            self.current = None
            return {"state": "discarded"}
        directories = sorted(p for p in self.staging.iterdir() if p.is_dir())
        if not directories:
            raise ValueError("没有可丢弃的 Episode")
        for directory in directories:
            metadata = load_json(directory / "episode.json")
            metadata["state"] = "discarded"
            event = {"episode_id": metadata.get("episode_id", directory.name),
                     "event": "discarded", "time": time.time_ns(), "time_unit": "unix_ns"}
            metadata.setdefault("events", []).append(event)
            self.event_publisher.publish(String(data=json.dumps(event, ensure_ascii=False)))
            shutil.rmtree(directory)
            atomic_json(self.discarded / f"{event['episode_id']}.json", metadata)
        return {"state": "discarded", "count": len(directories)}

    def _on_master_target(self, arm, message):
        if self.metadata.get("state") != "recording":
            return
        kind = classify_master_target(message, arm)
        state = getattr(self, "_master_monitor", {}).setdefault(
            arm, {"last_valid_monotonic_ns": None, "last_stale_monotonic_ns": None,
                  "stale_count": 0})
        received = time.monotonic_ns()
        if kind == "valid":
            state.update(last_valid_monotonic_ns=received,
                         last_stale_monotonic_ns=None, stale_count=0)
        elif kind == "stale":
            state["last_stale_monotonic_ns"] = received
            state["stale_count"] = state.get("stale_count", 0) + 1
        else:
            state["last_stale_monotonic_ns"] = None
            state["stale_count"] = 0

    def completion_preview(self):
        if self.current is None or self.metadata.get('state') != 'stopped':
            raise ValueError('停止后才能查看已录制的完成画面')
        confirmed = [a for a in self.metadata.get('annotations', [])
                     if a.get('kind') == 'skill_complete']
        frame = confirmed[0] if len(confirmed) == 1 else self.metadata.get('auto_completion_candidate')
        if not frame or frame.get('source_topic') not in self.frame_topics:
            raise ValueError('没有可复核的完成画面')
        result = recorded_image_preview(self.current / 'bag', frame['source_topic'], frame['header_stamp_ns'])
        return {**result, 'episode_id': self.metadata['episode_id'],
                'header_offset_sec': (frame['header_stamp_ns'] - self.metadata['ready_at_unix_ns']) / 1e9}

    def _terminal_stale_elapsed_ns(self, now_ns):
        if not getattr(self, "auto_trim_stale_tail", False):
            return 0
        if any(item.get("end_unix_ns") is None
               for item in (self.metadata.get("hold_segments") or [])):
            return 0
        arm_mode = self.metadata.get("task_metadata", {}).get("arm_mode")
        arms = ("left", "right") if arm_mode == "BIMANUAL" else (
            (arm_mode.lower(),) if arm_mode in ("LEFT", "RIGHT") else ())
        states = getattr(self, "_master_monitor", {})
        if not arms or not all(arm in states for arm in arms):
            return 0
        elapsed = []
        for arm in arms:
            state = states[arm]
            last_valid = state.get("last_valid_monotonic_ns")
            last_stale = state.get("last_stale_monotonic_ns")
            if (last_valid is None or last_stale is None
                    or state.get("stale_count", 0) < 2
                    or now_ns - last_stale > LIVE_STALE_FRESHNESS_NS):
                return 0
            elapsed.append(now_ns - last_valid)
        return min(elapsed) if elapsed else 0

    def status(self):
        if self.current is not None:
            if self.metadata["state"] == "recording":
                self._sample_clock()
            if self.process and self.process.poll() is not None and self.metadata["state"] == "recording":
                self.metadata["state"] = "interrupted"
                self.metadata["error"] = "rosbag2 意外退出"
                self._event("interrupted")
            if self.metadata["state"] == "recording" and self._available() < self.min_free_bytes:
                self.metadata["state"] = "failed"
                self.metadata["error"] = "剩余磁盘空间低于配置下限，录制已停止"
                self._event("failed")
                time.sleep(0.15)
                self._terminate()
            if (self.metadata["state"] == "recording"
                    and getattr(self, "auto_trim_stale_tail", False)
                    and self.metadata.get("mode") == "skill"
                    and self.metadata.get("schema_version") == 2
                    and self.metadata.get("skill") in SKILLS):
                elapsed = self._terminal_stale_elapsed_ns(time.monotonic_ns())
                if elapsed >= int(getattr(self, "stale_stop_grace_sec", 1.0) * 1_000_000_000):
                    return self.stop(reason="stale_tail")
            deadline = self.deadline_monotonic_ns
            if self.metadata["state"] == "recording" and deadline and time.monotonic_ns() >= deadline:
                return self.stop(reason="duration_limit")
            return {"state": self.metadata["state"], "path": str(self.current),
                    "metadata": self.metadata, "free_bytes": self._available(),
                    "remaining_sec": max(0, (deadline - time.monotonic_ns()) / 1_000_000_000) if deadline else None}
        pending = [str(p) for p in self.staging.iterdir() if p.is_dir()]
        return {"state": "idle", "pending": pending, "free_bytes": self._available()}

    def _sample_clock(self):
        now_ns = time.time_ns()
        offset_ns = now_ns - time.monotonic_ns()
        previous = self._last_clock_offset_ns
        self._last_clock_offset_ns = offset_ns
        if previous is not None and abs(offset_ns - previous) >= CLOCK_STEP_THRESHOLD_NS:
            self.metadata.setdefault("clock_adjustments", []).append({
                "observed_at_unix_ns": now_ns, "offset_delta_ns": offset_ns - previous})
            self._write_current()

    def _saved_paths(self):
        paths = list(self.root.glob("episode_[0-9]*"))
        index = self.root / ".episode_index.json"
        if index.exists():
            paths.extend(Path(path) for path in load_json(index)["paths"])
        return list(dict.fromkeys(paths))

    def list(self):
        episodes = []
        for directory in self._saved_paths():
            metadata_file = directory / "episode.json"
            if not metadata_file.is_file():
                continue
            metadata = load_json(metadata_file)
            episodes.append({key: metadata.get(key) for key in (
                "episode_id", "episode_number", "state", "mode", "task", "skill",
                "outcome", "started_at_unix_ns", "saved_at_unix_ns",
                "duration_limit_sec", "recorded_duration_sec", "stop_reason")})
            episodes[-1]["language_instruction"] = metadata.get("task_metadata", {}).get("language_instruction")
            episodes[-1]["vlm"] = metadata.get("task_metadata", {}).get("vlm")
            episodes[-1]["vlm_annotation"] = metadata.get("vlm_annotation")
            episodes[-1]["path"] = str(directory)
        episodes.sort(key=lambda entry: entry.get("episode_number") or 0, reverse=True)
        return {"episodes": episodes}

    def create_plan(self, value):
        plan = validate_plan(value)
        plan["schema_version"] = 1
        plan["plan_id"] = f"plan_{uuid.uuid4().hex}"
        plan["task_group_id"] = plan["task_group_id"] or uuid.uuid4().hex
        plan["source"] = "human"
        plan["created_at_unix_ns"] = time.time_ns()
        atomic_json(self.plans_dir / f'{plan["plan_id"]}.json', plan)
        return plan

    def list_plans(self):
        plans = [load_json(path) for path in self.plans_dir.glob("plan_*.json")]
        plans.sort(key=lambda item: item["created_at_unix_ns"], reverse=True)
        return {"plans": plans}

    def export_vlm(self):
        transitions = []
        for directory in self._saved_paths():
            path = directory / "episode.json"
            if not path.is_file():
                continue
            metadata = load_json(path)
            context = metadata.get("task_metadata", {}).get("vlm")
            if not context or not metadata.get("vlm_annotation"):
                continue
            if metadata.get("state") != "saved":
                raise ValueError("VLM 导出包含未完成保存的 Episode")
            if context["plan_id"]:
                plan_file = self.plans_dir / f'{context["plan_id"]}.json'
                if not plan_file.is_file():
                    raise ValueError("VLM 导出的关联 plan_id 不存在")
                plan = load_json(plan_file)
                if context["task_group_id"] != plan["task_group_id"]:
                    raise ValueError("VLM 导出的 task_group_id 与规划样本不一致")
                validate_plan_link(plan, context, metadata["skill"])
            transitions.append({
                "episode_id": metadata["episode_id"], "task_group_id": context["task_group_id"],
                "plan_id": context["plan_id"], "skill": metadata["skill"],
                "context": context, "human_annotation": metadata["vlm_annotation"],
                "rosbag_path": str(directory / "bag"), "episode_json_path": str(path),
                "topics": metadata["topics"], "bag_validation": metadata.get("bag_validation"),
                "started_at_unix_ns": metadata["started_at_unix_ns"],
                "ready_at_unix_ns": metadata.get("ready_at_unix_ns"),
                "stopped_at_unix_ns": metadata.get("stopped_at_unix_ns"),
                "saved_at_unix_ns": metadata.get("saved_at_unix_ns"),
            })
        return {"schema_version": 1, "planning_samples": self.list_plans()["plans"],
                "transition_samples": transitions}

    def close(self):
        if self.process:
            if self.current and self.metadata["state"] == "recording":
                self._event("interrupted")
                time.sleep(0.15)
            self._terminate()
            if self.current:
                self.metadata["state"] = "interrupted"
                self._write_current()
        self.node.destroy_node()
        rclpy.shutdown()
        self.lock.close()


def serve(config):
    rclpy.init()
    recorder = Recorder(config)
    path = recorder.socket_path
    if path.exists():
        path.unlink()
    server = socket.socket(socket.AF_UNIX)
    server.bind(str(path))
    os.chmod(path, 0o600)
    server.listen(8)
    server.settimeout(RECORDER_STATUS_POLL_SEC)
    running = True

    def shutdown(_signum, _frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)
    try:
        while running:
            # Only recording has multiple high-rate monitor inputs. Idle polling
            # must not busy-probe an empty executor dozens of times per cycle.
            drain_ros_callbacks(recorder.node, limit=8 if recorder.process is not None else 1)
            try:
                connection, _ = server.accept()
            except socket.timeout:
                recorder.status()
                continue
            with connection:
                try:
                    request = json.loads(connection.recv(1_000_000).decode())
                    action = request.pop("action")
                    if action not in ("start", "stop", "save", "discard", "status", "list",
                                      "plan_create", "plans", "vlm_export", "annotate", "recheck_completion",
                                      "completion_preview"):
                        raise ValueError("未知操作")
                    if action == "start":
                        result = recorder.start(request)
                    elif action == "save":
                        result = recorder.save(request.get("outcome"), request.get("save_dir"),
                                               request.get("annotation"))
                    elif action == "plan_create":
                        result = recorder.create_plan(request.get("plan"))
                    elif action == "plans":
                        result = recorder.list_plans()
                    elif action == "vlm_export":
                        result = recorder.export_vlm()
                    elif action == "annotate":
                        result = recorder.annotate(request)
                    else:
                        result = getattr(recorder, action)()
                    response = {"ok": True, "result": result}
                except (ValueError, OSError, KeyError, json.JSONDecodeError) as error:
                    response = {"ok": False, "error": str(error)}
                connection.sendall(json.dumps(response, ensure_ascii=False).encode())
    finally:
        server.close()
        path.unlink(missing_ok=True)
        recorder.close()


def main():
    parser = argparse.ArgumentParser(description="ROS 2 Episode SQLite 录制器")
    parser.add_argument("action", choices=["serve", "start", "stop", "save", "discard", "status", "list",
                                           "plan_create", "plans", "vlm_export", "annotate", "recheck_completion"])
    parser.add_argument("--config", required=True, help="JSON 配置文件")
    parser.add_argument("--mode", choices=["task", "skill"])
    parser.add_argument("--task")
    parser.add_argument("--skill")
    parser.add_argument("--metadata", help="JSON 任务元数据文件")
    parser.add_argument("--duration-sec", type=float, help="本条 Episode 自动停止的秒数（1 至 86400）")
    parser.add_argument("--outcome", choices=["success", "failure", "unknown"], help="保存时的结果标签")
    parser.add_argument("--annotation", help="JSON 人工结果标注文件")
    parser.add_argument("--plan", help="JSON 规划样本文件")
    parser.add_argument("--save-dir", help="保存 Episode 的绝对目录路径")
    parser.add_argument("--kind", choices=ANNOTATION_KINDS)
    parser.add_argument("--value")
    parser.add_argument("--source-topic")
    parser.add_argument("--header-stamp-ns", type=int)
    args = parser.parse_args()
    config = load_json(args.config)
    if args.action == "serve":
        serve(config)
        return
    root = Path(config["output_dir"]).expanduser().resolve()
    path = Path(config.get("socket_path", str(root / "recorder.sock"))).expanduser().resolve()
    request = {"action": args.action}
    if args.action == "start":
        request.update(mode=args.mode, task=args.task, skill=args.skill,
                       metadata=load_json(args.metadata) if args.metadata else {},
                       duration_sec=args.duration_sec)
    if args.action == "save":
        request["outcome"] = args.outcome
        request["save_dir"] = args.save_dir
        request["annotation"] = load_json(args.annotation) if args.annotation else None
    if args.action == "plan_create":
        request["plan"] = load_json(args.plan) if args.plan else None
    if args.action == "annotate":
        request.update(kind=args.kind, value=args.value, source_topic=args.source_topic,
                       header_stamp_ns=args.header_stamp_ns)
    with socket.socket(socket.AF_UNIX) as connection:
        connection.connect(str(path))
        connection.sendall(json.dumps(request, ensure_ascii=False).encode())
        chunks = []
        while chunk := connection.recv(1_000_000):
            chunks.append(chunk)
        response = json.loads(b"".join(chunks).decode())
    print(json.dumps(response, ensure_ascii=False, indent=2))
    if not response["ok"]:
        sys.exit(1)


if __name__ == "__main__":
    main()
