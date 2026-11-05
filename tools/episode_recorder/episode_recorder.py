#!/usr/bin/env python3
"""只读 Episode 录制服务与 CLI。运行：python3 episode_recorder.py serve --config config.json；另见 --help。"""

import argparse
import copy
import fcntl
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
from vlm_annotations import (required_text, validate_annotation, validate_plan,
                             validate_plan_link, validate_skill_context)


SKILLS = ("pick_and_place_banana_in_box", "flip_box_lid_bimanual", "latch_box_lid_bimanual")
ANNOTATION_KINDS = ("skill_start", "skill_complete", "outcome", "recovery_trigger")
CLOCK_STEP_THRESHOLD_NS = 50_000_000


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


class Recorder:
    def __init__(self, config):
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

    def _graph_ready(self, node_name):
        time.sleep(0.1)
        for topic in self.metadata.get("required_topics", self.topics):
            pubs = self.node.get_publishers_info_by_topic(topic)
            subs = self.node.get_subscriptions_info_by_topic(topic)
            if not pubs or not any(info.node_name == node_name for info in subs):
                return False
        return True

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
        self.ready_monotonic_ns = None
        self.deadline_monotonic_ns = None
        self.current = self.staging / uuid.uuid4().hex
        self.current.mkdir()
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
            "started_at_unix_ns": time.time_ns(),
            "timestamp_note": "rosbag2 messages.timestamp 是接收时间；原始消息 header.stamp 保留在 CDR 消息中",
        }
        self._write_current()
        node_name = f"piper_episode_{os.getpid()}"
        command = ["ros2", "bag", "record", "--storage", "sqlite3", "--max-cache-size",
                   str(self.rosbag_cache_bytes),
                   "--node-name", node_name, "-o", str(self.current / "bag"), *self.topics]
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
        while time.monotonic() < deadline:
            if self.process.poll() is not None:
                break
            if self._graph_ready(node_name) and (self.current / "bag").is_dir():
                self.metadata["state"] = "recording"
                ready = time.time_ns()
                self.metadata["ready_at_unix_ns"] = ready
                self._last_clock_offset_ns = ready - time.monotonic_ns()
                self.metadata["clock_adjustments"] = []
                if duration is not None:
                    duration_ns = round(duration * 1_000_000_000)
                    self.ready_monotonic_ns = time.monotonic_ns()
                    self.deadline_monotonic_ns = self.ready_monotonic_ns + duration_ns
                    self.metadata["recording_deadline_unix_ns"] = ready + duration_ns
                self._event("started")
                return self.status()
        self._terminate()
        self.metadata["state"] = "failed"
        self.metadata["error"] = "rosbag2 未在超时内订阅全部配置话题；见 rosbag.log"
        self._event("failed")
        raise ValueError(self.metadata["error"])

    def annotate(self, request):
        if self.current is None or self.metadata["state"] != "recording":
            raise ValueError("只能在录制中标注")
        if self.metadata.get("schema_version") != 2:
            raise ValueError("旧版 VLM 技能片段不支持 v2 实时标注")
        kind = request.get("kind")
        value = request.get("value")
        if kind not in ANNOTATION_KINDS:
            raise ValueError("未知标注类型")
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
            next_active = None if self.metadata["mode"] == "task" else active
        elif kind == "outcome":
            if value not in ("success", "failure"):
                raise ValueError("outcome 必须为 success 或 failure")
            if any(item["kind"] == "outcome" for item in self.metadata["annotations"]):
                raise ValueError("结果标签不能重复标注")
        elif not isinstance(value, str) or not value.strip():
            raise ValueError("恢复原因不能为空")
        topic = request.get("source_topic")
        stamp = request.get("header_stamp_ns")
        if (topic is None) != (stamp is None):
            raise ValueError("source_topic 与 header_stamp_ns 必须同时提供")
        if topic is not None and (topic not in self.frame_topics or
                                  not isinstance(stamp, int) or isinstance(stamp, bool) or stamp <= 0):
            raise ValueError("完成帧来源必须是配置中的图像话题及有效 Header 时间戳")
        annotation = {"kind": kind, "value": value, "recorded_at_unix_ns": time.time_ns()}
        if kind == "recovery_trigger":
            annotation["recovery_id"] = uuid.uuid4().hex
            annotation["skill"] = active
        if topic is not None:
            annotation.update(source_topic=topic, header_stamp_ns=stamp)
        else:
            annotation["frame_resolution_required"] = kind == "skill_complete"
        self.metadata["active_skill"] = next_active
        self.metadata["annotations"].append(annotation)
        self._write_current()
        self.event_publisher.publish(String(data=json.dumps({"episode_id": self.metadata["episode_id"],
            "event": "annotation", **annotation}, ensure_ascii=False)))
        return {"annotation": annotation, "active_skill": next_active}

    def _terminate(self):
        if self.process and self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGINT)
            try:
                self.process.wait(timeout=30)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGTERM)
                self.process.wait(timeout=5)
        self.process = None

    def stop(self, reason="manual"):
        if self.current is None or self.metadata["state"] != "recording":
            raise ValueError("没有正在录制的 Episode")
        self._sample_clock()
        if self.process is None or self.process.poll() is not None:
            self.metadata["state"] = "interrupted"
            self.metadata["error"] = "rosbag2 已意外退出"
            self._event("interrupted")
            raise ValueError(self.metadata["error"])
        stopped = time.time_ns()
        self.metadata["stop_reason"] = reason
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
    server.settimeout(0.2)
    running = True

    def shutdown(_signum, _frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)
    try:
        while running:
            rclpy.spin_once(recorder.node, timeout_sec=0)
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
                                      "plan_create", "plans", "vlm_export", "annotate"):
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
                                           "plan_create", "plans", "vlm_export", "annotate"])
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
