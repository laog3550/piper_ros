#!/usr/bin/env python3
"""可选离线运行轨迹落盘。运行：python3 -m unittest discover -s tools/vla_contract -p 'test_runtime_trace.py'。"""

import hashlib
import json
import os
from pathlib import Path
import re
import time
import uuid


VERSION = "piper_runtime_trace/1.0"
IDENTIFIER = re.compile(r"^[A-Za-z0-9_.-]{1,128}$")
SHA256 = re.compile(r"^[0-9a-f]{64}$")


class TraceError(ValueError):
    pass


def observation_summary(observation):
    """只保存帧引用和状态摘要，不复制 RGB 像素。"""
    return {
        "observation_id": observation["observation_id"],
        "observation_timestamp_ns": observation["observation_timestamp_ns"],
        "arm_mode": observation["arm_mode"],
        "calibration_version": observation["calibration_version"],
        "normalization_version": observation["normalization_version"],
        "images": {name: {"uri": frame["uri"], "header_timestamp_ns": frame["header_timestamp_ns"],
                          "frame_id": frame["frame_id"]} for name, frame in observation["images"].items()},
        "state": {name: {"header_timestamp_ns": state["header_timestamp_ns"],
                         "position": list(state["position"])} for name, state in observation["state"].items()},
    }


def derived_recovery_id(run_id, label):
    return "recovery-" + hashlib.sha256(f"{run_id}:{label}".encode("utf-8")).hexdigest()[:32]


def _identifier(value, label, optional=False):
    if optional and value is None:
        return None
    if type(value) is not str or not IDENTIFIER.fullmatch(value):
        raise TraceError(f"{label} 无效")
    return value


def _text_id(value, label, optional=False):
    if optional and value is None:
        return None
    if (type(value) is not str or not value.strip() or len(value) > 128
            or any(ord(character) < 32 for character in value)):
        raise TraceError(f"{label} 无效")
    return value


def _atomic_json(path, value):
    temporary = path.with_name(path.name + ".tmp")
    with os.fdopen(os.open(temporary, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600),
                   "w", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.flush()
        os.fsync(stream.fileno())
    temporary.replace(path)
    descriptor = os.open(path.parent, os.O_DIRECTORY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


class TraceSession:
    """单写入者运行目录；异常退出不恢复，未完成轨迹不得作为训练标签。"""

    def __init__(self, root, *, source_kind, models, links=None,
                 episode_manifest_sha256=None, run_id=None):
        if source_kind not in ("synthetic", "real"):
            raise TraceError("source_kind 必须为 synthetic 或 real")
        if type(models) is not dict or set(models) != {"planner", "policy"}:
            raise TraceError("models 必须绑定 planner 和 policy")
        if any(type(value) is not str or not value.strip() for value in models.values()):
            raise TraceError("模型标识不能为空")
        if type(links) is not dict or not set(links or {}) <= {"episode_id", "task_group_id", "plan_id",
                                                                  "recovery_id"}:
            raise TraceError("关联 ID 字段无效")
        links = {key: _text_id(value, key, optional=True) for key, value in (links or {}).items()}
        if source_kind == "real" and not links.get("episode_id"):
            raise TraceError("真实来源须绑定 episode_id")
        if source_kind == "real" and (type(episode_manifest_sha256) is not str
                                      or not SHA256.fullmatch(episode_manifest_sha256)):
            raise TraceError("真实来源须绑定 Episode manifest SHA-256")
        if episode_manifest_sha256 is not None and (type(episode_manifest_sha256) is not str
                                                    or not SHA256.fullmatch(episode_manifest_sha256)):
            raise TraceError("Episode manifest SHA-256 无效")
        self.run_id = _identifier(run_id or uuid.uuid4().hex, "run_id")
        self.path = Path(root).resolve() / self.run_id
        self.path.mkdir(mode=0o700, parents=True, exist_ok=False)
        self.events = self.path / "events.jsonl"
        self.events.touch(mode=0o600, exist_ok=False)
        self.manifest_path = self.path / "manifest.json"
        self.manifest = {
            "schema_version": VERSION, "run_id": self.run_id,
            "source_kind": source_kind, "episode_manifest_sha256": episode_manifest_sha256,
            "models": dict(models), "links": links,
            "created_at_unix_ns": time.time_ns(), "status": "incomplete",
            "terminal_status": None, "event_count": 0,
            "events_sha256": hashlib.sha256(b"").hexdigest(),
            "training_eligible": False, "real_success_verified": False,
        }
        self._digest = hashlib.sha256()
        self._closed = False
        _atomic_json(self.manifest_path, self.manifest)

    def record(self, event_type, data, *, ids=None, clock_ns=None):
        if self._closed:
            raise TraceError("轨迹已结束")
        _identifier(event_type, "event_type")
        if type(data) is not dict or type(ids) not in (dict, type(None)):
            raise TraceError("事件数据或关联 ID 无效")
        ids = {key: _text_id(value, key, optional=True) for key, value in (ids or {}).items()}
        if not set(ids) <= {"request_id", "step_id", "skill_id", "recovery_id", "observation_id"}:
            raise TraceError("事件关联 ID 字段无效")
        if "recovery_id" not in ids and self.manifest["links"].get("recovery_id") is not None:
            ids["recovery_id"] = self.manifest["links"]["recovery_id"]
        if clock_ns is not None and (type(clock_ns) is not int or clock_ns <= 0):
            raise TraceError("运行时钟无效")
        event = {"schema_version": VERSION, "run_id": self.run_id,
                 "seq": self.manifest["event_count"] + 1, "event_type": event_type,
                 "source_kind": self.manifest["source_kind"],
                 "wall_unix_ns": time.time_ns(), "clock_ns": clock_ns,
                 "ids": ids, "data": data}
        try:
            line = (json.dumps(event, ensure_ascii=False, separators=(",", ":"),
                               allow_nan=False) + "\n").encode("utf-8")
            with self.events.open("ab") as stream:
                stream.write(line)
                stream.flush()
                os.fsync(stream.fileno())
            self._digest.update(line)
            self.manifest["event_count"] += 1
            self.manifest["events_sha256"] = self._digest.hexdigest()
            _atomic_json(self.manifest_path, self.manifest)
        except (OSError, TypeError, ValueError) as exc:
            raise TraceError(f"轨迹写入失败：{type(exc).__name__}") from exc

    def finish(self, terminal_status, *, clock_ns=None):
        if terminal_status not in ("DONE", "HUMAN_REVIEW", "REOBSERVE", "SAFE_STOP", "HUMAN_TAKEOVER"):
            raise TraceError("终态无效")
        self.record("run_terminal", {"status": terminal_status}, clock_ns=clock_ns)
        self.manifest["terminal_status"] = terminal_status
        self.manifest["status"] = "complete"
        try:
            _atomic_json(self.manifest_path, self.manifest)
        except OSError as exc:
            raise TraceError("轨迹结束清单写入失败") from exc
        self._closed = True


def read_trace(directory, *, allow_incomplete=False):
    directory = Path(directory)
    try:
        manifest = json.loads((directory / "manifest.json").read_text(encoding="utf-8"))
        raw = (directory / "events.jsonl").read_bytes()
        if manifest["training_eligible"] is not False or manifest["real_success_verified"] is not False:
            raise TraceError("运行轨迹不能直接成为训练或真实成功标签")
        if manifest["schema_version"] != VERSION or hashlib.sha256(raw).hexdigest() != manifest["events_sha256"]:
            raise TraceError("轨迹摘要不匹配")
        lines = raw.splitlines(keepends=True)
        if len(lines) != manifest["event_count"] or any(not line.endswith(b"\n") for line in lines):
            raise TraceError("轨迹事件数或末行不完整")
        events = [json.loads(line) for line in lines]
        if any(event["seq"] != index or event["run_id"] != manifest["run_id"]
               or event["schema_version"] != VERSION or event["source_kind"] != manifest["source_kind"]
               for index, event in enumerate(events, 1)):
            raise TraceError("轨迹事件引用不一致")
        if manifest["status"] != "complete" and not allow_incomplete:
            raise TraceError("轨迹未完成；不得用于训练")
        if manifest["status"] == "complete" and (not events or events[-1]["event_type"] != "run_terminal"
                                                 or events[-1]["data"]["status"] != manifest["terminal_status"]):
            raise TraceError("终态事件与清单不一致")
        return manifest, events
    except (OSError, KeyError, TypeError, ValueError) as exc:
        if isinstance(exc, TraceError):
            raise
        raise TraceError(f"轨迹读取失败：{type(exc).__name__}") from exc
