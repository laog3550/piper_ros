#!/usr/bin/env python3
"""把已保存的 rosbag2 Episode 转为本地 LeRobot v3。成功 SFT 示例：python to_lerobot.py INPUT OUTPUT --repo-id local/piper --arm-mode LEFT --input-format can-vector --fps 10 --max-age-ms 100 --max-camera-skew-ms 40 --max-source-skew-ms 40 --success-sft；阈值须按现场证据指定。"""

import argparse
from bisect import bisect_right
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import sys
import tempfile

import numpy as np

try:
    from rosbags.rosbag2 import Reader
    from rosbags.typesys import Stores, get_typestore, get_types_from_msg
except ModuleNotFoundError as error:
    if error.name != "rosbags":
        raise
    Reader = None

JOINTS = [f"joint{i}" for i in range(1, 7)] + ["gripper"]
CAMERAS = {
    "top_rgb": "/camera_top/D455/color/image_raw",
    "left_wrist_rgb": "/camera_wrist_left/color/image_raw",
    "right_wrist_rgb": "/camera_wrist_right/color/image_raw",
}
STATES = {arm: f"/joint_{arm}" for arm in ("left", "right")}
ACTIONS = {arm: f"/demonstration/action_{arm}" for arm in ("left", "right")}
CAN_STATES = {arm: f"/piper_vector/{arm}/slave_feedback" for arm in ("left", "right")}
CAN_ACTIONS = {arm: f"/piper_vector/{arm}/master_target" for arm in ("left", "right")}
CAN_IDS = {"master_target": [0x155, 0x156, 0x157, 0x159],
           "slave_feedback": [0x2A5, 0x2A6, 0x2A7, 0x2A8]}
VECTOR_TYPE = "piper_msgs/msg/PiperArmVector"
SKILLS = ("pick_and_place_banana_in_box", "flip_box_lid_bimanual", "latch_box_lid_bimanual")
ACTION_HORIZON = 16
VECTOR_MSG = """std_msgs/Header header
string source_topic
string[7] joint_names
float64[7] position
uint32[4] source_can_ids
builtin_interfaces/Time[4] source_stamps
bool[4] missing
bool[4] stale
bool time_inconsistent
bool valid
bool gripper_mapping_verified
"""
TYPES = {**{topic: "sensor_msgs/msg/Image" for topic in CAMERAS.values()},
         **{topic: "sensor_msgs/msg/JointState" for topic in (*STATES.values(), *ACTIONS.values())},
         **{topic: VECTOR_TYPE for topic in (*CAN_STATES.values(), *CAN_ACTIONS.values())}}
TYPESTORE = None
if Reader is not None:
    TYPESTORE = get_typestore(Stores.ROS2_HUMBLE)
    TYPESTORE.register(get_types_from_msg(VECTOR_MSG, VECTOR_TYPE))
ALIGNMENT_POLICY = "causal_rgb_bundle_v1"
FOLLOWER_ALIGNMENT_POLICY = "causal_follower_next_state_v1"
ACTION_PROVENANCE = {"master_target": "human_demonstration",
                     "follower_next_state": "teleoperation_follower_next_state"}


def fail(message):
    raise ValueError(message)


class InvalidFrame(ValueError):
    """A quality failure may terminate a segment, but never becomes a valid frame."""


SOURCE_DECLARATION_CHECKS = frozenset(("time_domain", "teleoperation_mapping", "action_space",
                                       "action_provenance", "data_convention"))
SOURCE_EVIDENCE_CHECKS = SOURCE_DECLARATION_CHECKS | {"calibration"}
DOWNSTREAM_TRAINING_CHECKS = frozenset(("training_normalization", "independent_audit"))


def source_check_pending(key, check, accepted_checks):
    if not isinstance(check, dict) or not isinstance(check.get("status"), str):
        return True
    if key in DOWNSTREAM_TRAINING_CHECKS:
        return False
    if key in accepted_checks or check["status"] in ("verified", "passed", "accepted"):
        return False
    # These declarations still undergo Header, CAN provenance and vector checks
    # in read_episode. Camera acceptance cannot be inferred from pixel shape.
    return not (key in SOURCE_DECLARATION_CHECKS and check["status"] == "declared_pending_audit")


def training_contract_reasons(meta, expected_mode, accepted_checks=(), action_source="master_target"):
    """Check source export evidence; train statistics are a later dataset asset."""
    fields = meta.get("task_metadata") or {}
    reasons = []
    if not isinstance(fields, dict):
        return ["task_metadata 必须为对象"]
    if expected_mode not in ("LEFT", "RIGHT", "BIMANUAL"):
        reasons.append("arm_mode 无效")
    if action_source not in ACTION_PROVENANCE:
        return [f"未知 action_source={action_source}"]
    for key, expected in (("arm_mode", expected_mode), ("time_domain", "unix_ns"),
                          ("action_space", "absolute_joint_position_7")):
        if fields.get(key) != expected:
            reasons.append(f"缺少显式 {key}={expected} 元数据")
    expected_provenance = ACTION_PROVENANCE[action_source]
    if fields.get("action_provenance") != expected_provenance:
        if "action_provenance" not in accepted_checks:
            reasons.append(f"缺少显式 action_provenance={expected_provenance} 元数据")
    if action_source != "master_target":
        declared_source = fields.get("action_source")
        if declared_source is not None and declared_source != action_source:
            reasons.append(f"action_source={declared_source} 与请求的 {action_source} 不一致")
        elif declared_source is None and "action_provenance" not in accepted_checks:
            reasons.append(f"缺少显式 action_source={action_source} 元数据")
    for key in ("calibration_version", "data_convention_version"):
        if not isinstance(fields.get(key), str) or not fields[key].strip():
            reasons.append(f"缺少 {key} 标识")
    if fields.get("normalization_version") is not None and (
            not isinstance(fields["normalization_version"], str) or not fields["normalization_version"].strip()):
        reasons.append("normalization_version 若提供则必须为非空版本号；未知训练统计请省略字段或设为 null")
    if isinstance(fields.get("normalization_version"), str) and fields["normalization_version"] and (
            fields["normalization_version"] == fields.get("data_convention_version")
            or fields["normalization_version"].startswith("piper_raw_")):
        reasons.append("原始字段约定不能用作训练 normalization_version")
    status = str(fields.get("collection_contract_status", ""))
    if status != "metadata_ready_pending_data_audit" and any(
            word in status for word in ("provisional", "not_ready", "unverified", "pending")):
        reasons.append("采集契约尚未达到源导出条件")
    checks = fields.get("collection_contract_checks") or {}
    if not isinstance(checks, dict):
        reasons.append("collection_contract_checks 必须为对象")
        checks = {}
    for key, check in checks.items():
        if source_check_pending(key, check, accepted_checks):
            reasons.append(f"{key} 尚未验收")
    blockers = fields.get("collection_contract_blockers") or []
    if not isinstance(blockers, list) or any(not isinstance(item, str) for item in blockers):
        reasons.append("collection_contract_blockers 必须为字符串数组")
    else:
        for blocker in blockers:
            owners = [key for key, check in checks.items()
                      if isinstance(check, dict) and check.get("detail") == blocker]
            # Unstructured/unknown blockers stay blocking; only exact details of
            # resolved source checks or deferred training checks can be removed.
            if not owners or any(source_check_pending(key, checks[key], accepted_checks) for key in owners):
                reasons.append(f"采集契约仍有未验收项：{blocker}")
    if meta.get("clock_adjustments"):
        reasons.append("录制期间检测到系统时钟跳变")
    for topic, stats in (meta.get("bag_validation") or {}).get("frame_stats_by_header_stamp", {}).items():
        if stats.get("non_increasing_stamps", 0):
            reasons.append(f"{topic} 的 Header 时间戳不单调")
    return reasons


def normalization_status(fields):
    return {"status": "source_declared_unverified" if fields.get("normalization_version") else "not_computed",
            "verified": False, "required_source_split": "train", "checkpoint_bound": False}


def load_source_acceptance(path):
    raw = path.read_bytes()
    data = json.loads(raw)
    if (not isinstance(data, dict) or data.get("schema_version") != "piper_source_export_acceptance/1"
            or not isinstance(data.get("episodes"), dict)):
        fail("源导出验收文件格式无效")
    return {"path": str(path.resolve()), "sha256": hashlib.sha256(raw).hexdigest(),
            "episodes": data["episodes"]}


def source_acceptance_for_episode(path, meta, acceptance):
    if acceptance is None:
        return None
    record = acceptance["episodes"].get(meta.get("episode_id"))
    if record is None:
        return None
    fields = meta.get("task_metadata") or {}
    if (not isinstance(record, dict) or not isinstance(fields, dict)
            or record.get("source_sha256") != source_digest(path)
            or any(record.get(key) != fields.get(key) or not isinstance(record.get(key), str)
                   for key in ("arm_mode", "calibration_version", "data_convention_version"))
            or not isinstance(record.get("reviewed_by"), str) or not record["reviewed_by"].strip()
            or type(record.get("reviewed_at_unix_ns")) is not int or record["reviewed_at_unix_ns"] <= 0
            or type(meta.get("stopped_at_unix_ns")) is not int
            or record["reviewed_at_unix_ns"] < meta.get("stopped_at_unix_ns", 0)
            or not isinstance(record.get("checks"), dict) or not record["checks"]
            or not set(record["checks"]).issubset(SOURCE_EVIDENCE_CHECKS)):
        fail("源导出验收未绑定当前 Episode 摘要、版本、执行臂及独立复核人")
    for key, check in record["checks"].items():
        if (not isinstance(check, dict) or check.get("status") != "verified"
                or not isinstance(check.get("evidence"), list) or not check["evidence"]):
            fail(f"{key} 源导出验收缺少 verified 状态或独立证据")
        for evidence in check["evidence"]:
            if (not isinstance(evidence, dict) or not isinstance(evidence.get("path"), str)
                    or not isinstance(evidence.get("sha256"), str) or len(evidence["sha256"]) != 64):
                fail(f"{key} 源导出验收证据路径或 SHA-256 无效")
            evidence_path = Path(evidence["path"])
            if not evidence_path.is_absolute():
                evidence_path = Path(acceptance["path"]).parent / evidence_path
            digest = hashlib.sha256()
            with evidence_path.open("rb") as stream:
                for block in iter(lambda: stream.read(1024 * 1024), b""):
                    digest.update(block)
            if digest.hexdigest() != evidence["sha256"]:
                fail(f"{key} 源导出验收证据 SHA-256 不匹配")
    return {"path": acceptance["path"], "sha256": acceptance["sha256"], "record": record}


def stamp(message):
    value = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
    if value <= 0 or message.header.stamp.nanosec >= 1_000_000_000:
        fail("Header 时间戳无效或非 Unix 纳秒")
    return value


def stamp_value(value):
    timestamp = value.sec * 1_000_000_000 + value.nanosec
    if timestamp <= 0 or not 0 <= value.nanosec < 1_000_000_000:
        fail("CAN 来源时间戳无效")
    return timestamp


def vector(message, topic, allow_negative_gripper=False):
    names = list(message.joint_names if topic in CAN_STATES.values() or topic in CAN_ACTIONS.values()
                 else message.name)
    if len(names) != len(set(names)) or set(names) != set(JOINTS) or len(message.position) != len(names):
        fail(f"{topic}: 必须恰有 joint1..joint6、gripper 和对应的 7 个 position")
    by_name = dict(zip(names, message.position))
    values = np.array([by_name[name] for name in JOINTS], dtype=np.float32)
    if not np.isfinite(values).all() or (values[6] < 0 and not allow_negative_gripper):
        fail(f"{topic}: 位置非有限数或夹爪开口为负")
    return values


def can_vector(message, topic, max_can_skew_ns, gripper_calibration=None, *, raw_signed_gripper=False):
    arm, kind = topic.split("/")[2:]
    if (message.source_topic != f"/piper_direct/{arm}/{kind}"
            or message.header.frame_id != f"can_{arm}"
            or list(message.source_can_ids) != CAN_IDS[kind]):
        fail(f"{topic}: CAN 来源话题、接口或 ID 不符")
    values = vector(message, topic, allow_negative_gripper=(gripper_calibration is not None
                                                           or raw_signed_gripper))
    flags = []
    if any(message.missing):
        flags.append("missing")
    if any(message.stale):
        flags.append("stale")
    if message.time_inconsistent:
        flags.append("time_inconsistent")
    if not message.gripper_mapping_verified and gripper_calibration is None:
        flags.append("夹爪映射未核实")
    # The explicit mapping correction only explains valid=false when the original
    # publisher also declared its gripper mapping unverified. Keep both raw flags.
    if not message.valid and (message.gripper_mapping_verified or gripper_calibration is None):
        flags.append("valid=false")
    if flags:
        raise InvalidFrame(f"{topic}: {','.join(flags)}")
    timestamps = [stamp_value(value) for value in message.source_stamps]
    if max(timestamps) != stamp(message) or max(timestamps) - min(timestamps) > max_can_skew_ns:
        raise InvalidFrame(f"{topic}: Header 与四帧来源时间不一致或超窗")
    if gripper_calibration is not None:
        setting = gripper_calibration["streams"][topic]
        corrected = float(values[6]) - setting["closed_reference"]
        if abs(corrected) < 1e-6:
            corrected = 0.0
        bounds = gripper_calibration["valid_range"]
        values[6] = min(bounds["max"], max(bounds["min"], corrected))
    return values, timestamps


def load_gripper_calibration(path):
    data = json.loads(path.read_text(encoding="utf-8"))
    if (data.get("schema_version") != 1 or data.get("unit") != "m"
            or data.get("direction") != "opening_increases_value"
            or data.get("valid_range") != {"min": 0.0, "max": 0.1}
            or data.get("arm_mapping") != {
                arm: {"master_target": CAN_ACTIONS[arm], "slave_feedback": CAN_STATES[arm]}
                for arm in ("left", "right")}
            or set(data.get("streams", {})) != set((*CAN_STATES.values(), *CAN_ACTIONS.values()))):
        fail("采集映射配置的左右主从话题、夹爪单位、范围或四路话题无效")
    for setting in data["streams"].values():
        if (not isinstance(setting, dict)
                or not math.isfinite(setting.get("closed_reference", float("nan")))
                or not math.isfinite(setting.get("observed_open", float("nan")))
                or setting["observed_open"] <= setting["closed_reference"]):
            fail("夹爪采集标定端点无效")
    return data


def image(message, topic, materialize, spool_dir=None):
    if (message.encoding != "rgb8" or message.width != 640 or message.height != 480
            or message.is_bigendian or message.step != 640 * 3
            or len(message.data) != 480 * 640 * 3 or not message.header.frame_id):
        fail(f"{topic}: 需要带 frame_id 的 640×480 连续 rgb8 图像")
    if spool_dir is not None:
        path = spool_dir / f"{hashlib.sha256(topic.encode()).hexdigest()[:12]}_{stamp(message)}.rgb"
        path.write_bytes(message.data.tobytes())
        return path
    return np.frombuffer(message.data, dtype=np.uint8).reshape(480, 640, 3).copy() if materialize else None


def normalize_hold_segments(meta):
    """Read explicit operator hold records; never infer hold from source silence."""
    policy = meta.get("hold_policy") or {}
    if not isinstance(policy, dict):
        fail("hold_policy 必须为对象")
    try:
        max_hold_ns = int(float(policy.get("max_hold_sec", 10.0)) * 1_000_000_000)
        anchor_gap_ns = int(float(policy.get("anchor_max_gap_sec", 2.0)) * 1_000_000_000)
    except (TypeError, ValueError) as error:
        fail(f"hold_policy 时间参数无效：{error}")
    if max_hold_ns <= 0 or anchor_gap_ns <= 0:
        fail("hold_policy 时间参数必须为正")
    segments = []
    for item in meta.get("hold_segments") or []:
        if not isinstance(item, dict):
            fail("hold_segments 项必须为对象")
        start_ns, end_ns = item.get("start_unix_ns"), item.get("end_unix_ns")
        if type(start_ns) is not int or type(end_ns) is not int or end_ns <= start_ns:
            fail("保持段缺少有效起止时间")
        if item.get("source") != "operator_annotation":
            fail("保持段来源必须为 operator_annotation")
        if not isinstance(item.get("hold_id"), str) or not item["hold_id"]:
            fail("保持段缺少 hold_id")
        segments.append({"hold_id": item["hold_id"], "start_ns": start_ns, "end_ns": end_ns,
                         "source": item["source"], "skill": item.get("skill"),
                         "intent": item.get("intent"), "end_reason": item.get("end_reason")})
    segments.sort(key=lambda item: item["start_ns"])
    for left, right in zip(segments, segments[1:]):
        if right["start_ns"] < left["end_ns"]:
            fail("保持段互相重叠")
    return segments, max_hold_ns, anchor_gap_ns


def merge_intervals(intervals):
    merged = []
    for begin, end in sorted(intervals):
        if merged and begin <= merged[-1][1] + 1:
            merged[-1] = (merged[-1][0], max(merged[-1][1], end))
        else:
            merged.append((begin, end))
    return merged


def read_episode(path, arms, fps, max_age_ns, jitter_ns, materialize=False, spool_dir=None,
                 input_format="joint-state", max_can_skew_ns=30_000_000,
                 max_camera_gap_ns=100_000_000, max_camera_skew_ns=30_000_000,
                 extra_frame_topics=(), max_source_skew_ns=None, gripper_calibration=None,
                 extract_valid_segments=False, require_training_contract=True, source_acceptance=None,
                 action_source="master_target",
                 _metadata_override=None, _reader_factory=None, _deserialize=None,
                 _raw_signed_gripper=False, completion_header_range=None):
    if action_source not in ACTION_PROVENANCE:
        fail(f"action_source 无效：{action_source}")
    if _raw_signed_gripper and (_metadata_override is None or require_training_contract
                               or materialize or spool_dir is not None or source_acceptance is not None):
        fail("有符号夹爪原值仅可用于不授予导出资格的只读完成帧检查")
    metadata_path = path / "episode.json"
    if not metadata_path.is_file() or not (path / "bag" / "metadata.yaml").is_file():
        fail(f"{path}: 缺少 episode.json 或 bag/metadata.yaml")
    meta = json.loads(metadata_path.read_text(encoding="utf-8"))
    if _metadata_override is not None:
        if require_training_contract or materialize or spool_dir is not None or source_acceptance is not None:
            fail("停止后元数据覆盖仅可用于不授予导出资格的只读完成帧检查")
        meta = dict(_metadata_override)
    binding = (meta.get("task_metadata") or {}).get("gripper_calibration_evidence")
    if binding and not _raw_signed_gripper:
        if gripper_calibration is None:
            fail("本 Episode 已绑定夹爪零点；请用 --gripper-calibration 指定该标定资产")
        settings_hash = hashlib.sha256(json.dumps(gripper_calibration, sort_keys=True,
                                                separators=(",", ":")).encode()).hexdigest()
        if not isinstance(binding, dict) or settings_hash != binding.get("settings_sha256"):
            fail("夹爪标定与录制时绑定的零点不一致，拒绝混用标定版本")
    hold_segments, hold_max_ns, hold_anchor_gap_ns = normalize_hold_segments(meta)
    if action_source != "master_target" and hold_segments:
        fail("follower_next_state 不接受保持段；从臂反馈本身就是连续动作来源")
    expected_mode = "BIMANUAL" if len(arms) == 2 else arms[0].upper()
    if meta.get("schema_version") not in (1, 2) or meta.get("state") != "saved":
        fail(f"{path}: 仅接受 schema_version=1/2 且 state=saved 的 Episode")
    acceptance = source_acceptance_for_episode(path, meta, source_acceptance)
    accepted_checks = acceptance["record"]["checks"] if acceptance else ()
    contract_reasons = training_contract_reasons(meta, expected_mode, accepted_checks, action_source)
    if require_training_contract and contract_reasons:
        fail(f"{path}: {'；'.join(contract_reasons)}")
    meta["_source_export_acceptance"] = acceptance
    task = meta.get("task")
    if not isinstance(task, str) or not task.strip():
        fail(f"{path}: task 为空")
    if input_format not in ("joint-state", "can-vector"):
        fail("input_format 无效")
    if extract_valid_segments and input_format != "can-vector":
        fail("连续有效片段提取仅适用于 can-vector")
    if extract_valid_segments and max_source_skew_ns is None:
        fail("连续有效片段提取必须显式指定 RGB、状态与示教目标 Header 的总体同步阈值")
    start, stop = meta.get("started_at_unix_ns"), meta.get("stopped_at_unix_ns")
    if type(start) is not int or type(stop) is not int or start <= 0 or stop <= start:
        fail(f"{path}: Episode 录制区间无效")
    state_topics = CAN_STATES if input_format == "can-vector" else STATES
    if action_source != "master_target" and input_format != "can-vector":
        fail("follower_next_state 仅适用于 can-vector")
    action_topics = ((CAN_STATES if action_source == "follower_next_state" else CAN_ACTIONS)
                     if input_format == "can-vector" else ACTIONS)
    camera_topics = [CAMERAS["top_rgb"], *(CAMERAS[f"{arm}_wrist_rgb"] for arm in arms)]
    vector_topics = list(dict.fromkeys([*(state_topics[arm] for arm in arms),
                                        *(action_topics[arm] for arm in arms)]))
    diagnostic_topics = {f"/piper_direct/{arm}/diagnostics" for arm in arms} if input_format == "can-vector" else set()
    transport_diagnostics = {}
    if set(extra_frame_topics) & set(vector_topics):
        fail("完成帧必须引用相机 Image 话题")
    needed = {*camera_topics, *vector_topics, *extra_frame_topics}
    streams = {topic: [] for topic in needed}
    raw_vectors = {topic: {} for topic in vector_topics}
    vector_quality = {topic: {} for topic in vector_topics}
    raw_quality = {topic: {} for topic in vector_topics}
    valid_by_header = {topic: {} for topic in vector_topics}
    rejected_vectors = Counter()
    reader_factory = _reader_factory or Reader
    if reader_factory is None:
        fail("离线转换需要 rosbags；录制端完成帧检查须显式使用 ROS 只读解码器")
    deserialize = _deserialize or TYPESTORE.deserialize_cdr
    with reader_factory(path / "bag") as reader:
        connections = [conn for conn in reader.connections if conn.topic in needed | diagnostic_topics]
        found = {conn.topic for conn in connections}
        if not needed <= found:
            fail(f"{path}: 缺少话题 {sorted(needed - found)}")
        for conn in connections:
            expected_type = ("std_msgs/msg/String" if conn.topic in diagnostic_topics else
                             "sensor_msgs/msg/Image" if conn.topic in extra_frame_topics else TYPES[conn.topic])
            if conn.msgtype != expected_type or conn.ext.serialization_format != "cdr":
                fail(f"{conn.topic}: 消息类型或序列化格式不符")
        for conn, _, raw in reader.messages(connections=connections):
            message = deserialize(raw, conn.msgtype)
            if conn.topic in diagnostic_topics:
                try:
                    data = json.loads(message.data)
                except (TypeError, json.JSONDecodeError) as error:
                    fail(f"{conn.topic}: 无效 CAN 传输诊断：{error}")
                if (not isinstance(data, dict) or data.get("schema_version") != "piper_direct_can_diagnostics/1"
                        or type(data.get("stamp_ns")) is not int
                        or data.get("channel") != "can_" + conn.topic.split("/")[2]
                        or not isinstance(data.get("streams"), dict)
                        or type(data.get("capture_failed")) is not bool):
                    fail(f"{conn.topic}: CAN 传输诊断格式或实体归属无效")
                if not start <= data["stamp_ns"] <= stop:
                    continue
                for kind in ("master_target", "slave_feedback"):
                    counters = data["streams"].get(kind)
                    if not isinstance(counters, dict) or any(
                            type(counters.get(key)) is not int or counters[key] < 0
                            for key in ("overflow", "publish_errors", "shutdown_discarded")):
                        fail(f"{conn.topic}: CAN 传输诊断缺少完整损失计数")
                    if any(counters[key] for key in ("overflow", "publish_errors", "shutdown_discarded")):
                        fail(f"{conn.topic}: CAN 队列溢出、发布失败或退出丢帧，拒绝不完整来源")
                if data["capture_failed"]:
                    fail(f"{conn.topic}: CAN 接收/发布通路已标记采集失败：{data.get('error')}")
                previous = transport_diagnostics.get(conn.topic)
                if previous and previous["last"]["stamp_ns"] > data["stamp_ns"]:
                    fail(f"{conn.topic}: CAN 传输诊断时间倒退")
                peaks = previous["peaks"] if previous else {}
                for kind in ("master_target", "slave_feedback"):
                    counters = data["streams"][kind]
                    kind_peaks = peaks.setdefault(kind, {})
                    for key in ("interval_max_kernel_to_app_ms", "interval_max_queue_wait_ms",
                                "interval_max_publish_call_ms", "max_kernel_to_app_ms",
                                "max_queue_wait_ms", "max_publish_call_ms"):
                        value = counters.get(key)
                        if isinstance(value, (int, float)) and value > kind_peaks.get(key, 0):
                            kind_peaks[key] = value
                transport_diagnostics[conn.topic] = {
                    "observations": (previous["observations"] if previous else 0) + 1,
                    "last": data, "peaks": peaks}
                continue
            timestamp, topic = stamp(message), conn.topic
            error = None
            if topic in camera_topics:
                value, can_stamps = image(message, topic, materialize, spool_dir), None
            elif topic in extra_frame_topics:
                value, can_stamps = None, None
            elif input_format == "can-vector":
                # Flags can change as an unchanged Header is republished during a
                # pause. Compare the source payload separately from those flags.
                source_payload = (list(message.joint_names), list(message.position),
                                  [(item.sec, item.nanosec) for item in message.source_stamps])
                previous = raw_vectors[topic].setdefault(timestamp, source_payload)
                if source_payload != previous:
                    fail(f"{path}: {topic} 同一 Header 时间对应冲突向量")
                raw_quality[topic][timestamp] = {
                    "missing": list(map(bool, message.missing)),
                    "stale": list(map(bool, message.stale)),
                    "time_inconsistent": bool(message.time_inconsistent),
                    "raw_valid": bool(message.valid),
                    "gripper_mapping_verified": bool(message.gripper_mapping_verified)}
                try:
                    value, can_stamps = can_vector(message, topic, max_can_skew_ns, gripper_calibration,
                                                   raw_signed_gripper=_raw_signed_gripper)
                    # The next-state action for the last state may be one grid
                    # period after the Episode's logical stop.
                    source_stop = stop
                    if action_source == "follower_next_state" and topic in action_topics.values():
                        source_stop += round(1_000_000_000 / fps)
                    if min(can_stamps) < start or max(can_stamps) > source_stop:
                        raise InvalidFrame(f"{topic}: CAN 来源时间不在 Episode 录制区间内")
                except InvalidFrame as exception:
                    if not extract_valid_segments:
                        raise
                    error = str(exception)
                    value, can_stamps = None, None
                    rejected_vectors[(topic, timestamp, error)] += 1
                if error is None:
                    valid_by_header[topic].setdefault(timestamp, (value, can_stamps))
                    vector_quality[topic][timestamp] = {
                        "raw_valid": bool(message.valid),
                        "raw_gripper_mapping_verified": bool(message.gripper_mapping_verified),
                        "raw_missing": list(map(bool, message.missing)),
                        "raw_stale": list(map(bool, message.stale)),
                        "raw_time_inconsistent": bool(message.time_inconsistent),
                        "gripper_calibration_applied": gripper_calibration is not None}
            else:
                value, can_stamps = vector(message, topic), None
            streams[topic].append((timestamp, value, can_stamps, error))
    for topic in found & diagnostic_topics:
        if topic not in transport_diagnostics:
            fail(f"{topic}: 缺少录制区间内的 CAN 传输诊断")
    for topic, values in streams.items():
        if not values:
            fail(f"{path}: {topic} 没有消息")
        if any(a[0] > b[0] for a, b in zip(values, values[1:])):
            fail(f"{path}: {topic} 的 Header 时间戳倒退")
        if input_format == "can-vector" and topic in vector_topics:
            deduplicated = [values[0]]
            for item in values[1:]:
                if item[0] == deduplicated[-1][0]:
                    # A repeated Header may be published again after the source
                    # stops.  Keep the observation that was valid at its own
                    # source time; current expiration is checked separately by
                    # the age limit at each sampled frame.
                    if deduplicated[-1][3] is not None and item[3] is None:
                        deduplicated[-1] = item
                    elif deduplicated[-1][3] is None and item[3] is not None:
                        continue
                    else:
                        deduplicated[-1] = item
                else:
                    deduplicated.append(item)
            streams[topic] = values = deduplicated
        if any(a[0] >= b[0] for a, b in zip(values, values[1:])):
            fail(f"{path}: {topic} 的 Header 时间戳重复或倒退")
    valid_actions = {}
    valid_action_indexes = {}
    if input_format == "can-vector":
        for topic in (action_topics[arm] for arm in arms):
            valid_actions[topic] = [(timestamp, value, can_stamps)
                                    for timestamp, (value, can_stamps)
                                    in sorted(valid_by_header[topic].items())]
            valid_action_indexes[topic] = [item[0] for item in valid_actions[topic]]
    camera_gaps = {topic: max((right[0] - left[0] for left, right in zip(streams[topic], streams[topic][1:])),
                              default=0) for topic in camera_topics}
    for topic, gap in camera_gaps.items():
        if gap > max_camera_gap_ns and not extract_valid_segments:
            fail(f"{path}: {topic} 的 RGB Header 间隔 {gap} ns 超过连续性阈值")
    period = 1_000_000_000 / fps
    if input_format == "can-vector":
        first = max(streams[action_topics[arm]][0][0] for arm in arms)
        last = min(streams[action_topics[arm]][-1][0] for arm in arms)
        if extract_valid_segments:
            first, last = max(start, first), min(last, stop)
        action_times = [first + round(index * period) for index in range(max(0, int((last - first) / period) + 1))]
    else:
        action_times = [item[0] for item in streams[action_topics[arms[0]]]]
    if len(action_times) < 2 and not extract_valid_segments:
        fail(f"{path}: 至少需要两帧示教动作")
    if action_times and (action_times[0] < start or action_times[-1] > stop):
        fail(f"{path}: 动作 Header 时间不在 Episode 录制区间内")
    if input_format == "joint-state":
        for index, timestamp in enumerate(action_times):
            if abs((timestamp - action_times[0]) - index * period) > jitter_ns:
                fail(f"{path}: 示教动作时间戳不符合 {fps} FPS，帧 {index}")
        for arm in arms[1:]:
            other = streams[action_topics[arm]]
            if len(other) != len(action_times) or any(abs(item[0] - ref) > jitter_ns for item, ref in zip(other, action_times)):
                fail(f"{path}: 左右示教动作未同步")
    indexes = {topic: [item[0] for item in values] for topic, values in streams.items()}

    def hold_at(timestamp):
        for segment in hold_segments:
            held_end = min(segment["end_ns"], segment["start_ns"] + hold_max_ns)
            if segment["start_ns"] <= timestamp <= held_end:
                return segment
        return None

    def hold_action(topic, timestamp, segment):
        """Return the last explicit operator-held target, never a fabricated value."""
        anchor_index = bisect_right(valid_action_indexes[topic], segment["start_ns"]) - 1
        if anchor_index < 0:
            raise InvalidFrame(f"{topic}: 保持段前没有有效主目标")
        anchor_time, anchor_value, anchor_can = valid_actions[topic][anchor_index]
        if segment["start_ns"] - anchor_time > hold_anchor_gap_ns:
            raise InvalidFrame(f"{topic}: 保持段开始前最后有效主目标过旧")
        latest_index = bisect_right(indexes[topic], timestamp) - 1
        if latest_index < 0:
            raise InvalidFrame(f"{topic}: 保持段内没有原始向量")
        latest = streams[topic][latest_index]
        anchor_raw = raw_vectors[topic].get(anchor_time)
        latest_raw = raw_vectors[topic].get(latest[0])
        if (anchor_raw is None or latest_raw is None
                or not np.array_equal(np.asarray(anchor_raw[1]), np.asarray(latest_raw[1]))):
            raise InvalidFrame(f"{topic}: 保持段内过时向量与最后有效目标不一致")
        quality = raw_quality[topic].get(latest[0], {})
        return anchor_time, anchor_value, anchor_can, {
            "hold_id": segment["hold_id"], "source": segment["source"],
            "anchor_header_ns": anchor_time, "stale_header_ns": latest[0],
            "stale_flags": list(quality.get("stale", [])),
            "hold_start_ns": segment["start_ns"], "hold_end_ns": segment["end_ns"]}

    def preceding(topic, timestamp, allow_hold=False, hold_timestamp=None, allow_after_stop_ns=0):
        index = bisect_right(indexes[topic], timestamp) - 1
        segment = hold_at(timestamp if hold_timestamp is None else hold_timestamp) if allow_hold else None
        age_limit = min(max_age_ns, max_camera_gap_ns) if extract_valid_segments and topic in camera_topics else max_age_ns
        if index < 0 or timestamp - indexes[topic][index] > age_limit:
            if segment is not None:
                return hold_action(topic, timestamp if hold_timestamp is None else hold_timestamp, segment)
            raise InvalidFrame(f"{path}: {topic} 在动作时间 {timestamp} 前缺帧或超窗")
        item = streams[topic][index]
        if segment is not None and item[0] < segment["start_ns"]:
            # The explicit hold starts after this observation; reuse the last
            # valid absolute target instead of rejecting the frame on skew.
            return hold_action(topic, timestamp if hold_timestamp is None else hold_timestamp, segment)
        if item[3] is not None:
            if segment is not None:
                return hold_action(topic, timestamp if hold_timestamp is None else hold_timestamp, segment)
            raise InvalidFrame(item[3])
        if not start <= item[0] <= stop + allow_after_stop_ns:
            raise InvalidFrame(f"{path}: {topic} 的 Header 时间不在 Episode 录制区间内")
        return item[:3] + (None,)

    # Intersect continuous source coverage before sampling. An invalid source
    # between two grid points must still break the sequence.
    domains = [(start, stop)]
    if extract_valid_segments:
        for topic in (*camera_topics, *vector_topics):
            limit = min(max_age_ns, max_camera_gap_ns) if topic in camera_topics else max_age_ns
            intervals, current, previous_stamp = [], None, None
            for timestamp, _, _, error in streams[topic]:
                if error is not None:
                    if current:
                        intervals.append((current[0], min(current[1], timestamp - 1)))
                    current, previous_stamp = None, None
                    continue
                if current is not None and timestamp - previous_stamp <= limit:
                    current = (current[0], timestamp + limit)
                else:
                    if current:
                        intervals.append(current)
                    current = (timestamp, timestamp + limit)
                previous_stamp = timestamp
            if current:
                intervals.append(current)
            if input_format == "can-vector" and topic in action_topics.values():
                for segment in hold_segments:
                    anchor_index = bisect_right(valid_action_indexes[topic], segment["start_ns"]) - 1
                    if anchor_index < 0:
                        continue
                    anchor_time = valid_actions[topic][anchor_index][0]
                    if segment["start_ns"] - anchor_time > hold_anchor_gap_ns:
                        continue
                    held_end = min(segment["end_ns"], segment["start_ns"] + hold_max_ns)
                    intervals.append((max(anchor_time, start), min(held_end, stop)))
            intervals = merge_intervals(intervals)
            domains = [(max(a, c), min(b, d)) for a, b in domains for c, d in intervals
                       if max(a, c) <= min(b, d)]
    domain_starts = [a for a, _ in domains]

    def domain_at(timestamp):
        index = bisect_right(domain_starts, timestamp) - 1
        return index if index >= 0 and timestamp <= domains[index][1] else None

    def build_aligned_frame(timestamp, index=None, reference=None, camera_selection=None):
        reference = timestamp if reference is None else reference
        follower_action = action_source == "follower_next_state"
        action_reference = reference + period if follower_action else reference
        source_stamps, can_sources = {}, {}
        action_source_stamps, action_can_sources = {}, {}
        actions, states = [], []
        hold_actions = {}
        if follower_action:
            for arm in arms:
                topic = state_topics[arm]
                actual, value, can_stamps, _ = preceding(topic, reference)
                if actual > reference:
                    raise InvalidFrame(f"{topic}: 来源 Header 晚于图像组参考时刻")
                source_stamps[topic] = actual
                if can_stamps is not None:
                    can_sources[topic] = can_stamps
                states.append(value)
            for arm in arms:
                topic = action_topics[arm]
                # The last state's action is the next observed state, which may
                # be up to one grid period after the logical Episode stop.
                actual, value, can_stamps, _ = preceding(
                    topic, action_reference, allow_after_stop_ns=period)
                if actual > action_reference:
                    raise InvalidFrame(f"{topic}: 下一状态 Header 晚于动作参考时刻")
                if actual <= source_stamps[topic]:
                    raise InvalidFrame(f"{topic}: 下一状态反馈没有前进，不能作为下一时刻动作")
                action_source_stamps[topic] = actual
                if can_stamps is not None:
                    action_can_sources[topic] = can_stamps
                actions.append(value)
        else:
            for topics, target in ((action_topics, actions), (state_topics, states)):
                for arm in arms:
                    topic = topics[arm]
                    if topics is action_topics and input_format == "joint-state":
                        actual, value, can_stamps = streams[topic][index][:3]
                        hold = None
                    else:
                        actual, value, can_stamps, hold = preceding(
                            topic, reference, allow_hold=topics is action_topics,
                            hold_timestamp=timestamp)
                    if input_format == "can-vector" and actual > reference:
                        raise InvalidFrame(f"{topic}: 来源 Header 晚于图像组参考时刻")
                    source_stamps[topic] = actual
                    if can_stamps is not None:
                        can_sources[topic] = can_stamps
                    target.append(value)
                    if hold is not None:
                        hold_actions[topic] = hold
        if len({item["hold_id"] for item in hold_actions.values()}) > 1:
            raise InvalidFrame(f"{path}: 同一帧包含多个保持段")
        hold_info = None
        if hold_actions:
            hold_info = dict(next(iter(hold_actions.values())))
            hold_info["arms"] = sorted(topic.split("/")[2] for topic in hold_actions)
        frame = {"task": task, "observation.state": np.concatenate(states),
                 "action": np.concatenate(actions), "__source_header_ns__": source_stamps,
                 "__action_source_header_ns__": action_source_stamps,
                 "__source_can_stamps_ns__": can_sources,
                 "__action_source_can_stamps_ns__": action_can_sources,
                 "__target_timestamp_ns__": timestamp,
                 "__action_target_timestamp_ns__": action_reference,
                 "__hold__": hold_info,
                 "__source_vector_quality__": {topic: vector_quality[topic][source_stamps[topic]]
                     for topic in vector_topics if topic in source_stamps} if input_format == "can-vector" else {},
                 "__action_source_vector_quality__": {topic: vector_quality[topic][action_source_stamps[topic]]
                     for topic in action_source_stamps} if follower_action and input_format == "can-vector" else {}}
        for camera in ("top_rgb", *(f"{arm}_wrist_rgb" for arm in arms)):
            topic = CAMERAS[camera]
            if camera_selection is None:
                observed_at, value, _, _ = preceding(topic, timestamp)
            else:
                observed_at, value, _, _ = streams[topic][camera_selection[topic]]
            source_stamps[CAMERAS[camera]] = observed_at
            frame[f"observation.images.{camera}"] = value
        camera_times = [source_stamps[topic] for topic in camera_topics]
        skew = max(camera_times) - min(camera_times)
        if skew > max_camera_skew_ns:
            raise InvalidFrame(f"{path}: RGB 之间的 Header 时间差 {skew} ns 超过同步阈值")
        fresh_sources = [stamp for topic, stamp in source_stamps.items()
                         if topic not in hold_actions]
        if input_format == "can-vector" and any(
                not 0 <= timestamp - source <= max_age_ns for source in fresh_sources):
            raise InvalidFrame(f"{path}: 图像组、状态或目标相对采样网格缺帧或超窗")
        source_skew = max(fresh_sources) - min(fresh_sources)
        if max_source_skew_ns is not None and source_skew > max_source_skew_ns:
            raise InvalidFrame(f"{path}: RGB、状态与示教目标 Header 总体时间差 {source_skew} ns 超过同步阈值")
        if follower_action:
            action_headers = list(action_source_stamps.values())
            if any(not 0 <= action_reference - source <= max_age_ns for source in action_headers):
                raise InvalidFrame(f"{path}: 下一状态反馈相对动作参考时刻缺帧或超窗")
            if max(action_headers) - min(action_headers) > max_can_skew_ns:
                raise InvalidFrame(f"{path}: 左右臂下一状态反馈 Header 时间差超过同步阈值")
        if input_format == "can-vector":
            frame["__alignment__"] = {
                "policy": FOLLOWER_ALIGNMENT_POLICY if follower_action else ALIGNMENT_POLICY,
                "reference_timestamp_ns": reference,
                "action_reference_timestamp_ns": action_reference,
                "action_source": action_source,
                "action_offset_ns": period if follower_action else 0,
                "grid_lag_ns": timestamp - reference}
        return frame, skew, source_skew

    def aligned_frame(timestamp, index=None, required_camera=None):
        if input_format != "can-vector":
            return build_aligned_frame(timestamp, index)
        # Resample coherent, causal RGB bundles onto the unchanged fixed grid.
        # Action/state labels are sampled at the bundle's real Header reference;
        # every non-held source must also remain <= max_age from the grid.
        limit = min(max_age_ns, max_camera_gap_ns) if extract_valid_segments else max_age_ns
        references = set()
        for topic in camera_topics:
            times = indexes[topic]
            begin, end = bisect_right(times, timestamp - limit - 1), bisect_right(times, timestamp)
            references.update(times[begin:end])
        failure = InvalidFrame(f"{path}: RGB 图像组在采样网格前缺帧或超窗")
        for reference in sorted(references, reverse=True):
            selection = {topic: bisect_right(indexes[topic], reference) - 1 for topic in camera_topics}
            if any(index < 0 for index in selection.values()):
                continue
            selected = {topic: indexes[topic][index] for topic, index in selection.items()}
            if required_camera and required_camera[0] in selected and (
                    selected[required_camera[0]] != required_camera[1]):
                continue
            if any(timestamp - stamp > limit for stamp in selected.values()):
                continue
            if max(selected.values()) - min(selected.values()) > max_camera_skew_ns:
                failure = InvalidFrame(f"{path}: RGB 之间的 Header 时间差超过同步阈值")
                continue
            try:
                frame = build_aligned_frame(timestamp, index, reference, selection)
                if extract_valid_segments and (domain_at(timestamp) is None
                                               or domain_at(reference) != domain_at(timestamp)):
                    raise InvalidFrame("图像组参考时刻与采样网格不在同一连续有效来源区间内")
                return frame
            except InvalidFrame as error:
                failure = error
        raise failure

    frames, segments, rejected_frames = [], [], []
    max_selected_camera_skew = max_selected_source_skew = 0
    last_grid_index = last_domain = None
    for index, timestamp in enumerate(action_times):
        try:
            frame, skew, source_skew = aligned_frame(timestamp, index)
            if input_format == "can-vector" and frames and (
                    frame["__alignment__"]["reference_timestamp_ns"]
                    <= frames[-1]["__alignment__"]["reference_timestamp_ns"]):
                raise InvalidFrame("RGB 图像组重复或参考时刻倒退，不能重复计入连续示教")
            domain = domain_at(timestamp)
            if extract_valid_segments and domain is None:
                raise InvalidFrame("原始 Header 不在连续有效来源区间内")
        except InvalidFrame as error:
            if not extract_valid_segments:
                raise
            rejected_frames.append({"grid_index": index, "target_timestamp_ns": timestamp, "reason": str(error)})
            continue
        if last_grid_index != index - 1 or domain != last_domain:
            valid_start, valid_stop = domains[domain]
            if index:
                valid_start = max(valid_start, action_times[index - 1] + 1)
            segments.append({"segment_index": len(segments), "frames": 0,
                             "first_target_timestamp_ns": timestamp, "last_target_timestamp_ns": timestamp,
                             "valid_start_ns": valid_start, "valid_stop_ns": valid_stop})
        segment = segments[-1]
        segment["frames"] += 1
        segment["last_target_timestamp_ns"] = timestamp
        # A sampled alignment failure is also a boundary, even if every
        # individual source has continuous coverage.
        if index + 1 < len(action_times):
            segment["valid_stop_ns"] = min(domains[domain][1], action_times[index + 1] - 1)
        else:
            segment["valid_stop_ns"] = domains[domain][1]
        frame["__segment_index__"] = segment["segment_index"]
        frames.append(frame)
        last_grid_index, last_domain = index, domain
        max_selected_camera_skew = max(max_selected_camera_skew, skew)
        max_selected_source_skew = max(max_selected_source_skew, source_skew)
    for segment in segments:
        segment["window_count"] = max(0, segment["frames"] - ACTION_HORIZON + 1)
        segment["meets_16_step_window"] = segment["frames"] >= ACTION_HORIZON
    hold_frame_counts = Counter(frame["__hold__"]["hold_id"] for frame in frames
                                if frame.get("__hold__"))
    meta["_quality"] = {"camera_max_gap_ns": camera_gaps,
                        "transport_diagnostics": transport_diagnostics,
                        "alignment_policy": ((FOLLOWER_ALIGNMENT_POLICY if action_source == "follower_next_state"
                                              else ALIGNMENT_POLICY) if input_format == "can-vector"
                                             else "joint_state_grid"),
                        "action_source": action_source,
                        "max_grid_lag_ns": max((frame.get("__alignment__", {}).get("grid_lag_ns", 0)
                                                for frame in frames), default=0),
                        "max_selected_camera_skew_ns": max_selected_camera_skew,
                        "max_selected_source_skew_ns": max_selected_source_skew,
                        "aligned_frame_count": len(frames), "sampled_frame_count": len(action_times),
                        "segment_extraction": extract_valid_segments, "segments": segments,
                        "hold_segments": [{"hold_id": item["hold_id"],
                                           "start_unix_ns": item["start_ns"],
                                           "end_unix_ns": item["end_ns"],
                                           "source": item["source"],
                                           "max_hold_ns": hold_max_ns,
                                           "truncated": item["end_ns"] - item["start_ns"] > hold_max_ns,
                                           "aligned_frames": hold_frame_counts.get(item["hold_id"], 0)}
                                          for item in hold_segments],
                        "rejected_frames": rejected_frames,
                        "rejected_vector_messages": [{"topic": topic, "header_timestamp_ns": timestamp,
                            "reason": reason, "count": count}
                            for (topic, timestamp, reason), count in sorted(rejected_vectors.items())],
                        "training_contract_reasons": contract_reasons}
    meta["_camera_header_stamps"] = {topic: indexes[topic] for topic in (*camera_topics, *extra_frame_topics)}
    if extract_valid_segments:
        completion_segments = {}
        completion_errors = {}
        completion_proofs = {}
        for topic, timestamps in meta["_camera_header_stamps"].items():
            completion_segments[topic] = {}
            for timestamp in timestamps:
                if completion_header_range is not None and not (
                        completion_header_range[0] <= timestamp <= completion_header_range[1]):
                    continue
                segment = next((item for item in segments if item["valid_start_ns"] <= timestamp <= item["valid_stop_ns"]), None)
                if segment is None:
                    completion_errors[(topic, timestamp)] = "完成帧不在已对齐的连续片段内"
                    continue
                try:
                    completion_frame, camera_skew, source_skew = aligned_frame(
                        timestamp, required_camera=(topic, timestamp))
                except InvalidFrame as error:
                    completion_errors[(topic, timestamp)] = str(error)
                    continue
                completion_segments[topic][timestamp] = segment["segment_index"]
                completion_proofs[(topic, timestamp)] = {
                    "reference_timestamp_ns": completion_frame['__alignment__']['reference_timestamp_ns'],
                    "source_header_stamps_ns": completion_frame['__source_header_ns__'],
                    "action_source_header_stamps_ns": completion_frame['__action_source_header_ns__'],
                    "camera_skew_ns": camera_skew, "source_skew_ns": source_skew}
        meta["_completion_segment_by_header"] = completion_segments
        meta["_completion_errors"] = completion_errors
        meta["_completion_proofs"] = completion_proofs
        meta["_continuous_source_domains"] = domains
    return meta, frames


def features(arms):
    names = [f"{arm}.{joint}" for arm in arms for joint in JOINTS]
    result = {key: {"dtype": "float32", "shape": (len(names),), "names": names}
              for key in ("observation.state", "action")}
    for camera in ("top_rgb", *(f"{arm}_wrist_rgb" for arm in arms)):
        result[f"observation.images.{camera}"] = {"dtype": "image", "shape": (480, 640, 3),
                                                    "names": ["height", "width", "channel"]}
    return result


def source_digest(path):
    digest = hashlib.sha256()
    databases = sorted((path / "bag").glob("*.db3"))
    if not databases:
        fail(f"{path}: 缺少 rosbag2 SQLite 文件")
    for file in [path / "episode.json", path / "bag" / "metadata.yaml", *databases]:
        digest.update(file.name.encode())
        with file.open("rb") as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(block)
    return digest.hexdigest()


def sft_label(meta):
    """Return (label, reasons); v1 remains readable but has no success-SFT proof."""
    reasons = []
    if meta.get("schema_version") != 2:
        return None, ["缺少 episode.json v2 技能标注"]
    if meta.get("mode") != "skill":
        return None, ["task 模式尚未按技能段导出"]
    skill = meta.get("skill")
    if skill not in SKILLS:
        reasons.append("skill 不在固定原子技能集中")
    fields = meta.get("task_metadata")
    arm_mode = fields.get("arm_mode") if isinstance(fields, dict) else None
    initial = fields.get("initial_conditions") if isinstance(fields, dict) else None
    group_id = initial.get("perturbation_id") if isinstance(initial, dict) else None
    if not isinstance(group_id, str) or not group_id.strip():
        reasons.append("缺少 task_metadata.initial_conditions.perturbation_id 场景组标识")
    if (skill in SKILLS[1:] and arm_mode != "BIMANUAL"
            or skill == SKILLS[0] and arm_mode not in ("LEFT", "RIGHT")):
        reasons.append("skill 与 arm_mode 不匹配")
    task_text = meta.get("task")
    if not isinstance(task_text, str) or not task_text.strip():
        reasons.append("缺少该 Episode 的 task 文本")
    annotations = meta.get("annotations")
    if not isinstance(annotations, list):
        return None, reasons + ["annotations 不是数组"]
    outcomes = [item for item in annotations if isinstance(item, dict) and item.get("kind") == "outcome"]
    completes = [item for item in annotations if isinstance(item, dict) and item.get("kind") == "skill_complete"]
    if len(outcomes) != 1 or outcomes[0].get("value") not in ("success", "failure"):
        reasons.append("缺少唯一有效的 outcome 标注")
    elif outcomes[0]["value"] != meta.get("outcome"):
        reasons.append("outcome 标注与 Episode 结果不一致")
    elif outcomes[0]["value"] != "success":
        reasons.append("Episode 结果不是 success")
    if len(completes) != 1 or completes[0].get("value") != skill:
        reasons.append("缺少唯一且匹配 skill 的完成标注")
        complete = None
    else:
        complete = completes[0]
        topic, timestamp = complete.get("source_topic"), complete.get("header_stamp_ns")
        declared_frames = meta.get("frame_topics") or []
        camera_source = (isinstance(declared_frames, list) and topic in declared_frames
                         and isinstance(topic, str) and topic.endswith("/image_raw"))
        if (complete.get("frame_resolution_required") is True or not camera_source
                or type(timestamp) is not int or timestamp <= 0):
            reasons.append("完成帧缺少可核验的相机 Header 引用")
        elif (type(meta.get("started_at_unix_ns")) is not int
              or type(meta.get("stopped_at_unix_ns")) is not int
              or not meta["started_at_unix_ns"] <= timestamp <= meta["stopped_at_unix_ns"]):
            reasons.append("完成帧 Header 不在 Episode 区间内")
    for item in (*outcomes, *completes):
        if ((item.get("kind") == "outcome" and item.get("annotation_origin") == "save_selection")
                or (item.get("kind") == "skill_complete"
                    and item.get("annotation_origin") in (
                        "post_recording_review", "operator_confirmed_candidate",
                        "duration_limit_last_frame",
                        "duration_limit_last_valid_master_frame", "duration_limit_before_terminal_stale"))):
            reviewed = item.get("reviewed_at_unix_ns")
            if (type(reviewed) is not int or type(meta.get("stopped_at_unix_ns")) is not int
                    or type(meta.get("saved_at_unix_ns")) is not int
                    or not meta["stopped_at_unix_ns"] <= reviewed <= meta["saved_at_unix_ns"]):
                reasons.append("保存时结果标注缺少可核验的复核时间")
            continue
        recorded = item.get("recorded_at_unix_ns")
        if (type(recorded) is not int or type(meta.get("started_at_unix_ns")) is not int
                or type(meta.get("stopped_at_unix_ns")) is not int
                or not meta["started_at_unix_ns"] <= recorded <= meta["stopped_at_unix_ns"]):
            reasons.append("标注记录时间不在 Episode 区间内")
            break
    if reasons:
        return None, reasons
    return {"skill_id": skill, "task_text": task_text.strip(),
            "group_id": group_id.strip(), "completion_frame": complete}, []


def group_split(group_ids, test_percent):
    """Deterministically assign complete perturbation groups to train/test."""
    groups = set(group_ids)
    if len(groups) < 2:
        fail("至少需要两个不同的 perturbation_id 场景组才能划分 train/test")
    test_count = min(len(groups) - 1, max(1, round(len(groups) * test_percent / 100)))
    ranked = sorted(groups, key=lambda group: (hashlib.sha256(group.encode()).digest(), group))
    return {group: "test" if group in ranked[:test_count] else "train" for group in groups}, test_count


def select_sft_frames(meta, frames, label, arms, input_format):
    complete = label["completion_frame"]
    topic, timestamp = complete["source_topic"], complete["header_stamp_ns"]
    if timestamp not in meta["_camera_header_stamps"].get(topic, ()):
        fail("完成帧 Header 未在该 Episode 的相机消息中找到")
    follower_action = bool(frames and frames[0].get("__action_source_header_ns__"))
    action_topics = (CAN_STATES if follower_action else
                     CAN_ACTIONS if input_format == "can-vector" else ACTIONS)
    def action_stamp(frame, arm):
        headers = frame.get("__action_source_header_ns__") or frame["__source_header_ns__"]
        return headers[action_topics[arm]]
    segment_index = None
    segmented = meta["_quality"].get("segment_extraction", False)
    if segmented:
        segment_index = meta["_completion_segment_by_header"].get(topic, {}).get(timestamp)
        if segment_index is None:
            fail("完成帧不属于任何连续有效片段；其他片段不能继承成功标注")
    selected = [frame for frame in frames if max(action_stamp(frame, arm)
                  for arm in arms) <= timestamp and (not segmented or (
                  frame["__segment_index__"] == segment_index and frame["__target_timestamp_ns__"] <= timestamp))]
    minimum = ACTION_HORIZON if segmented else 2
    if len(selected) < minimum:
        fail(f"完成帧所属片段在完成前不足 {minimum} 帧连续有效示教")
    return selected, {"aligned_frames": len(frames), "annotated_frames": len(selected),
                      "annotation_coverage": len(selected) / len(frames),
                      "segment_index": segment_index,
                      "window_count": max(0, len(selected) - ACTION_HORIZON + 1),
                      "completion_frame": {"source_topic": topic, "header_stamp_ns": timestamp}}


def convert(args):
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    from importlib.metadata import version
    if version("lerobot") != "0.4.4" or version("rosbags") != "0.11.7":
        fail("依赖版本必须为 lerobot==0.4.4、rosbags==0.11.7")
    arms = ["left", "right"] if args.arm_mode == "BIMANUAL" else [args.arm_mode.lower()]
    calibration_path = getattr(args, "gripper_calibration", None)
    if calibration_path and args.input_format != "can-vector":
        fail("--gripper-calibration 仅适用于 can-vector")
    gripper_calibration = load_gripper_calibration(calibration_path) if calibration_path else None
    acceptance_path = getattr(args, "source_acceptance", None)
    source_acceptance = load_source_acceptance(acceptance_path) if acceptance_path else None
    action_source = getattr(args, "action_source", "master_target")
    paths = sorted(path for path in args.input_dir.glob("episode_*") if path.is_dir())
    if not paths:
        fail("输入目录没有 episode_* 目录")
    if args.output_dir.exists():
        fail("输出目录已存在，拒绝覆盖")
    prepared = []
    excluded = []
    success_sft = getattr(args, "success_sft", False)
    extract_segments = getattr(args, "extract_valid_segments", False)
    if extract_segments and args.input_format != "can-vector":
        fail("--extract-valid-segments 仅适用于 can-vector")
    source_skew_ms = getattr(args, "max_source_skew_ms", None)
    if (success_sft or extract_segments) and source_skew_ms is None:
        fail("成功 SFT 或连续片段提取必须显式指定 --max-source-skew-ms")
    skills = set(getattr(args, "skills", None) or ())
    ids = set()
    for path in paths:
        label = None
        if success_sft:
            try:
                raw_meta = json.loads((path / "episode.json").read_text(encoding="utf-8"))
                label, reasons = sft_label(raw_meta)
            except (OSError, ValueError) as error:
                label, reasons = None, [str(error)]
            if label is not None and skills and label["skill_id"] not in skills:
                reasons = ["不在 --skills 选择范围内"]
            if label is not None and raw_meta.get("task_metadata", {}).get("arm_mode") != args.arm_mode:
                reasons = ["与 --arm-mode 不匹配"]
            if reasons:
                excluded.append({"source": str(path.resolve()), "reasons": reasons})
                continue
        try:
            extra_frames = (label["completion_frame"]["source_topic"],) if (
                label and label["completion_frame"]["source_topic"] not in CAMERAS.values()) else ()
            meta, frames = read_episode(path, arms, args.fps, int(args.max_age_ms * 1e6),
                                        int(args.jitter_ms * 1e6), input_format=args.input_format,
                                        max_can_skew_ns=int(args.max_can_skew_ms * 1e6),
                                        max_camera_gap_ns=int(getattr(args, "max_camera_gap_ms", 100) * 1e6),
                                        max_camera_skew_ns=int(getattr(args, "max_camera_skew_ms", 40) * 1e6),
                                        max_source_skew_ns=int(source_skew_ms * 1e6) if source_skew_ms is not None else None,
                                        extra_frame_topics=extra_frames,
                                        gripper_calibration=gripper_calibration,
                                        extract_valid_segments=extract_segments,
                                        action_source=action_source,
                                        source_acceptance=source_acceptance)
            annotation_stats = None
            if success_sft:
                frames, annotation_stats = select_sft_frames(meta, frames, label, arms, args.input_format)
        except (ValueError, OSError) as error:
            if not success_sft and not extract_segments:
                raise
            excluded.append({"source": str(path.resolve()), "reasons": [str(error)]})
            continue
        episode_id = meta.get("episode_id")
        if not isinstance(episode_id, str) or not episode_id or episode_id in ids:
            fail("episode_id 缺失或重复")
        ids.add(episode_id)
        digest = source_digest(path)
        if extract_segments:
            segment_ids = sorted({frame["__segment_index__"] for frame in frames})
            candidates = [(segment, sum(frame["__segment_index__"] == segment for frame in frames))
                          for segment in segment_ids]
            candidates = [(segment, count) for segment, count in candidates if count >= ACTION_HORIZON]
            if not candidates:
                excluded.append({"source": str(path.resolve()), "reasons": ["没有满足 16 步窗口的连续有效片段"]})
            for segment, count in candidates:
                export_id = episode_id if success_sft else f"{episode_id}__segment_{segment:03d}"
                prepared.append((path, meta, count, export_id, digest, label, annotation_stats, segment, episode_id))
        else:
            prepared.append((path, meta, len(frames), episode_id, digest, label, annotation_stats, None, episode_id))
    if len({item[8] for item in prepared}) < 2:
        fail(f"至少需要两个合格 Episode 才能划分 train/test；排除 {len(excluded)} 段")
    if success_sft:
        group_splits, test_count = group_split((item[5]["group_id"] for item in prepared), args.test_percent)
        split_rule = (f"sha256(task_metadata.initial_conditions.perturbation_id) 排序前 "
                      f"{test_count}/{len(group_splits)} 个完整场景组为 test")
    else:
        source_ids = {item[8] for item in prepared}
        test_count = min(len(source_ids) - 1, max(1, round(len(source_ids) * args.test_percent / 100)))
        ranked = sorted(source_ids, key=lambda item: (hashlib.sha256(item.encode()).digest(), item))
        test_ids = set(ranked[:test_count])
        split_rule = f"sha256(源 episode_id) 排序前 {test_count}/{len(source_ids)} 个完整来源为 test"
    args.output_dir.mkdir(parents=True)
    manifest = {"format": "LeRobotDataset v3.0", "lerobot_version": "0.4.4", "fps": args.fps,
                "eligibility_scope": "source_sft_export", "training_ready": False,
                "training_normalization": {"status": "pending_train_statistics_and_checkpoint_binding",
                                           "verified": False, "required_source_split": "train"},
                "purpose": "success_sft" if success_sft else "legacy_unlabeled",
                "extract_valid_segments": extract_segments,
                "action_horizon": ACTION_HORIZON if extract_segments else None,
                "max_source_skew_ms": source_skew_ms,
                "max_camera_skew_ms": getattr(args, "max_camera_skew_ms", 40),
                "action_source": action_source,
                "action_provenance": ACTION_PROVENANCE[action_source],
                "action_offset_ns": (round(1_000_000_000 / args.fps)
                                     if action_source == "follower_next_state" else 0),
                "alignment_policy": ((FOLLOWER_ALIGNMENT_POLICY
                                      if action_source == "follower_next_state"
                                      else ALIGNMENT_POLICY) if args.input_format == "can-vector"
                                     else "joint_state_grid"),
                "arm_mode": args.arm_mode, "input_format": args.input_format,
                "joint_names": [f"{arm}.{joint}" for arm in arms for joint in JOINTS],
                "state_unit": "rad (joint1..6), m (gripper)", "action_space": "absolute_joint_position_7",
                "split_rule": split_rule,
                "gripper_calibration": ({"path": str(calibration_path.resolve()),
                    "sha256": hashlib.sha256(calibration_path.read_bytes()).hexdigest(),
                    "settings": gripper_calibration} if gripper_calibration else None),
                "episodes": [], "excluded_episodes": excluded}
    timestamps_file = (args.output_dir / "source_timestamps.jsonl").open("w", encoding="utf-8")
    for split in ("train", "test"):
        repo_id = f"{args.repo_id}_{split}"
        dataset = LeRobotDataset.create(repo_id=repo_id, fps=args.fps,
                                        features=features(arms), root=args.output_dir / split,
                                        robot_type="piper", use_videos=False)
        expected_episodes = expected_frames = 0
        for path, meta, count, episode_id, digest, label, annotation_stats, segment_index, source_id in prepared:
            assigned = group_splits[label["group_id"]] if success_sft else (
                "test" if source_id in test_ids else "train")
            if assigned != split:
                continue
            with tempfile.TemporaryDirectory(prefix="piper-rgb-") as temporary:
                written_meta, frames = read_episode(path, arms, args.fps, int(args.max_age_ms * 1e6),
                                         int(args.jitter_ms * 1e6), spool_dir=Path(temporary),
                                         input_format=args.input_format,
                                         max_can_skew_ns=int(args.max_can_skew_ms * 1e6),
                                         max_camera_gap_ns=int(getattr(args, "max_camera_gap_ms", 100) * 1e6),
                                         max_camera_skew_ns=int(getattr(args, "max_camera_skew_ms", 40) * 1e6),
                                         max_source_skew_ns=int(source_skew_ms * 1e6) if source_skew_ms is not None else None,
                                         extra_frame_topics=(label["completion_frame"]["source_topic"],) if (
                                             label and label["completion_frame"]["source_topic"] not in CAMERAS.values()) else (),
                                         gripper_calibration=gripper_calibration,
                                         extract_valid_segments=extract_segments,
                                         action_source=action_source,
                                         source_acceptance=source_acceptance)
                if success_sft:
                    frames, _ = select_sft_frames(written_meta, frames, label, arms, args.input_format)
                if extract_segments:
                    frames = [frame for frame in frames if frame["__segment_index__"] == segment_index]
                if len(frames) != count:
                    fail(f"{episode_id}: 预检与写出帧数不一致")
                for index, frame in enumerate(frames):
                    if success_sft:
                        frame["task"] = label["task_text"]
                    timestamps_file.write(json.dumps({"split": split, "episode_id": episode_id,
                                                      "source_episode_id": source_id,
                                                      "dataset_episode_index": expected_episodes,
                                                      "segment_index": frame.pop("__segment_index__"),
                                                      "frame_index": index,
                                                      "target_timestamp_ns": frame.pop("__target_timestamp_ns__"),
                                                      "header_timestamp_ns": frame.pop("__source_header_ns__"),
                                                      "source_can_stamps_ns": frame.pop("__source_can_stamps_ns__"),
                                                      "source_vector_quality": frame.pop("__source_vector_quality__"),
                                                      "action_target_timestamp_ns": frame.pop("__action_target_timestamp_ns__", None),
                                                      "action_source_header_timestamp_ns": frame.pop("__action_source_header_ns__", None),
                                                      "action_source_can_stamps_ns": frame.pop("__action_source_can_stamps_ns__", None),
                                                      "action_source_vector_quality": frame.pop("__action_source_vector_quality__", None),
                                                      "hold": frame.pop("__hold__", None),
                                                      "alignment": frame.pop("__alignment__", None)},
                                                     ensure_ascii=False) + "\n")
                    for key, value in frame.items():
                        if key.startswith("observation.images."):
                            frame[key] = np.fromfile(value, dtype=np.uint8).reshape(480, 640, 3)
                    dataset.add_frame(frame)
            dataset.save_episode()
            expected_episodes += 1
            expected_frames += count
            manifest["episodes"].append({"episode_id": episode_id, "source": str(path.resolve()),
                                         "source_episode_id": source_id, "segment_index": segment_index,
                                         "dataset_episode_index": expected_episodes - 1,
                                         "segment": next(({**item, "exported_frame_count": count,
                                             "exported_window_count": max(0, count - ACTION_HORIZON + 1)}
                                             for item in meta["_quality"]["segments"]
                                             if item["segment_index"] == segment_index), None),
                                         "split": split, "frames": count,
                                         "source_sha256": digest,
                                         "calibration_version": meta["task_metadata"]["calibration_version"],
                                         "data_convention_version": meta["task_metadata"]["data_convention_version"],
                                         "normalization_version": meta["task_metadata"].get("normalization_version"),
                                         "training_normalization": normalization_status(meta["task_metadata"]),
                                         "source_export_acceptance": meta.get("_source_export_acceptance"),
                                         "deferred_training_checks": {
                                             key: check for key, check in (meta["task_metadata"].get(
                                                 "collection_contract_checks") or {}).items()
                                             if key in DOWNSTREAM_TRAINING_CHECKS},
                                         "skill_id": label["skill_id"] if label else None,
                                         "group_id": label["group_id"] if label else None,
                                         "task_text": label["task_text"] if label else meta["task"],
                                         "annotation_quality": annotation_stats,
                                         "source_quality": meta.get("_quality"),
                                         "bag_validation": meta.get("bag_validation")})
        dataset.finalize()
        check = LeRobotDataset(repo_id, root=args.output_dir / split)
        if check.meta.total_episodes != expected_episodes or len(check) != expected_frames:
            fail(f"{split}: LeRobot 回读 Episode 或帧数不匹配")
        sample = check[0]
        if tuple(sample["action"].shape) != (7 * len(arms),) or tuple(sample["observation.images.top_rgb"].shape) != (3, 480, 640):
            fail(f"{split}: LeRobot 回读特征维度不匹配")
    timestamps_file.close()
    (args.output_dir / "conversion_manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({"output": str(args.output_dir), "episodes": len(prepared),
                      "frames": sum(item[2] for item in prepared)}, ensure_ascii=False))


def main():
    parser = argparse.ArgumentParser(description="Episode rosbag2 → LeRobot v3 离线转换")
    parser.add_argument("input_dir", type=Path)
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("--repo-id", required=True)
    parser.add_argument("--arm-mode", choices=("LEFT", "RIGHT", "BIMANUAL"), required=True)
    parser.add_argument("--input-format", choices=("joint-state", "can-vector"), required=True)
    parser.add_argument("--action-source", choices=tuple(ACTION_PROVENANCE), default="master_target",
                        help="master_target：主臂绝对目标；follower_next_state：从臂反馈 t+1 作为动作")
    parser.add_argument("--gripper-calibration", type=Path,
                        help="can-vector 的四路夹爪闭合零点与 0～100 mm 导出范围 JSON")
    parser.add_argument("--source-acceptance", type=Path,
                        help="绑定原 Episode 摘要、版本和证据 SHA-256 的独立来源验收 JSON；不授予训练资格")
    parser.add_argument("--extract-valid-segments", action="store_true",
                        help="按原始 Header 提取连续有效片段，各片段独立保存且至少 16 帧；成功标注只归完成帧所在片段")
    parser.add_argument("--fps", type=int, required=True)
    parser.add_argument("--max-age-ms", type=float, required=True)
    parser.add_argument("--jitter-ms", type=float, default=2)
    parser.add_argument("--max-can-skew-ms", type=float, default=30)
    parser.add_argument("--max-camera-gap-ms", type=float, default=100)
    parser.add_argument("--max-camera-skew-ms", type=float, default=40)
    parser.add_argument("--max-source-skew-ms", type=float,
                        help="成功 SFT 必填；每帧 RGB、状态、示教目标 Header 的最大总体跨度")
    parser.add_argument("--success-sft", action="store_true", help="仅导出 v2 已核验成功的原子技能 Episode")
    parser.add_argument("--skills", nargs="+", choices=SKILLS, help="仅导出所列原子技能")
    parser.add_argument("--test-percent", type=int, default=20)
    args = parser.parse_args()
    if (args.fps <= 0 or not math.isfinite(args.max_age_ms) or args.max_age_ms <= 0
            or not math.isfinite(args.jitter_ms) or args.jitter_ms < 0
            or not math.isfinite(args.max_can_skew_ms) or args.max_can_skew_ms < 0
            or not math.isfinite(args.max_camera_gap_ms) or args.max_camera_gap_ms <= 0
            or not math.isfinite(args.max_camera_skew_ms) or args.max_camera_skew_ms < 0
            or (args.max_source_skew_ms is not None
                and (not math.isfinite(args.max_source_skew_ms) or args.max_source_skew_ms <= 0))
            or not 0 < args.test_percent < 100):
        parser.error("fps 或时间阈值、test-percent 无效")
    if (args.success_sft or args.extract_valid_segments) and args.max_source_skew_ms is None:
        parser.error("--success-sft/--extract-valid-segments 必须显式指定 --max-source-skew-ms")
    if args.action_source != "master_target" and args.input_format != "can-vector":
        parser.error("--action-source follower_next_state 仅适用于 --input-format can-vector")
    if args.skills and not args.success_sft:
        parser.error("--skills 只能与 --success-sft 同用")
    try:
        convert(args)
    except (ValueError, OSError) as error:
        print(f"转换失败: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
