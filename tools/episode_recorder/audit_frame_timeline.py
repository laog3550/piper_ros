#!/usr/bin/env python3
"""逐帧只读检查真实 Episode 的 RGB、主目标和从反馈来源。"""

import argparse
from bisect import bisect_right
from collections import Counter
import hashlib
import json
import math
from pathlib import Path

from rosbags.rosbag2 import Reader

from to_lerobot import CAMERAS, CAN_ACTIONS, CAN_IDS, CAN_STATES, JOINTS, TYPESTORE


def header_ns(message):
    stamp = message.header.stamp
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def source_row(message, topic):
    stamp = header_ns(message)
    if topic in CAMERAS.values():
        return {"header_ns": stamp, "rgb_format_ok": (
            message.encoding == "rgb8" and message.width == 640 and message.height == 480
            and message.step == 640 * 3 and bool(message.header.frame_id)
            and len(message.data) == message.step * message.height)}
    arm, kind = topic.split("/")[2:]
    sources = [value.sec * 1_000_000_000 + value.nanosec for value in message.source_stamps]
    names = list(message.joint_names)
    by_name = dict(zip(names, message.position))
    return {"header_ns": stamp, "source_stamps_ns": sources,
            "vector_format_ok": (len(names) == len(set(names)) == len(JOINTS)
                                 and set(names) == set(JOINTS)
                                 and len(message.position) == len(names)
                                 and all(math.isfinite(value) for value in message.position)),
            "source_topic_ok": message.source_topic == f"/piper_direct/{arm}/{kind}",
            "frame_id_ok": message.header.frame_id == f"can_{arm}",
            "can_ids_ok": list(message.source_can_ids) == CAN_IDS[kind],
            "missing": list(message.missing), "stale": list(message.stale),
            "time_inconsistent": bool(message.time_inconsistent),
            "valid": bool(message.valid),
            "gripper_mapping_verified": bool(message.gripper_mapping_verified),
            "gripper_raw_m": float(by_name["gripper"]) if "gripper" in by_name else None}


def load_streams(episode, topics):
    streams = {topic: [] for topic in topics}
    with Reader(episode / "bag") as reader:
        connections = [conn for conn in reader.connections if conn.topic in streams]
        for conn, _, raw in reader.messages(connections=connections):
            message = TYPESTORE.deserialize_cdr(raw, conn.msgtype)
            streams[conn.topic].append(source_row(message, conn.topic))
    stats = {}
    for topic, rows in streams.items():
        original = [row["header_ns"] for row in rows]
        rows.sort(key=lambda row: row["header_ns"])
        stamps = [row["header_ns"] for row in rows]
        stats[topic] = {
            "messages": len(rows),
            "header_non_increasing_in_bag_order": sum(b <= a for a, b in zip(original, original[1:])),
            "max_header_gap_ms": round(max((b - a for a, b in zip(stamps, stamps[1:])), default=0) / 1e6, 3),
        }
        if topic not in CAMERAS.values():
            stats[topic].update(
                missing_messages=sum(any(row["missing"]) for row in rows),
                stale_messages=sum(any(row["stale"]) for row in rows),
                time_inconsistent_messages=sum(row["time_inconsistent"] for row in rows),
                valid_messages=sum(row["valid"] for row in rows),
                gripper_mapping_verified_messages=sum(row["gripper_mapping_verified"] for row in rows))
    return streams, stats


def audit(episode, arm_mode, fps, max_age_ms, max_source_skew_ms, max_can_skew_ms):
    metadata = json.loads((episode / "episode.json").read_text(encoding="utf-8"))
    arms = ("left", "right") if arm_mode == "BIMANUAL" else (arm_mode.lower(),)
    cameras = [CAMERAS["top_rgb"], *(CAMERAS[f"{arm}_wrist_rgb"] for arm in arms)]
    actions = [CAN_ACTIONS[arm] for arm in arms]
    states = [CAN_STATES[arm] for arm in arms]
    topics = cameras + actions + states
    streams, stream_stats = load_streams(episode, topics)
    indexes = {topic: [row["header_ns"] for row in rows] for topic, rows in streams.items()}
    start = metadata.get("ready_at_unix_ns") or metadata.get("started_at_unix_ns")
    stop = metadata.get("stop_requested_at_unix_ns") or metadata.get("stopped_at_unix_ns")
    if type(start) is not int or type(stop) is not int or stop <= start:
        raise ValueError("Episode 缺少有效的录制起止时间")
    if any(not indexes[topic] for topic in topics):
        raise ValueError("所需 RGB、主目标或从反馈话题缺帧，无法建立逐帧时基")
    period_ns = round(1_000_000_000 / fps)
    first = max(start, *(indexes[topic][0] for topic in actions))
    last = min(stop, *(indexes[topic][-1] for topic in actions))
    if last < first:
        raise ValueError("所需 RGB、主目标或从反馈话题缺帧，无法建立逐帧时基")
    frames = []
    reason_counts = Counter()
    max_age_ns = round(max_age_ms * 1e6)
    max_source_skew_ns = round(max_source_skew_ms * 1e6)
    max_can_skew_ns = round(max_can_skew_ms * 1e6)
    for frame_index, target in enumerate(range(first, last + 1, period_ns)):
        selected, reasons = {}, []
        for topic in topics:
            position = bisect_right(indexes[topic], target) - 1
            if position < 0:
                reasons.append(f"{topic}:尚无帧")
                continue
            row = streams[topic][position]
            age = target - row["header_ns"]
            selected[topic] = {"header_ns": row["header_ns"], "age_ms": round(age / 1e6, 3)}
            if age > max_age_ns or row["header_ns"] < start or row["header_ns"] > stop:
                reasons.append(f"{topic}:超龄或越界")
            if topic in cameras:
                if not row["rgb_format_ok"]:
                    reasons.append(f"{topic}:RGB格式无效")
                continue
            for key in ("source_topic_ok", "frame_id_ok", "can_ids_ok"):
                if not row[key]:
                    reasons.append(f"{topic}:{key}无效")
            if not row["vector_format_ok"]:
                reasons.append(f"{topic}:向量格式无效")
            sources = row["source_stamps_ns"]
            if (len(sources) != 4 or min(sources) <= 0
                    or max(sources) != row["header_ns"]
                    or max(sources) - min(sources) > max_can_skew_ns):
                reasons.append(f"{topic}:CAN来源时间无效")
            if any(row["missing"]):
                reasons.append(f"{topic}:missing")
            if any(row["stale"]):
                reasons.append(f"{topic}:stale")
            if row["time_inconsistent"]:
                reasons.append(f"{topic}:time_inconsistent")
            if not row["gripper_mapping_verified"]:
                reasons.append(f"{topic}:夹爪映射标志未启用")
            if not row["valid"]:
                reasons.append(f"{topic}:valid=false")
            selected[topic].update({"source_stamps_ns": row["source_stamps_ns"],
                                    "gripper_raw_m": row["gripper_raw_m"],
                                    "valid": row["valid"]})
        if len(selected) == len(topics):
            stamps = [row["header_ns"] for row in selected.values()]
            if max(stamps) - min(stamps) > max_source_skew_ns:
                reasons.append("RGB/动作/状态跨源偏差超限")
        reasons = sorted(set(reasons))
        reason_counts.update(reasons)
        frames.append({"frame_index": frame_index, "target_unix_ns": target,
                       "sensor_alignment_pass": not reasons,
                       "selected": selected, "reasons": reasons,
                       "training_eligible": False})
    source_hashes = {}
    for relative in ("episode.json", "bag/metadata.yaml"):
        source_hashes[relative] = hashlib.sha256((episode / relative).read_bytes()).hexdigest()
    summary = {"schema_version": "piper_frame_timeline_audit_v1",
               "source_episode": str(episode.resolve()), "episode_id": metadata.get("episode_id"),
               "source_metadata_sha256": source_hashes,
               "arm_mode": arm_mode, "fps": fps,
               "thresholds_ms": {"max_age": max_age_ms, "max_source_skew": max_source_skew_ms,
                                 "max_can_skew": max_can_skew_ms},
               "sampled_frames": len(frames),
               "sensor_alignment_pass_frames": sum(row["sensor_alignment_pass"] for row in frames),
               "training_eligible_frames": 0,
               "reason_counts": dict(reason_counts), "streams": stream_stats,
               "note": "逐帧来源诊断不授予训练资格；标注、标定、归一化与独立核验另行审计。"}
    return summary, frames


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("episode", type=Path)
    parser.add_argument("--arm-mode", choices=("LEFT", "RIGHT", "BIMANUAL"), required=True)
    parser.add_argument("--fps", type=int, default=10)
    parser.add_argument("--max-age-ms", type=float, default=100)
    parser.add_argument("--max-source-skew-ms", type=float, default=30)
    parser.add_argument("--max-can-skew-ms", type=float, default=30)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    if args.fps <= 0 or min(args.max_age_ms, args.max_source_skew_ms, args.max_can_skew_ms) <= 0:
        parser.error("帧率和阈值必须为正数")
    if args.output_dir.exists() and any(args.output_dir.iterdir()):
        parser.error("输出目录必须为空，避免覆盖既有逐帧报告")
    summary, frames = audit(args.episode, args.arm_mode, args.fps, args.max_age_ms,
                            args.max_source_skew_ms, args.max_can_skew_ms)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    with (args.output_dir / "frames.jsonl").open("w", encoding="utf-8") as output:
        for row in frames:
            output.write(json.dumps(row, ensure_ascii=False) + "\n")
    print(json.dumps({"summary": str(args.output_dir / "summary.json"),
                      "frames": str(args.output_dir / "frames.jsonl"),
                      "sampled_frames": summary["sampled_frames"],
                      "sensor_alignment_pass_frames": summary["sensor_alignment_pass_frames"]},
                     ensure_ascii=False))


if __name__ == "__main__":
    main()
