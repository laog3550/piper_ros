#!/usr/bin/env python3
"""只读盘点真实录制并给出训练契约拒绝原因。成功 SFT 审计须显式传入 --max-source-skew-ms；运行：python3 audit_real_data.py DATA_ROOT [--episode-root PATH] [--output PATH]。"""

import argparse
import json
import math
from pathlib import Path

import yaml

from to_lerobot import (CAN_ACTIONS, CAN_STATES, ACTIONS, STATES, group_split, read_episode,
                        select_sft_frames, sft_label)

CAMERAS = {"top_rgb": "/camera_top/D455/color/image_raw",
           "left_wrist_rgb": "/camera_wrist_left/color/image_raw",
           "right_wrist_rgb": "/camera_wrist_right/color/image_raw"}
STATES = {arm: f"/joint_{arm}" for arm in ("left", "right")}
ACTIONS = {arm: f"/demonstration/action_{arm}" for arm in STATES}
CAN_STATES = {arm: f"/piper_vector/{arm}/slave_feedback" for arm in STATES}
CAN_ACTIONS = {arm: f"/piper_vector/{arm}/master_target" for arm in STATES}


def bag_info(metadata_path):
    info = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))["rosbag2_bagfile_information"]
    return info, {row["topic_metadata"]["name"]: row["message_count"]
                  for row in info["topics_with_message_count"]}


def audit_legacy(metadata_path):
    bag = metadata_path.parent
    info, counts = bag_info(metadata_path)
    manifest_path = bag / "episode_manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8")) if manifest_path.exists() else {}
    candidates = []
    for path in sorted(bag.parent.glob(bag.name + "-candidate*/candidate_manifest.json")):
        item = json.loads(path.read_text(encoding="utf-8"))
        candidates.append({
            "path": str(path), "sample_count": item.get("sample_count", 0),
            "max_internal_gap_seconds": item.get("max_internal_gap_seconds"),
            "contiguous_segments": len(item.get("contiguous_segments", [])),
            "training_ready": item.get("training_ready"),
            "quality_errors": item.get("quality_errors", []),
            "pending": item.get("pending", []),
        })
    direct = any(name.startswith("/dataset/can_left/") for name in counts)
    reasons = ["录制格式不是已保存的 episode.json/bag 契约；缺少显式动作来源、时间域、标定与归一化版本"]
    if direct:
        reasons += ["动作/反馈实体归属及夹爪映射未验证", "只有左侧 CAN；没有独立 pick/place 技能标签"]
    else:
        reasons += ["无示教动作目标与实测关节反馈", "只有任务/场景标签，无独立技能标签"]
    if candidates and any(item["training_ready"] is False for item in candidates):
        reasons.append("现有候选清单明确标记 training_ready=false")
    return {
        "source": str(bag), "kind": "legacy_rosbag", "episode_id": manifest.get("episode_id", bag.name),
        "task": manifest.get("metadata", {}).get("task", "banana_into_box"),
        "skill": None, "recording_validated": manifest.get("valid"), "topic_counts": counts,
        "bag_start_receive_ns": info["starting_time"]["nanoseconds_since_epoch"],
        "bag_duration_ns": info["duration"]["nanoseconds"],
        "header_time_domain_declared": False,
        "has_depth": any("/depth/" in name and count > 0 for name, count in counts.items()),
        "has_rgb": any("/color/" in name and count > 0 for name, count in counts.items()),
        "has_action_target": counts.get("/dataset/can_left/action_target", 0) > 0,
        "has_feedback": counts.get("/dataset/can_left/feedback_observed", 0) > 0,
        "candidates": candidates, "eligible": False, "reasons": reasons,
    }


def audit_native(episode_path, fps=10, max_age_ms=100, max_can_skew_ms=30,
                 max_camera_gap_ms=100, max_camera_skew_ms=30, max_source_skew_ms=None):
    meta = json.loads((episode_path / "episode.json").read_text(encoding="utf-8"))
    info, counts = bag_info(episode_path / "bag" / "metadata.yaml")
    fields = meta.get("task_metadata") or {}
    reasons = []
    if meta.get("state") != "saved":
        reasons.append("Episode 尚未保存完成")
    if meta.get("clock_adjustments"):
        reasons.append("录制期间检测到系统时钟跳变")
    for topic, stats in (meta.get("bag_validation") or {}).get("frame_stats_by_header_stamp", {}).items():
        if stats.get("non_increasing_stamps", 0):
            reasons.append(f"{topic} 的 Header 时间戳不单调")
    for key in ("arm_mode", "time_domain", "action_space", "action_provenance",
                "calibration_version", "normalization_version"):
        if not fields.get(key):
            reasons.append(f"缺少 {key}")
    arms = ("left", "right") if fields.get("arm_mode") == "BIMANUAL" else (
        fields.get("arm_mode", "").lower(),)
    for arm in arms:
        if arm not in STATES:
            continue
        if not any(counts.get(t, 0) for t in (STATES[arm], CAN_STATES[arm])):
            reasons.append(f"缺少 {arm} 实测状态")
        if not any(counts.get(t, 0) for t in (ACTIONS[arm], CAN_ACTIONS[arm])):
            reasons.append(f"缺少 {arm} 示教动作")
    if not counts.get(CAMERAS["top_rgb"], 0):
        reasons.append("缺少顶部 RGB")
    if not any(counts.get(t, 0) for t in (*STATES.values(), *CAN_STATES.values())):
        reasons.append("缺少实测关节反馈")
    if not any(counts.get(t, 0) for t in (*ACTIONS.values(), *CAN_ACTIONS.values())):
        reasons.append("缺少示教动作目标")
    for arm in arms:
        if arm in STATES and not counts.get(CAMERAS[f"{arm}_wrist_rgb"], 0):
            reasons.append(f"缺少 {arm} 腕视 RGB")
    label, label_reasons = sft_label(meta)
    reasons.extend(label_reasons)
    if label is not None and max_source_skew_ms is None:
        reasons.append("未指定 RGB、状态与示教目标 Header 的总体同步阈值 --max-source-skew-ms")
    aligned_frames = annotated_frames = 0
    quality = None
    input_format = None
    if meta.get("state") == "saved" and all(arm in STATES for arm in arms):
        if all(counts.get(CAN_STATES[arm], 0) and counts.get(CAN_ACTIONS[arm], 0) for arm in arms):
            input_format = "can-vector"
        elif all(counts.get(STATES[arm], 0) and counts.get(ACTIONS[arm], 0) for arm in arms):
            input_format = "joint-state"
        else:
            reasons.append("状态与动作未使用同一种完整输入格式")
        if input_format:
            try:
                validated, frames = read_episode(
                    episode_path, list(arms), fps, int(max_age_ms * 1e6), 2_000_000,
                    input_format=input_format, max_can_skew_ns=int(max_can_skew_ms * 1e6),
                    max_camera_gap_ns=int(max_camera_gap_ms * 1e6),
                    max_camera_skew_ns=int(max_camera_skew_ms * 1e6),
                    max_source_skew_ns=(int(max_source_skew_ms * 1e6)
                                        if max_source_skew_ms is not None else None),
                    extra_frame_topics=(label["completion_frame"]["source_topic"],) if (
                        label and label["completion_frame"]["source_topic"] not in CAMERAS.values()) else ())
                aligned_frames = len(frames)
                quality = validated["_quality"]
                if label is not None:
                    selected, coverage = select_sft_frames(validated, frames, label, list(arms), input_format)
                    annotated_frames = len(selected)
                    quality = {**quality, **coverage}
            except (ValueError, OSError) as error:
                reasons.append(str(error))
    return {
        "source": str(episode_path), "kind": "native_episode", "episode_id": meta.get("episode_id"),
        "task": meta.get("task"), "skill": meta.get("skill"), "topic_counts": counts,
        "bag_start_receive_ns": info["starting_time"]["nanoseconds_since_epoch"],
        "bag_duration_ns": info["duration"]["nanoseconds"],
        "header_time_domain_declared": fields.get("time_domain") == "unix_ns",
        "frame_stats_by_header_stamp": meta.get("bag_validation", {}).get("frame_stats_by_header_stamp", {}),
        "has_depth": any("depth/" in name and count > 0 for name, count in counts.items()),
        "has_rgb": any("color/image_raw" in name and count > 0 for name, count in counts.items()),
        "has_action_target": any(counts.get(t, 0) for t in (*ACTIONS.values(), *CAN_ACTIONS.values())),
        "has_feedback": any(counts.get(t, 0) for t in (*STATES.values(), *CAN_STATES.values())),
        "annotation_coverage": annotated_frames / aligned_frames if aligned_frames else 0.0,
        "aligned_frame_count": aligned_frames, "eligible_frame_count": annotated_frames if not reasons else 0,
        "skill_id": label["skill_id"] if label else meta.get("skill"),
        "group_id": label["group_id"] if label else None,
        "task_text": label["task_text"] if label else meta.get("task"),
        "input_format": input_format, "quality": quality,
        "eligible": not reasons, "reasons": reasons,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("data_root", type=Path)
    parser.add_argument("--episode-root", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--fps", type=int, default=10)
    parser.add_argument("--max-age-ms", type=float, default=100)
    parser.add_argument("--max-can-skew-ms", type=float, default=30)
    parser.add_argument("--max-camera-gap-ms", type=float, default=100)
    parser.add_argument("--max-camera-skew-ms", type=float, default=30)
    parser.add_argument("--max-source-skew-ms", type=float,
                        help="成功 SFT 审计必填；RGB、状态、示教目标 Header 的最大总体跨度")
    parser.add_argument("--test-percent", type=int, default=20)
    args = parser.parse_args()
    if (args.fps <= 0 or not 0 < args.test_percent < 100
            or min(args.max_age_ms, args.max_can_skew_ms,
                   args.max_camera_gap_ms, args.max_camera_skew_ms) < 0):
        parser.error("帧率或时间阈值无效")
    if args.max_source_skew_ms is not None and (not math.isfinite(args.max_source_skew_ms)
                                                or args.max_source_skew_ms <= 0):
        parser.error("--max-source-skew-ms 必须为正数")
    rows = [audit_legacy(path) for path in sorted(args.data_root.rglob("metadata.yaml"))
            if not (path.parent / "episode.json").exists()
            and not (path.parent.parent / "episode.json").exists()]
    if args.episode_root:
        rows += [audit_native(path.parent, args.fps, args.max_age_ms, args.max_can_skew_ms,
                              args.max_camera_gap_ms, args.max_camera_skew_ms,
                              args.max_source_skew_ms)
                 for path in sorted(args.episode_root.rglob("episode.json"))
                 if (path.parent / "bag" / "metadata.yaml").exists()]
    eligible = [row for row in rows if row["eligible"]]
    split_ready = len({row["group_id"] for row in eligible}) >= 2
    if split_ready:
        group_splits, _ = group_split((row["group_id"] for row in eligible), args.test_percent)
        for row in eligible:
            row["suggested_split"] = group_splits[row["group_id"]]
    skills = {skill: sum(row.get("skill_id") == skill for row in eligible)
              for skill in sorted({row.get("skill_id") for row in eligible})}
    report = {
        "schema_version": "piper_real_data_audit_v2", "sources": rows,
        "source_count": len(rows), "eligible_episode_count": len(eligible),
        "eligible_frame_count": sum(row.get("eligible_frame_count", 0) for row in eligible),
        "split_ready": split_ready,
        "split_blocker": None if split_ready else "至少需要两个不同的 perturbation_id 场景组",
        "split_rule": "sha256(task_metadata.initial_conditions.perturbation_id) 按完整场景组划分",
        "train_episode_count": sum(row.get("suggested_split") == "train" for row in eligible),
        "test_episode_count": sum(row.get("suggested_split") == "test" for row in eligible),
        "skill_coverage": skills,
        "note": "记录有效性不等于训练契约有效性；候选帧不得作为已验收训练帧。",
    }
    rendered = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if args.output:
        args.output.write_text(rendered, encoding="utf-8")
    else:
        print(rendered, end="")


if __name__ == "__main__":
    main()
