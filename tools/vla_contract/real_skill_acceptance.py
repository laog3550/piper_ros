#!/usr/bin/env python3
"""只读核验真实 Piper pick/place 离线预测。运行：python3 tools/vla_contract/real_skill_acceptance.py 数据目录 [--evaluation 评估清单.json]。"""

import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np


ACTION_SPACE = "absolute_joint_position_7"
SKILLS = ("pick", "place")


def digest(path):
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(block)
    return value.hexdigest()


def require(condition, reason):
    if not condition:
        raise ValueError(reason)


def evidence_file(entry, field):
    require(isinstance(entry, dict), f"{field}: 需要路径及 SHA-256")
    path = Path(entry.get("path", "")).expanduser().resolve()
    expected = entry.get("sha256")
    require(path.is_file() and isinstance(expected, str) and len(expected) == 64,
            f"{field}: 文件或 SHA-256 缺失")
    require(digest(path) == expected.lower(), f"{field}: SHA-256 不匹配")
    return path


def inventory(root):
    raw_bags = sorted(root.rglob("metadata.yaml"))
    episodes = []
    for path in sorted(root.rglob("episode_manifest.json")):
        data = json.loads(path.read_text(encoding="utf-8"))
        episodes.append({"episode_id": data.get("episode_id"),
                         "mode": data.get("mode"), "valid": data.get("valid"),
                         "path": str(path.parent),
                         "action_topic": "/dataset/can_left/action_target" in data.get("available_topics", [])})
    candidates = []
    for path in sorted(root.rglob("candidate_manifest.json")):
        data = json.loads(path.read_text(encoding="utf-8"))
        candidates.append({"episode_id": data.get("episode_id"), "path": str(path.parent),
                           "sample_count": data.get("sample_count"),
                           "training_ready": data.get("training_ready"),
                           "max_internal_gap_seconds": data.get("max_internal_gap_seconds"),
                           "pending": data.get("pending", [])})
    return {"raw_bag_count": len(raw_bags), "episodes": episodes, "candidates": candidates}


def arrays(path, names):
    with np.load(path, allow_pickle=False) as data:
        require(set(names) <= set(data.files), f"{path}: 缺少 {sorted(set(names) - set(data.files))}")
        return {name: data[name] for name in names}


def evaluate(manifest_path, root):
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    require(manifest.get("schema_version") == "piper_real_skill_eval_v1", "评估清单版本不符")
    require(manifest.get("action_space") == ACTION_SPACE, "模型动作空间不是 Piper 绝对关节位置 7 维")
    require(manifest.get("state_space") == "piper_joint_position_7", "状态空间不是 Piper 实测关节位置 7 维")
    checkpoint = evidence_file(manifest.get("checkpoint"), "checkpoint")
    normalization = evidence_file(manifest.get("normalization"), "normalization")
    version = manifest.get("normalization_version")
    require(isinstance(version, str) and version.strip(), "归一化版本缺失")
    require(manifest.get("checkpoint_normalization_sha256") == digest(normalization),
            "checkpoint 与归一化文件未绑定")
    require(checkpoint.stat().st_size > 0 and normalization.stat().st_size > 0, "模型或归一化文件为空")
    episodes = manifest.get("episodes")
    require(isinstance(episodes, list) and len(episodes) >= 2, "至少需要独立训练与验证 Episode")
    seen_ids, seen_sources, seen_bags = set(), set(), set()
    split_ids = {"train": set(), "validation": set()}
    metrics = {skill: {"episodes": 0, "samples": 0, "errors": 0,
                       "joint_error_sum": 0.0, "gripper_error_sum": 0.0,
                       "latencies_ms": []} for skill in SKILLS}
    for entry in episodes:
        require(isinstance(entry, dict), "Episode 条目必须是对象")
        episode_id, split = entry.get("episode_id"), entry.get("split")
        require(isinstance(episode_id, str) and episode_id and episode_id not in seen_ids,
                "Episode ID 缺失或重复")
        require(split in split_ids, f"{episode_id}: split 必须是 train 或 validation")
        seen_ids.add(episode_id)
        split_ids[split].add(episode_id)
        source = evidence_file(entry.get("source_manifest"), f"{episode_id}.source_manifest")
        require(source.is_relative_to(root.resolve()), f"{episode_id}: 原始 Episode 不在指定数据目录")
        source_meta = json.loads(source.read_text(encoding="utf-8"))
        require(source_meta.get("episode_id") == episode_id and source_meta.get("valid") is True,
                f"{episode_id}: 原始 Episode 身份或完整性未核实")
        bag = evidence_file(entry.get("source_bag"), f"{episode_id}.source_bag")
        require(bag.parent == source.parent and bag.suffix in (".db3", ".zstd"),
                f"{episode_id}: bag 文件未绑定原始 Episode")
        require(digest(bag) not in seen_bags, "训练与验证复用相同 bag 内容")
        seen_bags.add(digest(bag))
        require(source.parent not in seen_sources, "训练与验证复用同一原始 Episode")
        seen_sources.add(source.parent)
        if split == "train":
            continue
        require(source_meta.get("mode") == "direct_demonstration" and
                "/dataset/can_left/action_target" in source_meta.get("available_topics", []) and
                "/dataset/can_left/feedback_observed" in source_meta.get("available_topics", []),
                f"{episode_id}: 缺少真实 Piper 状态或动作")
        data_path = evidence_file(entry.get("aligned_data"), f"{episode_id}.aligned_data")
        predictions_path = evidence_file(entry.get("predictions"), f"{episode_id}.predictions")
        require(entry.get("prediction_checkpoint_sha256") == digest(checkpoint)
                and entry.get("prediction_normalization_sha256") == digest(normalization),
                f"{episode_id}: 预测文件未绑定模型与归一化版本")
        labels_path = evidence_file(entry.get("skill_labels"), f"{episode_id}.skill_labels")
        require(data_path.is_relative_to(root.resolve()), f"{episode_id}: 对齐数据不在指定数据目录")
        candidate_path = data_path.parent / "candidate_manifest.json"
        require(candidate_path.is_file(), f"{episode_id}: 缺少候选数据质量清单")
        candidate = json.loads(candidate_path.read_text(encoding="utf-8"))
        require(candidate.get("episode_id") == episode_id and candidate.get("training_ready") is True
                and candidate.get("format_valid") is True and not candidate.get("pending")
                and not candidate.get("quality_errors"), f"{episode_id}: 示教质量尚未通过核验")
        require(isinstance(candidate.get("requested_fps"), (int, float))
                and candidate["requested_fps"] > 0
                and isinstance(candidate.get("max_internal_gap_seconds"), (int, float))
                and candidate["max_internal_gap_seconds"] <= 1.5 / candidate["requested_fps"],
                f"{episode_id}: 内部数据缺口超过 1.5 个采样周期")
        label_meta = json.loads(labels_path.read_text(encoding="utf-8"))
        require(label_meta.get("episode_id") == episode_id and label_meta.get("verified") is True
                and label_meta.get("source_manifest_sha256") == digest(source),
                f"{episode_id}: 技能标签未经人工核实或未绑定来源")
        data = arrays(data_path, ("timestamp_ns", "state", "action"))
        pred = arrays(predictions_path, ("timestamp_ns", "action", "latency_ms", "error"))
        count = len(data["timestamp_ns"])
        require(count > 0 and all(len(value) == count for value in (*data.values(), *pred.values()))
                and np.array_equal(data["timestamp_ns"], pred["timestamp_ns"]),
                f"{episode_id}: 预测与原始样本未逐条对齐")
        require(np.all(np.diff(data["timestamp_ns"]) > 0), f"{episode_id}: 时间戳非递增")
        require(data["state"].shape == data["action"].shape == pred["action"].shape == (count, 7)
                and np.isfinite(data["state"]).all() and np.isfinite(data["action"]).all()
                and np.all(data["state"][:, 6] >= 0) and np.all(data["action"][:, 6] >= 0),
                f"{episode_id}: Piper 状态或动作维度/有限性错误")
        require(candidate.get("sample_count") == count, f"{episode_id}: 样本数与质量清单不符")
        require(pred["error"].shape == pred["latency_ms"].shape == (count,)
                and np.isin(pred["error"], [0, 1]).all(), f"{episode_id}: 错误标志无效")
        labels = label_meta.get("labels")
        require(isinstance(labels, list) and len(labels) == count and set(labels) <= set(SKILLS),
                f"{episode_id}: 需要逐样本 pick/place 标签")
        for skill in SKILLS:
            indexes = np.flatnonzero(np.asarray(labels) == skill)
            if not len(indexes):
                continue
            item = metrics[skill]
            item["episodes"] += 1
            item["samples"] += len(indexes)
            item["errors"] += int(pred["error"][indexes].sum())
            valid = indexes[pred["error"][indexes] == 0]
            require(np.isfinite(pred["action"][valid]).all()
                    and np.isfinite(pred["latency_ms"][valid]).all()
                    and np.all(pred["latency_ms"][valid] >= 0),
                    f"{episode_id}: 预测或耗时无效")
            delta = np.abs(pred["action"][valid] - data["action"][valid])
            item["joint_error_sum"] += float(delta[:, :6].sum())
            item["gripper_error_sum"] += float(delta[:, 6].sum())
            item["latencies_ms"].extend(pred["latency_ms"][valid].tolist())
    require(split_ids["train"] and split_ids["validation"], "训练/验证 Episode 未隔离")
    result = {}
    for skill, item in metrics.items():
        require(item["samples"] > 0, f"验证集没有经核实的 {skill} 样本")
        valid_count = item["samples"] - item["errors"]
        result[skill] = {"validation_episodes": item["episodes"], "samples": item["samples"],
                         "errors": item["errors"], "error_rate": item["errors"] / item["samples"],
                         "joint_mae_rad": item["joint_error_sum"] / (valid_count * 6) if valid_count else None,
                         "gripper_mae_m": item["gripper_error_sum"] / valid_count if valid_count else None,
                         "latency_p50_ms": float(np.percentile(item["latencies_ms"], 50)) if valid_count else None,
                         "latency_p95_ms": float(np.percentile(item["latencies_ms"], 95)) if valid_count else None}
    return {"status": "离线预测指标已核验；不代表真实执行成功率", "checkpoint_sha256": digest(checkpoint),
            "normalization_version": version, "normalization_sha256": digest(normalization),
            "train_episodes": len(split_ids["train"]), "validation_episodes": len(split_ids["validation"]),
            "skills": result}


def main():
    parser = argparse.ArgumentParser(description="真实 Piper 技能离线验收；只读，不连接 ROS/CAN")
    parser.add_argument("data_root", type=Path)
    parser.add_argument("--evaluation", type=Path, help="训练任务输出的离线评估清单")
    args = parser.parse_args()
    report = {"data_root": str(args.data_root.resolve()), "inventory": inventory(args.data_root),
              "minimum_data_requirements": [
                  "至少两个来源独立且带原始 bag 摘要的真实 Piper Episode，按 Episode 隔离训练/验证",
                  "验证 Episode 需有经核实的 pick 与 place 逐样本标签、实测状态、人工示教动作及通过质量核验的对齐数据",
                  "Piper absolute_joint_position_7 checkpoint、绑定的归一化文件和版本、逐样本离线预测及耗时"]}
    try:
        require(args.data_root.is_dir(), "数据目录不存在")
        require(args.evaluation is not None, "缺少真实 Piper checkpoint、独立验证 Episode 与评估清单")
        report["evaluation"] = evaluate(args.evaluation, args.data_root)
        code = 0
    except (ValueError, OSError, KeyError, TypeError) as error:
        report["evaluation"] = {"status": "FAIL", "reason": str(error)}
        code = 2
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return code


if __name__ == "__main__":
    sys.exit(main())
