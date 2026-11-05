#!/usr/bin/env python3
"""筛选 Piper 原子技能 LeRobot 数据并准备 π0.5 SFT；运行：python3 tools/vla_contract/atomic_skill_sft.py --help。"""

import argparse
import json
import math
import sys
from pathlib import Path

from piper_pi05_openpi import OPENPI_COMMIT, check_openpi, digest, episode_source_digest, verify_base_checkpoint

SKILLS = {
    "pick_and_place_banana_in_box": {"modes": ("LEFT", "RIGHT"), "roles": None,
                                     "prompt": "抓起香蕉并放入盒子中"},
    "flip_box_lid_bimanual": {"modes": ("BIMANUAL",),
                               "roles": ("hold_box", "flip_lid"),
                               "prompt": "将盒子的盖子翻过来"},
    "latch_box_lid_bimanual": {"modes": ("BIMANUAL",),
                                "roles": ("hold_box", "latch_lid"),
                                "prompt": "扣上盒子的盖子"},
}
HORIZON = 16
STATE_UNIT = "rad (joint1..6), m (gripper)"
ACTION_SPACE = "absolute_joint_position_7"
TIMESTAMP_TOPICS = {"top_rgb": "/camera_top/D455/color/image_raw",
                    "left_wrist_rgb": "/camera_wrist_left/color/image_raw",
                    "right_wrist_rgb": "/camera_wrist_right/color/image_raw"}


def require(condition, reason):
    if not condition:
        raise ValueError(reason)


def read_json(path):
    return json.loads(path.read_text(encoding="utf-8"))


def arms_for(mode):
    return ("left", "right") if mode == "BIMANUAL" else (mode.lower(),)


def expected_features(mode):
    dim = 7 * len(arms_for(mode))
    images = ("top_rgb", *(f"{arm}_wrist_rgb" for arm in arms_for(mode)))
    return {"observation.state": [dim], "action": [dim],
            **{f"observation.images.{name}": [480, 640, 3] for name in images}}


def check_info(root, mode, expected_episodes, expected_frames, fps):
    info = read_json(root / "meta" / "info.json")
    require(info.get("codebase_version") == "v3.0" and info.get("robot_type") == "piper",
            f"{root}: LeRobot 必须为 Piper v3.0")
    require(info.get("total_episodes") == expected_episodes and info.get("total_frames") == expected_frames,
            f"{root}: Episode/帧数与转换清单不符")
    require(type(info.get("fps")) in (int, float) and info["fps"] == fps,
            f"{root}: FPS 与转换清单不符")
    features = info.get("features", {})
    for name, shape in expected_features(mode).items():
        feature = features.get(name, {})
        dtype = "float32" if name in ("observation.state", "action") else "image"
        require(feature.get("dtype") == dtype and feature.get("shape") == shape,
                f"{root}: {name} 的 dtype/shape 不符")
    try:
        import pyarrow.parquet as pq
    except ImportError as error:
        raise ValueError("缺少 pyarrow，无法读取 LeRobot task") from error
    task_path = root / "meta" / "tasks.parquet"
    schema = pq.read_schema(task_path)
    if "task" in schema.names:
        task_column = "task"
    else:
        pandas_meta = json.loads(schema.metadata.get(b"pandas", b"{}").decode()) if schema.metadata else {}
        require("__index_level_0__" in schema.names
                and pandas_meta.get("index_columns") == ["__index_level_0__"],
                f"{root}: LeRobot 任务文本列缺失")
        task_column = "__index_level_0__"
    tasks = pq.read_table(task_path, columns=["task_index", task_column]).to_pylist()
    require(tasks and all(type(item["task_index"]) is int and item["task_index"] >= 0
                          and isinstance(item[task_column], str) and item[task_column].strip() for item in tasks)
            and len({item["task_index"] for item in tasks}) == len(tasks),
            f"{root}: LeRobot 任务表无效")
    return {item["task_index"]: item[task_column] for item in tasks}


def parquet_rows(root, mode):
    try:
        import pyarrow.parquet as pq
    except ImportError as error:
        raise ValueError("缺少 pyarrow，无法读取真实 LeRobot action/state") from error
    names = ["episode_index", "frame_index", "timestamp", "task_index", "observation.state", "action"]
    image_names = set(expected_features(mode)) - {"observation.state", "action"}
    files = sorted((root / "data").rglob("*.parquet"))
    require(bool(files), f"{root}: 没有真实 LeRobot Parquet 数据")
    grouped = {}
    for file in files:
        schema = pq.read_schema(file)
        require(set(names) | image_names <= set(schema.names), f"{file}: 缺 action/state 或 RGB 列")
        table = pq.read_table(file, columns=names + sorted(image_names))
        require(all(table.column(name).null_count == 0 for name in image_names),
                f"{file}: RGB 列存在空帧")
        table = table.select(names)
        for row in table.to_pylist():
            index = row["episode_index"]
            require(type(index) is int and index >= 0, f"{file}: episode_index 无效")
            grouped.setdefault(index, []).append(row)
    return grouped


def check_vectors(rows, episode_id, dim, fps, tasks, task_text):
    require(bool(rows), f"{episode_id}: 没有 LeRobot 帧")
    rows.sort(key=lambda row: row["frame_index"])
    require([row["frame_index"] for row in rows] == list(range(len(rows))),
            f"{episode_id}: frame_index 不连续")
    stamps = [row["timestamp"] for row in rows]
    require(all(type(value) in (int, float) and math.isfinite(value) for value in stamps)
            and all(b > a for a, b in zip(stamps, stamps[1:])),
            f"{episode_id}: LeRobot timestamp 非有限数或不递增")
    require(all(b - a <= 1.5 / fps for a, b in zip(stamps, stamps[1:])),
            f"{episode_id}: LeRobot 序列有时间缺口")
    require(all(type(row["task_index"]) is int and tasks.get(row["task_index"]) == task_text
                for row in rows), f"{episode_id}: task_index 与来源任务文本不符")
    for row in rows:
        for key in ("observation.state", "action"):
            vector = row[key]
            require(isinstance(vector, list) and len(vector) == dim
                    and all(type(value) in (int, float) and math.isfinite(value) for value in vector),
                    f"{episode_id}: {key} 不是真实有限的 {dim} 维数值")
            require(all(vector[index] >= 0 for index in range(6, dim, 7)),
                    f"{episode_id}: {key} 夹爪开口为负")
    require(len(rows) >= HORIZON, f"{episode_id}: 不足 16 帧")


def check_source(row, annotation, mode, skill):
    source = Path(row.get("source", ""))
    require(source.is_absolute() and row.get("source_sha256") == episode_source_digest(source),
            f"{row.get('episode_id')}: 原始 Episode 摘要不匹配")
    meta = read_json(source / "episode.json")
    task = meta.get("task_metadata", {})
    task_text = meta.get("task")
    initial = task.get("initial_conditions")
    group_id = initial.get("perturbation_id") if isinstance(initial, dict) else None
    require(meta.get("schema_version") == 2 and meta.get("mode") == "skill"
            and meta.get("state") == "saved" and meta.get("episode_id") == row["episode_id"]
            and meta.get("skill") == skill and meta.get("outcome") == "success"
            and isinstance(task_text, str) and bool(task_text.strip())
            and task_text.strip() == row.get("task_text") and row.get("skill_id") == skill
            and isinstance(group_id, str) and bool(group_id.strip())
            and group_id.strip() == row.get("group_id")
            and task.get("arm_mode") == mode and task.get("time_domain") == "unix_ns"
            and task.get("action_space") == ACTION_SPACE
            and task.get("action_provenance") == "human_demonstration",
            f"{row['episode_id']}: 原始 Episode 状态/动作来源未核验")
    marks = meta.get("annotations")
    require(isinstance(marks, list), f"{row['episode_id']}: 缺少 v2 技能标注")
    outcome = [item for item in marks if isinstance(item, dict) and item.get("kind") == "outcome"]
    complete = [item for item in marks if isinstance(item, dict) and item.get("kind") == "skill_complete"]
    quality = row.get("annotation_quality")
    frame_topics = meta.get("frame_topics")
    require(len(outcome) == len(complete) == 1 and outcome[0].get("value") == "success"
            and complete[0].get("value") == skill
            and isinstance(complete[0].get("source_topic"), str)
            and isinstance(frame_topics, list) and complete[0]["source_topic"] in frame_topics
            and type(complete[0].get("header_stamp_ns")) is int
            and complete[0]["header_stamp_ns"] > 0
            and isinstance(quality, dict) and quality.get("completion_frame") == {
                "source_topic": complete[0]["source_topic"],
                "header_stamp_ns": complete[0]["header_stamp_ns"]},
            f"{row['episode_id']}: 成功完成帧与来源标注不符")
    require(task.get("calibration_version") == row.get("calibration_version")
            == annotation.get("calibration_version")
            and task.get("normalization_version") == row.get("normalization_version")
            == annotation.get("normalization_version")
            and all(isinstance(task.get(field), str) and task[field].strip()
                    for field in ("calibration_version", "normalization_version")),
            f"{row['episode_id']}: 标定或归一化版本不一致")
    require(annotation.get("skill_id") == skill and annotation.get("success") is True
            and annotation.get("verified") is True and annotation.get("atomic_episode") is True
            and annotation.get("action_provenance") == "human_demonstration"
            and annotation.get("calibration_verified") is True
            and annotation.get("units_verified") is True
            and annotation.get("gripper_mapping_verified") is True
            and isinstance(annotation.get("gripper_mapping_version"), str)
            and bool(annotation["gripper_mapping_version"].strip())
            and annotation.get("source_sha256") == row["source_sha256"],
            f"{row['episode_id']}: 缺少绑定来源的成功、原子技能或夹爪核验")
    if mode == "BIMANUAL":
        roles = annotation.get("roles")
        require(isinstance(roles, dict) and set(roles) == {"left", "right"}
                and set(roles.values()) == set(SKILLS[skill]["roles"])
                and annotation.get("left_right_sync_verified") is True,
                f"{row['episode_id']}: 双臂角色或同步核验缺失")


def check_source_timestamps(path, episodes, mode, input_format, max_sync_ns, max_age_ns):
    records = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
    by_key = {}
    for item in records:
        key = (item.get("split"), item.get("episode_id"))
        by_key.setdefault(key, []).append(item)
    require(set(by_key) == {(row["split"], row["episode_id"]) for row in episodes},
            "来源时间清单与 Episode 不一致")
    arms = arms_for(mode)
    topics = [TIMESTAMP_TOPICS["top_rgb"], *(TIMESTAMP_TOPICS[f"{arm}_wrist_rgb"] for arm in arms)]
    states = [f"/piper_vector/{arm}/slave_feedback" if input_format == "can-vector" else f"/joint_{arm}"
              for arm in arms]
    actions = [f"/piper_vector/{arm}/master_target" if input_format == "can-vector"
               else f"/demonstration/action_{arm}" for arm in arms]
    references = {}
    for row in episodes:
        items = by_key[row["split"], row["episode_id"]]
        require(len(items) == row["frames"]
                and sorted(item.get("frame_index") for item in items) == list(range(row["frames"])),
                f"{row['episode_id']}: 来源时间帧数或顺序不符")
        references[row["split"], row["episode_id"]] = []
        for item in items:
            stamps = item.get("header_timestamp_ns", {})
            require(set(topics + states + actions) <= set(stamps)
                    and all(type(stamps[key]) is int and stamps[key] > 0 for key in topics + states + actions),
                    f"{row['episode_id']}: 缺 RGB/状态/示教动作 Header 时间")
            reference = max(stamps[key] for key in actions)
            references[row["split"], row["episode_id"]].append((item["frame_index"], reference))
            require(max(stamps[key] for key in actions) - min(stamps[key] for key in actions) <= max_sync_ns
                    and max(stamps[key] for key in states) - min(stamps[key] for key in states) <= max_sync_ns,
                    f"{row['episode_id']}: 左右臂时间不同步")
            require(all(0 <= reference - stamps[key] <= max_age_ns for key in topics + states),
                    f"{row['episode_id']}: RGB 或状态超窗")
            sources = [stamps[key] for key in topics + states + actions]
            require(max(sources) - min(sources) <= max_sync_ns,
                    f"{row['episode_id']}: RGB、状态与动作 Header 总体不同步")
            if input_format == "can-vector":
                sources = item.get("source_can_stamps_ns", {})
                require(set(states + actions) <= set(sources)
                        and all(isinstance(sources[key], list) and len(sources[key]) == 4
                                and all(type(value) is int and value > 0 for value in sources[key])
                                and max(sources[key]) - min(sources[key]) <= max_sync_ns
                                for key in states + actions),
                        f"{row['episode_id']}: 原始 CAN 四帧不同步或缺失")
        ordered = sorted(references[row["split"], row["episode_id"]])
        times = [value for _, value in ordered]
        require(all(b > a for a, b in zip(times, times[1:])),
                f"{row['episode_id']}: 来源动作时间不递增")
        references[row["split"], row["episode_id"]] = times
    return references


def inspect(dataset, annotations_path, audit_path, skill, max_sync_ns, max_age_ns,
            min_train=10, min_test=3):
    require(skill in SKILLS, "未知原子技能")
    require(type(max_sync_ns) is int and 0 < max_sync_ns <= 30_000_000
            and type(max_age_ns) is int and 0 < max_age_ns <= 250_000_000,
            "必须提供同步 ≤30 ms、源年龄 ≤250 ms 的正数门限")
    manifest_path = dataset / "conversion_manifest.json"
    manifest, annotations, audit = read_json(manifest_path), read_json(annotations_path), read_json(audit_path)
    mode = manifest.get("arm_mode")
    require(mode in SKILLS[skill]["modes"] and manifest.get("format") == "LeRobotDataset v3.0"
            and manifest.get("lerobot_version") == "0.4.4"
            and manifest.get("purpose") == "success_sft"
            and manifest.get("action_space") == ACTION_SPACE
            and manifest.get("state_unit") == STATE_UNIT
            and manifest.get("input_format") in ("joint-state", "can-vector"),
            "转换清单与固定技能、动作空间或 LeRobot v3.0 不符")
    joint_names = [f"{arm}.{joint}" for arm in arms_for(mode)
                   for joint in [*(f"joint{i}" for i in range(1, 7)), "gripper"]]
    require(manifest.get("joint_names") == joint_names, "关节顺序或左右臂拼接顺序不符")
    episodes = manifest.get("episodes")
    require(isinstance(episodes, list) and episodes, "0 个合格真实 Episode：转换清单为空")
    ids = [row.get("episode_id") for row in episodes]
    require(all(isinstance(value, str) and value for value in ids) and len(set(ids)) == len(ids),
            "Episode ID 缺失或重复")
    require(annotations.get("schema_version") == "piper_atomic_skill_labels/1"
            and annotations.get("conversion_manifest_sha256") == digest(manifest_path)
            and annotations.get("source_audit_sha256") == digest(audit_path),
            "原子技能标注未绑定转换清单与独立审计")
    labels = annotations.get("episodes")
    require(isinstance(labels, dict) and set(labels) == set(ids), "必须逐 Episode 提供核验标注")
    audit_rows = audit.get("sources", [])
    audited = {row.get("episode_id"): row for row in audit_rows if row.get("eligible") is True}
    require(audit.get("schema_version") == "piper_real_data_audit_v2"
            and audit.get("split_ready") is True and set(audited) >= set(ids),
            "独立来源审计未批准所有真实 Episode")
    counts = {"train": 0, "test": 0}
    groups = {"train": set(), "test": set()}
    versions = {"calibration_version": set(), "normalization_version": set(), "gripper_mapping_version": set()}
    role_maps = set()
    for row in episodes:
        episode_id = row["episode_id"]
        split = row.get("split")
        require(split in counts and type(row.get("frames")) is int and row["frames"] >= HORIZON,
                f"{episode_id}: 划分无效或不足 16 帧")
        audit_row = audited[episode_id]
        require(audit_row.get("kind") == "native_episode"
                and audit_row.get("source") == row.get("source")
                and audit_row.get("skill_id") == skill
                and audit_row.get("group_id") == row.get("group_id")
                and audit_row.get("task_text") == row.get("task_text")
                and audit_row.get("suggested_split") == split
                and audit_row.get("has_action_target") is True
                and audit_row.get("has_feedback") is True
                and audit_row.get("has_rgb") is True
                and audit_row.get("header_time_domain_declared") is True,
                f"{episode_id}: 独立审计来源、技能、场景组、任务文本、切分或传感器条件不一致")
        check_source(row, labels[episode_id], mode, skill)
        if mode == "BIMANUAL":
            role_maps.add(tuple(labels[episode_id]["roles"][arm] for arm in ("left", "right")))
        counts[split] += 1
        groups[split].add(row["group_id"])
        for field in versions:
            value = labels[episode_id][field] if field == "gripper_mapping_version" else row[field]
            versions[field].add(value)
    require(counts["train"] >= min_train and counts["test"] >= min_test,
            f"0 个合格真实 Episode 或训练/测试数量不足：{counts}；最低 {min_train}/{min_test}")
    require(not groups["train"].intersection(groups["test"]),
            "同一 perturbation_id 场景组跨越训练/测试划分")
    require(all(len(values) == 1 for values in versions.values()), "标定、归一化或夹爪映射版本混用")
    require(len(role_maps) <= 1, "双臂左右角色分配在 Episode 间混用")
    fps = manifest.get("fps")
    require(type(fps) is int and fps > 0, "转换清单 FPS 无效")
    parquet_times = {}
    task_texts = set()
    for split in counts:
        rows_for_split = [row for row in episodes if row["split"] == split]
        root = dataset / split
        tasks = check_info(root, mode, len(rows_for_split), sum(row["frames"] for row in rows_for_split), fps)
        grouped = parquet_rows(root, mode)
        require(set(grouped) == set(range(len(rows_for_split))), f"{split}: Parquet Episode 数量/索引不符")
        for index, row in enumerate(rows_for_split):
            require(len(grouped[index]) == row["frames"], f"{row['episode_id']}: Parquet 帧数不符")
            check_vectors(grouped[index], row["episode_id"], 7 * len(arms_for(mode)), fps,
                          tasks, row["task_text"])
            task_texts.add(row["task_text"])
            parquet_times[split, row["episode_id"]] = [frame["timestamp"] for frame in grouped[index]]
    references = check_source_timestamps(dataset / "source_timestamps.jsonl", episodes, mode,
                                         manifest["input_format"], max_sync_ns, max_age_ns)
    for key, stamps in parquet_times.items():
        source = references[key]
        require(all(abs((timestamp - stamps[0]) - (source[index] - source[0]) / 1e9) <= 0.002
                    for index, timestamp in enumerate(stamps)),
                f"{key[1]}: LeRobot 时间与来源动作 Header 不对应")
    return {"status": "data_gate_passed_launch_blocked", "skill_id": skill, "arm_mode": mode,
            "counts": counts, "frames": sum(row["frames"] for row in episodes),
            "versions": {field: next(iter(values)) for field, values in versions.items()},
            "conversion_manifest_sha256": digest(manifest_path), "annotations_sha256": digest(annotations_path),
            "source_audit_sha256": digest(audit_path), "openpi_commit": OPENPI_COMMIT,
            "model": {"family": "pi05_flow", "internal_action_dim": 32,
                      "piper_state_dim": 7 * len(arms_for(mode)),
                      "piper_action_dim": 7 * len(arms_for(mode)), "action_horizon": HORIZON,
                      "action_space": ACTION_SPACE, "rgb_slots": list(expected_features(mode))[2:],
                      "prompt": next(iter(task_texts)) if len(task_texts) == 1 else None,
                      "task_texts": sorted(task_texts),
                      "base_checkpoint": "gs://openpi-assets/checkpoints/pi05_droid",
                      "normalization": "仅从通过门禁的 Piper train Episode 重新计算"},
            "bimanual_roles": (dict(zip(("left", "right"), next(iter(role_maps)))) if role_maps else None),
            "launch_blockers": ["固定 openpi commit 使用 LeRobot v2.1 布局与旧读取 API，当前转换器输出 v3.0",
                                *(["尚无经过实测的 Piper 双臂 14 维 π0.5 输入/输出适配"]
                                  if mode == "BIMANUAL" else [])]}


def main():
    parser = argparse.ArgumentParser(description="固定原子技能 π0.5 SFT 数据门禁；默认只读 dry-run")
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--annotations", type=Path, required=True)
    parser.add_argument("--audit-report", type=Path, required=True)
    parser.add_argument("--skill", choices=SKILLS, required=True)
    parser.add_argument("--max-sync-ns", type=int, required=True)
    parser.add_argument("--max-source-age-ns", type=int, required=True)
    parser.add_argument("--min-train", type=int, default=10)
    parser.add_argument("--min-test", type=int, default=3)
    parser.add_argument("--steps", type=int, default=20000)
    parser.add_argument("--batch-size", type=int, default=8)
    parser.add_argument("--plan", type=Path, help="通过后写入准备清单；默认仅打印")
    parser.add_argument("--openpi-root", type=Path, help="--launch 时核对的固定官方 checkout")
    parser.add_argument("--base-checkpoint", type=Path, help="--launch 时核对的完整官方 checkpoint")
    parser.add_argument("--launch", action="store_true", help="尝试启动训练；当前 v3/v2.1 不兼容时明确拒绝")
    args = parser.parse_args()
    try:
        require(args.min_train >= 10 and args.min_test >= 3 and args.steps > 0 and args.batch_size > 0,
                "每技能至少需要 10/3 个训练/测试 Episode，steps 和 batch-size 必须为正")
        result = inspect(args.dataset, args.annotations, args.audit_report, args.skill,
                         args.max_sync_ns, args.max_source_age_ns, args.min_train, args.min_test)
        result["training"] = {"steps": args.steps, "batch_size": args.batch_size,
                              "evaluation_split": "test", "execute": False}
        if args.plan:
            require(not args.plan.exists(), "准备清单已存在，拒绝覆盖")
            args.plan.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        if args.launch:
            require(args.openpi_root is not None and args.base_checkpoint is not None,
                    "--launch 需要 --openpi-root 与 --base-checkpoint")
            check_openpi(args.openpi_root, args.base_checkpoint)
            verify_base_checkpoint(args.base_checkpoint)
            raise ValueError("训练启动已拒绝：" + "；".join(result["launch_blockers"]))
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0
    except (ValueError, OSError, KeyError, TypeError) as error:
        print(json.dumps({"status": "REJECTED", "skill_id": args.skill, "reason": str(error)},
                         ensure_ascii=False, indent=2))
        return 2


if __name__ == "__main__":
    sys.exit(main())
