"""合成 v2 技能 Episode 的审计与成功 SFT 导出。运行：pytest -q tools/episode_recorder/test_atomic_skill_export.py。"""

import json
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).parent))
sys.path.insert(0, str(Path(__file__).parents[1] / "vla_contract"))
import audit_real_data as audit
import to_lerobot as exporter
from atomic_skill_sft import inspect
from piper_pi05_openpi import digest
from test_to_lerobot import fixture, vector_fixture


def annotate(episode, skill="pick_and_place_banana_in_box", task="抓起香蕉并放入盒子中", outcome="success",
             completion_index=-1, resolved=True, group_id="auto"):
    path = episode / "episode.json"
    meta = json.loads(path.read_text(encoding="utf-8"))
    arms = ["left", "right"] if skill in ("flip_box_lid_bimanual", "latch_box_lid_bimanual") else ["left"]
    input_format = "can-vector" if len(arms) == 2 else "joint-state"
    checked, _ = exporter.read_episode(episode, arms, 10, 200_000_000, 2_000_000,
                                       input_format=input_format,
                                       max_camera_gap_ns=200_000_000,
                                       max_camera_skew_ns=200_000_000)
    topic = exporter.CAMERAS["top_rgb"]
    stamp = checked["_camera_header_stamps"][topic][completion_index]
    meta.update(schema_version=2, mode="skill", skill=skill, active_skill=skill,
                frame_topics=[exporter.CAMERAS["top_rgb"]],
                task=task, outcome=outcome)
    if group_id is not None:
        meta["task_metadata"]["initial_conditions"] = {
            "perturbation_id": episode.name if group_id == "auto" else group_id}
    recorded = meta["stopped_at_unix_ns"] - 10_000_000
    complete = {"kind": "skill_complete", "value": skill, "recorded_at_unix_ns": recorded}
    if resolved:
        complete.update(source_topic=topic, header_stamp_ns=stamp)
    else:
        complete["frame_resolution_required"] = True
    meta["annotations"] = [complete, {"kind": "outcome", "value": outcome,
                                         "recorded_at_unix_ns": recorded + 1}]
    path.write_text(json.dumps(meta, ensure_ascii=False), encoding="utf-8")
    return meta


def args(source, output, arm_mode="LEFT", input_format="joint-state", skills=None):
    return SimpleNamespace(input_dir=source, output_dir=output, repo_id="local/piper_atomic",
                           arm_mode=arm_mode, input_format=input_format, fps=10,
                           max_age_ms=30, jitter_ms=2, max_can_skew_ms=30,
                           max_camera_gap_ms=100, max_camera_skew_ms=30,
                           max_source_skew_ms=30,
                           test_percent=50, success_sft=True, skills=skills)


def checked_audit(path, **kwargs):
    return audit.audit_native(path, max_source_skew_ms=30, **kwargs)


def test_v2_audit_requires_resolved_success_and_complete_camera_frame(tmp_path):
    good = fixture(tmp_path, "episode_000001")
    annotate(good)
    row = checked_audit(good, max_age_ms=30)
    assert row["eligible"] and row["eligible_frame_count"] == 2
    assert row["annotation_coverage"] == 1.0
    assert row["skill_id"] == "pick_and_place_banana_in_box"
    assert row["quality"]["max_selected_camera_skew_ns"] == 0
    assert row["quality"]["completion_frame"]["header_stamp_ns"] > 0

    save_labeled = fixture(tmp_path, "episode_000007")
    save_meta = annotate(save_labeled)
    save_meta["annotations"] = [item for item in save_meta["annotations"]
                                if item["kind"] == "skill_complete"]
    save_meta["saved_at_unix_ns"] = save_meta["stopped_at_unix_ns"] + 2
    save_meta["annotations"].append({"kind": "outcome", "value": "success",
                                     "annotation_origin": "save_selection",
                                     "reviewed_at_unix_ns": save_meta["stopped_at_unix_ns"] + 1})
    (save_labeled / "episode.json").write_text(json.dumps(save_meta), encoding="utf-8")
    assert checked_audit(save_labeled, max_age_ms=30)["eligible"]

    reviewed = fixture(tmp_path, "episode_000008")
    review_meta = annotate(reviewed)
    review_meta["saved_at_unix_ns"] = review_meta["stopped_at_unix_ns"] + 2
    review_meta["annotations"][0].pop("recorded_at_unix_ns")
    review_meta["annotations"][0].update(annotation_origin="post_recording_review",
                                         reviewed_at_unix_ns=review_meta["stopped_at_unix_ns"] + 1)
    (reviewed / "episode.json").write_text(json.dumps(review_meta), encoding="utf-8")
    assert checked_audit(reviewed, max_age_ms=30)["eligible"]

    automatic = fixture(tmp_path, "episode_000009")
    automatic_meta = annotate(automatic)
    automatic_meta["saved_at_unix_ns"] = automatic_meta["stopped_at_unix_ns"] + 2
    automatic_meta["annotations"][0].pop("recorded_at_unix_ns")
    automatic_meta["annotations"][0].update(annotation_origin="duration_limit_last_frame",
                                            reviewed_at_unix_ns=automatic_meta["stopped_at_unix_ns"] + 1)
    (automatic / "episode.json").write_text(json.dumps(automatic_meta), encoding="utf-8")
    assert checked_audit(automatic, max_age_ms=30)["eligible"]

    unresolved = fixture(tmp_path, "episode_000002")
    annotate(unresolved, resolved=False)
    row = checked_audit(unresolved, max_age_ms=30)
    assert not row["eligible"] and "完成帧" in " ".join(row["reasons"])

    failure = fixture(tmp_path, "episode_000003")
    annotate(failure, outcome="failure")
    row = checked_audit(failure, max_age_ms=30)
    assert not row["eligible"] and row["eligible_frame_count"] == 0

    absent = fixture(tmp_path, "episode_000004")
    row = checked_audit(absent, max_age_ms=30)
    assert not row["eligible"] and "v2" in " ".join(row["reasons"])

    task_episode = fixture(tmp_path, "episode_000005")
    meta = annotate(task_episode)
    meta["mode"] = "task"
    (task_episode / "episode.json").write_text(json.dumps(meta), encoding="utf-8")
    row = checked_audit(task_episode, max_age_ms=30)
    assert not row["eligible"] and "task 模式" in " ".join(row["reasons"])

    missing_group = fixture(tmp_path, "episode_000006")
    annotate(missing_group, group_id=None)
    row = checked_audit(missing_group, max_age_ms=30)
    assert not row["eligible"] and "perturbation_id" in " ".join(row["reasons"])


def test_v2_audit_rejects_unmapped_completion_and_camera_gap(tmp_path):
    episode = fixture(tmp_path, "episode_000001")
    meta = annotate(episode)
    meta["annotations"][0]["header_stamp_ns"] += 1
    (episode / "episode.json").write_text(json.dumps(meta), encoding="utf-8")
    row = checked_audit(episode, max_age_ms=30)
    assert not row["eligible"] and "未在该 Episode" in " ".join(row["reasons"])

    undeclared = fixture(tmp_path, "episode_000004")
    meta = annotate(undeclared)
    meta["frame_topics"] = []
    (undeclared / "episode.json").write_text(json.dumps(meta), encoding="utf-8")
    row = checked_audit(undeclared, max_age_ms=30)
    assert not row["eligible"] and "完成帧" in " ".join(row["reasons"])

    episode2 = fixture(tmp_path, "episode_000002")
    annotate(episode2)
    row = checked_audit(episode2, max_age_ms=30, max_camera_gap_ms=50)
    assert not row["eligible"] and "连续性阈值" in " ".join(row["reasons"])

    episode3 = fixture(tmp_path, "episode_000003", wrist_lag_ns=50_000_000)
    annotate(episode3)
    row = checked_audit(episode3, max_age_ms=100, max_camera_skew_ms=30)
    assert not row["eligible"] and "同步阈值" in " ".join(row["reasons"])


def test_state_camera_source_skew_requires_explicit_bound(tmp_path):
    episode = fixture(tmp_path, "episode_000001", state_lag_ns=50_000_000)
    annotate(episode)
    no_bound = audit.audit_native(episode, max_age_ms=100)
    assert not no_bound["eligible"] and "--max-source-skew-ms" in " ".join(no_bound["reasons"])
    row = checked_audit(episode, max_age_ms=100)
    assert not row["eligible"] and "总体时间差" in " ".join(row["reasons"])
    meta, _ = exporter.read_episode(episode, ["left"], 10, 100_000_000, 2_000_000)
    assert meta["_quality"]["max_selected_camera_skew_ns"] == 0
    assert meta["_quality"]["max_selected_source_skew_ns"] == 50_000_000


def test_audit_rejects_clock_step_and_nonmonotonic_camera_header(tmp_path):
    episode = fixture(tmp_path, "episode_000001")
    meta = annotate(episode)
    meta["clock_adjustments"] = [{"observed_at_unix_ns": 1_800_000_100_000_000_000,
                                   "offset_delta_ns": 700_000_000}]
    meta["bag_validation"] = {"frame_stats_by_header_stamp": {
        exporter.CAMERAS["top_rgb"]: {"non_increasing_stamps": 1}}}
    (episode / "episode.json").write_text(json.dumps(meta, ensure_ascii=False), encoding="utf-8")

    row = checked_audit(episode, max_age_ms=100)

    assert not row["eligible"]
    assert "系统时钟跳变" in " ".join(row["reasons"])
    assert "Header 时间戳不单调" in " ".join(row["reasons"])


def test_bimanual_audit_rejects_invalid_action_state(tmp_path):
    episode = vector_fixture(tmp_path, "episode_000001", verified=False)
    # 标注不调用深检；默认未核实的夹爪绝不能获得成功资格。
    path = episode / "episode.json"
    meta = json.loads(path.read_text(encoding="utf-8"))
    stamp = 1_800_000_100_100_000_000
    recorded = meta["stopped_at_unix_ns"] - 10_000_000
    meta["task_metadata"]["initial_conditions"] = {"perturbation_id": episode.name}
    meta.update(schema_version=2, mode="skill", skill="latch_box_lid_bimanual",
                task="双臂合上盒盖", outcome="success",
                frame_topics=[exporter.CAMERAS["top_rgb"]],
                annotations=[{"kind": "skill_complete", "value": "latch_box_lid_bimanual",
                              "source_topic": exporter.CAMERAS["top_rgb"], "header_stamp_ns": stamp,
                              "recorded_at_unix_ns": recorded},
                             {"kind": "outcome", "value": "success",
                              "recorded_at_unix_ns": recorded + 1}])
    path.write_text(json.dumps(meta, ensure_ascii=False), encoding="utf-8")
    row = checked_audit(episode, max_age_ms=30)
    assert not row["eligible"] and "夹爪映射未核实" in " ".join(row["reasons"])


def test_completion_can_resolve_declared_depth_header(tmp_path):
    episode = vector_fixture(tmp_path, "episode_000001")
    meta = annotate(episode, skill="latch_box_lid_bimanual", task="双臂合上盒盖")
    depth = "/camera_top/D455/aligned_depth_to_color/image_raw"
    meta["frame_topics"] = [exporter.CAMERAS["top_rgb"], depth]
    meta["annotations"][0]["source_topic"] = depth
    (episode / "episode.json").write_text(json.dumps(meta, ensure_ascii=False), encoding="utf-8")
    row = checked_audit(episode, max_age_ms=30)
    assert row["eligible"] and row["quality"]["completion_frame"]["source_topic"] == depth


def test_audit_cli_splits_complete_episodes(tmp_path):
    source = tmp_path / "episodes"
    source.mkdir()
    for number in (1, 2):
        annotate(fixture(source, f"episode_{number:06d}"), task=f"抓取香蕉{number}")
    report = tmp_path / "audit.json"
    subprocess.run([sys.executable, str(Path(audit.__file__)), str(source),
                    "--episode-root", str(source), "--output", str(report),
                    "--max-age-ms", "30", "--max-source-skew-ms", "30"], check=True)
    data = json.loads(report.read_text(encoding="utf-8"))
    assert data["source_count"] == data["eligible_episode_count"] == 2
    assert data["train_episode_count"] == data["test_episode_count"] == 1
    assert {row["suggested_split"] for row in data["sources"]} == {"train", "test"}


def test_audit_and_export_keep_perturbation_group_together(tmp_path):
    from importlib.metadata import version
    if version("lerobot") != "0.4.4":
        pytest.skip("需要 lerobot==0.4.4")
    source = tmp_path / "episodes"
    source.mkdir()
    for number, group in ((1, "scene-A"), (2, "scene-A"), (3, "scene-B")):
        annotate(fixture(source, f"episode_{number:06d}"), group_id=group)
    report = tmp_path / "audit.json"
    subprocess.run([sys.executable, str(Path(audit.__file__)), str(source),
                    "--episode-root", str(source), "--output", str(report),
                    "--max-age-ms", "30", "--max-source-skew-ms", "30",
                    "--test-percent", "50"], check=True)
    data = json.loads(report.read_text(encoding="utf-8"))
    assert data["split_ready"]
    assert len({row["suggested_split"] for row in data["sources"]
                if row["group_id"] == "scene-A"}) == 1
    output = tmp_path / "dataset"
    exporter.convert(args(source, output))
    manifest = json.loads((output / "conversion_manifest.json").read_text(encoding="utf-8"))
    assert "perturbation_id" in manifest["split_rule"]
    assert len({item["split"] for item in manifest["episodes"]
                if item["group_id"] == "scene-A"}) == 1
    assert {item["split"] for item in manifest["episodes"]} == {"train", "test"}


def test_success_sft_requires_two_distinct_groups(tmp_path):
    source = tmp_path / "episodes"
    source.mkdir()
    for number in (1, 2):
        annotate(fixture(source, f"episode_{number:06d}"), group_id="same-scene")
    report = tmp_path / "audit.json"
    subprocess.run([sys.executable, str(Path(audit.__file__)), str(source),
                    "--episode-root", str(source), "--output", str(report),
                    "--max-age-ms", "30", "--max-source-skew-ms", "30"], check=True)
    data = json.loads(report.read_text(encoding="utf-8"))
    assert not data["split_ready"] and "两个不同" in data["split_blocker"]
    assert data["train_episode_count"] == data["test_episode_count"] == 0
    with pytest.raises(ValueError, match="两个不同的 perturbation_id"):
        exporter.convert(args(source, tmp_path / "dataset"))
    assert not (tmp_path / "dataset").exists()


def test_success_sft_real_lerobot_roundtrip_and_failure_exclusion(tmp_path):
    from importlib.metadata import version
    if version("lerobot") != "0.4.4":
        pytest.skip("需要 lerobot==0.4.4")
    pytest.importorskip("lerobot.datasets.lerobot_dataset")
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    source = tmp_path / "episodes"
    source.mkdir()
    annotate(fixture(source, "episode_000001"), task="抓取第一根香蕉")
    annotate(fixture(source, "episode_000002"), task="抓取第二根香蕉")
    annotate(fixture(source, "episode_000003"), task="失败抓取", outcome="failure")
    annotate(vector_fixture(source, "episode_000004"), skill="flip_box_lid_bimanual", task="将盒子的盖子翻过来")
    annotate(fixture(source, "episode_000005"), task="缺场景组", group_id=None)
    output = tmp_path / "dataset"
    exporter.convert(args(source, output, skills=["pick_and_place_banana_in_box"]))
    manifest = json.loads((output / "conversion_manifest.json").read_text(encoding="utf-8"))
    assert manifest["purpose"] == "success_sft"
    assert len(manifest["episodes"]) == 2
    assert len(manifest["excluded_episodes"]) == 3
    assert any("perturbation_id" in " ".join(item["reasons"])
               for item in manifest["excluded_episodes"])
    assert {item["split"] for item in manifest["episodes"]} == {"train", "test"}
    assert {item["skill_id"] for item in manifest["episodes"]} == {"pick_and_place_banana_in_box"}
    tasks = set()
    for split in ("train", "test"):
        dataset = LeRobotDataset(f"local/piper_atomic_{split}", root=output / split)
        assert dataset.meta.total_episodes == 1 and len(dataset) == 2
        sample = dataset[0]
        assert sample["action"].shape == (7,)
        assert sample["observation.images.top_rgb"].shape == (3, 480, 640)
        tasks.add(sample["task"])
    assert tasks == {"抓取第一根香蕉", "抓取第二根香蕉"}


def test_v2_export_reaches_atomic_gate_with_external_labels(tmp_path):
    from importlib.metadata import version
    if version("lerobot") != "0.4.4":
        pytest.skip("需要 lerobot==0.4.4")
    pytest.importorskip("lerobot.datasets.lerobot_dataset")

    source = tmp_path / "episodes"
    source.mkdir()
    for index in (1, 2):
        annotate(fixture(source, f"episode_{index:06d}", frame_count=16),
                 task=f"抓取第{index}根香蕉")
    report = tmp_path / "audit.json"
    subprocess.run([sys.executable, str(Path(audit.__file__)), str(source),
                    "--episode-root", str(source), "--output", str(report),
                    "--max-age-ms", "30", "--max-source-skew-ms", "30"], check=True)
    audited = json.loads(report.read_text(encoding="utf-8"))
    assert audited["eligible_episode_count"] == 2

    output = tmp_path / "dataset"
    exporter.convert(args(source, output, skills=["pick_and_place_banana_in_box"]))
    manifest_path = output / "conversion_manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    labels = {}
    for row in manifest["episodes"]:
        labels[row["episode_id"]] = {
            "skill_id": "pick_and_place_banana_in_box", "success": True, "verified": True,
            "atomic_episode": True, "action_provenance": "human_demonstration",
            "source_sha256": row["source_sha256"],
            "calibration_version": row["calibration_version"],
            "normalization_version": row["normalization_version"],
            "calibration_verified": True, "units_verified": True,
            "gripper_mapping_verified": True, "gripper_mapping_version": "synthetic-gripper-v1",
        }
    annotations = tmp_path / "external_labels.json"
    annotations.write_text(json.dumps({
        "schema_version": "piper_atomic_skill_labels/1",
        "conversion_manifest_sha256": digest(manifest_path),
        "source_audit_sha256": digest(report), "episodes": labels,
    }), encoding="utf-8")
    result = inspect(output, annotations, report, "pick_and_place_banana_in_box", 30_000_000, 30_000_000,
                     min_train=1, min_test=1)
    assert result["status"] == "data_gate_passed_launch_blocked"
    assert result["counts"] == {"train": 1, "test": 1}
    assert result["model"]["task_texts"] == ["抓取第1根香蕉", "抓取第2根香蕉"]
    assert result["launch_blockers"]


@pytest.mark.parametrize("skill,task", [
    ("flip_box_lid_bimanual", "将盒子的盖子翻过来"),
    ("latch_box_lid_bimanual", "扣上盒子的盖子"),
])
def test_bimanual_success_sft_real_roundtrip(tmp_path, skill, task):
    from importlib.metadata import version
    if version("lerobot") != "0.4.4":
        pytest.skip("需要 lerobot==0.4.4")
    pytest.importorskip("lerobot.datasets.lerobot_dataset")
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    source = tmp_path / "episodes"
    source.mkdir()
    for number in (1, 2):
        annotate(vector_fixture(source, f"episode_{number:06d}"),
                 skill=skill, task=f"{task}{number}")
    output = tmp_path / "dataset"
    exporter.convert(args(source, output, arm_mode="BIMANUAL", input_format="can-vector",
                        skills=[skill]))
    manifest = json.loads((output / "conversion_manifest.json").read_text(encoding="utf-8"))
    assert len(manifest["episodes"]) == 2
    assert all(item["skill_id"] == skill for item in manifest["episodes"])
    assert {item["split"] for item in manifest["episodes"]} == {"train", "test"}
    for split in ("train", "test"):
        dataset = LeRobotDataset(f"local/piper_atomic_{split}", root=output / split)
        assert dataset.meta.total_episodes == 1 and len(dataset) == 2
        item = dataset[0]
        assert item["action"][[0, 7]].tolist() == [1.0, 3.0]
        assert item["observation.state"][[0, 7]].tolist() == [0.0, 2.0]
        assert tuple(item["observation.images.right_wrist_rgb"].shape) == (3, 480, 640)
