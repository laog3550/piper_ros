"""运行：/path/to/openpi/.venv/bin/python -m unittest discover -s tools/vla_contract -p 'test_atomic_skill_sft.py'。"""

import io
import json
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path
from unittest import mock

import pyarrow as pa
import pyarrow.parquet as pq
from atomic_skill_sft import SKILLS, STATE_UNIT, TIMESTAMP_TOPICS, expected_features, inspect, main
from piper_pi05_openpi import digest, episode_source_digest


class AtomicSkillGateTest(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)

    def fixture(self, skill="pick_banana", varied_tasks=False, same_group=False):
        mode = SKILLS[skill]["modes"][0]
        arms = ("left", "right") if mode == "BIMANUAL" else ("left",)
        dim = len(arms) * 7
        dataset = self.root / skill
        dataset.mkdir()
        episodes, labels, stamps = [], {}, []
        for split in ("train", "test"):
            episode_id = f"{skill}-{split}"
            group_id = "scene-shared" if same_group else f"scene-{split}"
            task_text = f"{SKILLS[skill]['prompt']}（{split}）" if varied_tasks else SKILLS[skill]["prompt"]
            completion = {"source_topic": TIMESTAMP_TOPICS["top_rgb"],
                          "header_stamp_ns": 1_800_000_000_000_000_000 + 15 * 100_000_000 - 5_000_000}
            source = self.root / "sources" / episode_id
            (source / "bag").mkdir(parents=True)
            (source / "episode.json").write_text(json.dumps({
                "schema_version": 2, "state": "saved", "mode": "skill", "episode_id": episode_id,
                "skill": skill, "outcome": "success", "task": task_text,
                "frame_topics": [TIMESTAMP_TOPICS["top_rgb"]],
                "annotations": [{"kind": "outcome", "value": "success"},
                                {"kind": "skill_complete", "value": skill, **completion}],
                "task_metadata": {"arm_mode": mode, "time_domain": "unix_ns",
                                  "initial_conditions": {"perturbation_id": group_id},
                                  "action_space": "absolute_joint_position_7",
                                  "action_provenance": "human_demonstration",
                                  "calibration_version": "cal-v1", "normalization_version": "norm-v1"}}))
            (source / "bag" / "metadata.yaml").write_text("synthetic")
            (source / "bag" / "sample.db3").write_bytes(b"synthetic")
            source_hash = episode_source_digest(source)
            episodes.append({"episode_id": episode_id, "split": split, "frames": 16,
                             "source": str(source), "source_sha256": source_hash,
                             "skill_id": skill, "task_text": task_text, "group_id": group_id,
                             "annotation_quality": {"completion_frame": completion},
                             "calibration_version": "cal-v1", "normalization_version": "norm-v1"})
            labels[episode_id] = {"skill_id": skill, "success": True, "verified": True,
                                  "atomic_episode": True, "action_provenance": "human_demonstration",
                                  "source_sha256": source_hash, "calibration_version": "cal-v1",
                                  "normalization_version": "norm-v1",
                                  "calibration_verified": True, "units_verified": True,
                                  "gripper_mapping_verified": True, "gripper_mapping_version": "grip-v1"}
            if mode == "BIMANUAL":
                labels[episode_id]["roles"] = {"left": "hold_box", "right": "close_lid"}
                labels[episode_id]["left_right_sync_verified"] = True
            root = dataset / split
            (root / "meta").mkdir(parents=True)
            (root / "data").mkdir()
            info = {"codebase_version": "v3.0", "robot_type": "piper", "fps": 10,
                    "total_episodes": 1, "total_frames": 16,
                    "features": {name: {"dtype": "float32" if name in ("action", "observation.state")
                                                  else "image", "shape": shape}
                                 for name, shape in expected_features(mode).items()}}
            (root / "meta" / "info.json").write_text(json.dumps(info))
            pq.write_table(pa.table({"task_index": [0], "task": [task_text]}),
                           root / "meta" / "tasks.parquet")
            data = {"episode_index": [0] * 16, "frame_index": list(range(16)),
                    "timestamp": [i / 10 for i in range(16)], "task_index": [0] * 16,
                    "observation.state": [[0.0] * dim for _ in range(16)],
                    "action": [[0.1] * dim for _ in range(16)]}
            for name in set(expected_features(mode)) - {"action", "observation.state"}:
                data[name] = [b"rgb"] * 16
            pq.write_table(pa.table(data), root / "data" / "part.parquet")
            for index in range(16):
                reference = 1_800_000_000_000_000_000 + index * 100_000_000
                header = {TIMESTAMP_TOPICS["top_rgb"]: reference - 5_000_000}
                for arm in arms:
                    header[TIMESTAMP_TOPICS[f"{arm}_wrist_rgb"]] = reference - 5_000_000
                    header[f"/joint_{arm}"] = reference - 3_000_000
                    header[f"/demonstration/action_{arm}"] = reference
                stamps.append({"split": split, "episode_id": episode_id,
                               "frame_index": index, "header_timestamp_ns": header,
                               "source_can_stamps_ns": {}})
        manifest = {"format": "LeRobotDataset v3.0", "lerobot_version": "0.4.4",
                    "purpose": "success_sft", "arm_mode": mode, "input_format": "joint-state", "fps": 10,
                    "action_space": "absolute_joint_position_7", "state_unit": STATE_UNIT,
                    "joint_names": [f"{arm}.{joint}" for arm in arms
                                    for joint in [*(f"joint{i}" for i in range(1, 7)), "gripper"]],
                    "episodes": episodes}
        (dataset / "conversion_manifest.json").write_text(json.dumps(manifest))
        (dataset / "source_timestamps.jsonl").write_text(
            "".join(json.dumps(row) + "\n" for row in stamps))
        audit_path = self.root / "audit.json"
        audit_path.write_text(json.dumps({"schema_version": "piper_real_data_audit_v2",
                                          "sources": [{"episode_id": row["episode_id"], "eligible": True,
                                                       "kind": "native_episode", "source": row["source"],
                                                       "skill_id": skill, "task_text": row["task_text"],
                                                       "group_id": row["group_id"],
                                                       "has_action_target": True, "has_feedback": True,
                                                       "has_rgb": True, "header_time_domain_declared": True}
                                                      for row in episodes]}))
        annotations_path = self.root / "annotations.json"
        annotations_path.write_text(json.dumps({
            "schema_version": "piper_atomic_skill_labels/1",
            "conversion_manifest_sha256": digest(dataset / "conversion_manifest.json"),
            "source_audit_sha256": digest(audit_path), "episodes": labels,
        }))
        return dataset, annotations_path, audit_path

    def check(self, files, skill):
        return inspect(*files, skill, max_sync_ns=10_000_000, max_age_ns=20_000_000,
                       min_train=1, min_test=1)

    def test_single_and_bimanual_plan_never_ready_to_launch(self):
        for skill in SKILLS:
            with self.subTest(skill=skill):
                files = self.fixture(skill)
                result = self.check(files, skill)
                self.assertEqual(result["counts"], {"train": 1, "test": 1})
                self.assertEqual(result["model"]["piper_action_dim"], 14 if "bimanual" in skill else 7)
                self.assertEqual(result["status"], "data_gate_passed_launch_blocked")
                self.assertTrue(result["launch_blockers"])

    def test_v2_source_with_distinct_task_texts_reaches_gate(self):
        files = self.fixture(varied_tasks=True)
        result = self.check(files, "pick_banana")
        self.assertEqual(result["model"]["task_texts"], ["抓取香蕉（test）", "抓取香蕉（train）"])
        self.assertIsNone(result["model"]["prompt"])
        source = self.root / "sources" / "pick_banana-test" / "episode.json"
        meta = json.loads(source.read_text())
        meta["annotations"][1]["value"] = "place_banana_in_box"
        source.write_text(json.dumps(meta))
        with self.assertRaisesRegex(ValueError, "摘要不匹配"):
            self.check(files, "pick_banana")

    def test_reject_perturbation_group_leak(self):
        files = self.fixture(same_group=True)
        with self.assertRaisesRegex(ValueError, "场景组跨越"):
            self.check(files, "pick_banana")

    def test_reject_false_success_and_corrupt_action(self):
        files = self.fixture()
        annotations = json.loads(files[1].read_text())
        annotations["episodes"]["pick_banana-test"]["success"] = False
        files[1].write_text(json.dumps(annotations))
        with self.assertRaisesRegex(ValueError, "成功"):
            self.check(files, "pick_banana")
        annotations["episodes"]["pick_banana-test"]["success"] = True
        files[1].write_text(json.dumps(annotations))
        path = files[0] / "test" / "data" / "part.parquet"
        data = pq.read_table(path).to_pydict()
        data["action"][0] = [0.0] * 6
        pq.write_table(pa.table(data), path)
        with self.assertRaisesRegex(ValueError, "action"):
            self.check(files, "pick_banana")

    def test_reject_bimanual_desync_and_zero_episode(self):
        files = self.fixture("close_hinged_lid_bimanual")
        rows = [json.loads(line) for line in (files[0] / "source_timestamps.jsonl").read_text().splitlines()]
        rows[0]["header_timestamp_ns"]["/demonstration/action_right"] += 50_000_000
        (files[0] / "source_timestamps.jsonl").write_text("".join(json.dumps(row) + "\n" for row in rows))
        with self.assertRaisesRegex(ValueError, "不同步"):
            self.check(files, "close_hinged_lid_bimanual")
        manifest_path = files[0] / "conversion_manifest.json"
        manifest = json.loads(manifest_path.read_text())
        manifest["episodes"] = []
        manifest_path.write_text(json.dumps(manifest))
        with self.assertRaisesRegex(ValueError, "0 个合格"):
            self.check(files, "close_hinged_lid_bimanual")

    def test_reject_unverified_calibration_and_wrong_task(self):
        files = self.fixture("place_banana_in_box")
        labels = json.loads(files[1].read_text())
        labels["episodes"]["place_banana_in_box-train"]["calibration_verified"] = False
        files[1].write_text(json.dumps(labels))
        with self.assertRaisesRegex(ValueError, "夹爪核验"):
            self.check(files, "place_banana_in_box")
        labels["episodes"]["place_banana_in_box-train"]["calibration_verified"] = True
        files[1].write_text(json.dumps(labels))
        task_file = files[0] / "train" / "meta" / "tasks.parquet"
        pq.write_table(pa.table({"task_index": [0], "task": ["香蕉入盒整段任务"]}), task_file)
        with self.assertRaisesRegex(ValueError, "任务文本"):
            self.check(files, "place_banana_in_box")

    def test_launch_remains_closed_after_data_gate(self):
        argv = ["atomic_skill_sft.py", "--dataset", "dataset", "--annotations", "labels.json",
                "--audit-report", "audit.json", "--skill", "pick_banana", "--max-sync-ns", "10000000",
                "--max-source-age-ns", "20000000", "--openpi-root", "openpi",
                "--base-checkpoint", "checkpoint", "--launch"]
        output = io.StringIO()
        with mock.patch.object(sys, "argv", argv), \
                mock.patch("atomic_skill_sft.inspect", return_value={
                    "launch_blockers": ["LeRobot v3.0 与 v2.1 不兼容"]}), \
                mock.patch("atomic_skill_sft.check_openpi"), \
                mock.patch("atomic_skill_sft.verify_base_checkpoint"), redirect_stdout(output):
            self.assertEqual(main(), 2)
        self.assertIn("训练启动已拒绝", output.getvalue())


if __name__ == "__main__":
    unittest.main()
