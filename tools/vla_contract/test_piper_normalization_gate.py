"""用合成文件核验原始版本、train 统计与 checkpoint 的独立证据链。"""

import copy
from contextlib import ExitStack, redirect_stderr, redirect_stdout
import io
import json
import os
from pathlib import Path
import shutil
import sys
import tempfile
import types
import unittest
from unittest import mock

from piper_pi05_openpi import (
    ASSET_ID, CHECKPOINT_NORMALIZATION, OPENPI_COMMIT, bind_checkpoint_normalization,
    check_dataset, check_work_dir, checkpoint_hashes, dataset_files_digest, digest, episode_source_digest,
    load_validated_policy, main, normalization_stats, verify_checkpoint_normalization,
    write_normalization_provenance,
)


def write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False), encoding="utf-8")


def sample_stats():
    return {"norm_stats": {key: {"mean": [0.1] * 7, "std": [0.2] * 7,
                                "q01": [0.0] * 7, "q99": [0.3] * 7}
                           for key in ("state", "actions")}}


def audited_dataset(root):
    """Synthetic structural fixture; it does not contain real robot or LeRobot data."""
    for split in ("train", "test"):
        (root / split / "meta").mkdir(parents=True)
        write_json(root / split / "meta" / "info.json", {"codebase_version": "v3.0"})
    episodes, labels = [], {}
    for index in range(13):
        identifier = f"banana-{index}"
        source = root / "sources" / identifier
        (source / "bag").mkdir(parents=True)
        skill, group_id, task_text = "pick_and_place_banana_in_box", f"scene-{index}", f"合成示教 {index}"
        write_json(source / "episode.json", {
            "schema_version": 2, "mode": "skill", "state": "saved", "outcome": "success",
            "episode_id": identifier, "skill": skill, "task": task_text,
            "task_metadata": {"arm_mode": "LEFT", "time_domain": "unix_ns",
                "action_space": "absolute_joint_position_7", "action_provenance": "human_demonstration",
                "calibration_version": "verified-camera-v1", "data_convention_version": "raw-units-v1",
                "training_eligibility": {"eligible": False, "status": "pending_independent_evidence_and_audit"},
                "initial_conditions": {"perturbation_id": group_id}}})
        (source / "bag" / "metadata.yaml").write_text("合成占位文件", encoding="utf-8")
        (source / "bag" / "sample.db3").write_bytes(b"synthetic fixture")
        episodes.append({"episode_id": identifier, "source_episode_id": identifier,
            "split": "train" if index < 10 else "test", "frames": 16, "source": str(source),
            "source_sha256": episode_source_digest(source), "skill_id": skill,
            "group_id": group_id, "task_text": task_text, "calibration_version": "verified-camera-v1",
            "data_convention_version": "raw-units-v1", "normalization_version": None})
        labels[identifier] = "banana_in_box"
    manifest_path, audit_path, proof_path = (root / name for name in (
        "conversion_manifest.json", "audit.json", "evidence.json"))
    write_json(manifest_path, {"format": "LeRobotDataset v3.0", "purpose": "success_sft",
        "arm_mode": "LEFT", "action_space": "absolute_joint_position_7",
        "joint_names": [f"left.joint{i}" for i in range(1, 7)] + ["left.gripper"], "episodes": episodes})
    write_json(audit_path, {"schema_version": "piper_real_data_audit_v2", "split_ready": True,
        "eligible_episode_count": len(episodes), "sources": [
            {"episode_id": row["episode_id"], "eligible": True, "kind": "native_episode",
             "source": row["source"], "skill_id": row["skill_id"], "task_text": row["task_text"],
             "group_id": row["group_id"], "suggested_split": row["split"]} for row in episodes]})
    proof = {"status": "verified_for_piper_pi05_source_data", "arm_mode": "LEFT",
        "action_space": "absolute_joint_position_7", "action_provenance": "human_demonstration",
        "gripper_mapping_verified": True, "calibration_verified": True,
        "calibration_version": "verified-camera-v1", "data_convention_version": "raw-units-v1",
        "conversion_manifest_sha256": digest(manifest_path), "source_audit_sha256": digest(audit_path),
        "episode_skills": labels}
    write_json(proof_path, proof)
    evidence = check_dataset(root, proof_path, audit_path, require_normalization=False)
    asset, provenance = root / "norm_stats.json", root / "normalization_provenance.json"
    write_json(asset, sample_stats())
    generated = write_normalization_provenance(asset, evidence, "train-statistics-v1", provenance)
    proof.update({key: value for key, value in generated.items() if key != "status"})
    proof["status"] = "verified_for_piper_pi05_training"
    proof["normalization_verified"] = True
    write_json(proof_path, proof)
    return proof, episodes


class NormalizationGateTest(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix="piper-norm-gate-")
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.proof, self.episodes = audited_dataset(self.root)
        self.proof_path, self.audit_path = self.root / "evidence.json", self.root / "audit.json"

    def check(self, proof=None, **kwargs):
        write_json(self.proof_path, proof if proof is not None else self.proof)
        return check_dataset(self.root, self.proof_path, self.audit_path, **kwargs)

    def test_new_sources_need_no_training_version_and_remain_unchanged(self):
        paths = [Path(row["source"]) / "episode.json" for row in self.episodes]
        before = [digest(path) for path in paths]
        result = self.check()
        self.assertEqual(result["data_convention_version"], "raw-units-v1")
        self.assertEqual(result["normalization_version"], "train-statistics-v1")
        self.assertEqual(len(result["train_episodes"]), 10)
        self.assertEqual(result["counts"]["train_banana_in_box"], 10)
        self.assertEqual(before, [digest(path) for path in paths])
        for path in paths:
            self.assertNotIn("normalization_version", json.loads(path.read_text())["task_metadata"])
        test_ids = {row["episode_id"] for row in self.episodes if row["split"] == "test"}
        self.assertFalse(test_ids & {row["episode_id"] for row in result["train_episodes"]})

    def test_sources_without_statistics_pass_only_preparation_gate(self):
        proof = {key: value for key, value in self.proof.items() if not key.startswith("normalization")}
        proof["status"] = "verified_for_piper_pi05_source_data"
        self.assertEqual(self.check(proof, require_normalization=False)["counts"]["train_banana_in_box"], 10)
        with self.assertRaisesRegex(ValueError, "独立 Piper 训练核验证据"):
            self.check(proof)
        proof["status"] = "verified_for_piper_pi05_training"
        with self.assertRaisesRegex(ValueError, "normalization_version"):
            self.check(proof)

    def test_missing_unreviewed_or_raw_statistics_are_rejected(self):
        mutations = ({"normalization_version": "raw-units-v1"}, {"normalization_verified": False},
                     {"normalization": None}, {"normalization_provenance": None},
                     {"normalization": {**self.proof["normalization"], "sha256": "0" * 64}})
        for mutation in mutations:
            with self.subTest(mutation=mutation), self.assertRaises(ValueError):
                self.check({**self.proof, **mutation})
        path = self.root / "norm_stats.json"
        write_json(path, {"version_id": "raw-units-v1", "time_domain": "unix_ns", "vector": {}})
        proof = copy.deepcopy(self.proof)
        proof["normalization"]["sha256"] = digest(path)
        with self.assertRaisesRegex(ValueError, "原始数据约定"):
            self.check(proof)

    def test_invalid_statistics_rejected(self):
        path = self.root / "norm_stats.json"
        for key, value in (("std", [-1] * 7), ("mean", [float("nan")] * 7),
                           ("q99", [-1] * 7), ("q01", [0] * 6), ("mean", ["0"] * 7)):
            data = sample_stats()
            data["norm_stats"]["state"][key] = value
            write_json(path, data)
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                normalization_stats(path)

    def test_provenance_rejects_test_leakage_and_stale_bindings(self):
        path = self.root / "normalization_provenance.json"
        original = json.loads(path.read_text())
        mutations = ({"source_split": "test"}, {"train_episodes": original["train_episodes"][:-1]},
                     {"train_episodes": original["train_episodes"] + [{"episode_id": "banana-12"}]},
                     {"source_audit_sha256": "0" * 64}, {"conversion_manifest_sha256": "0" * 64},
                     {"train_dataset_sha256": "0" * 64}, {"data_convention_version": "other"},
                     {"normalization_sha256": "0" * 64}, {"openpi_commit": "other"})
        for mutation in mutations:
            write_json(path, {**original, **mutation})
            proof = copy.deepcopy(self.proof)
            proof["normalization_provenance"]["sha256"] = digest(path)
            with self.subTest(mutation=mutation), self.assertRaisesRegex(ValueError, "已审计的 train"):
                self.check(proof)

    def test_changed_train_files_invalidate_statistics(self):
        train_hash = dataset_files_digest(self.root / "train")
        (self.root / "test" / "held_out.txt").write_text("不得用于统计", encoding="utf-8")
        self.assertEqual(dataset_files_digest(self.root / "train"), train_hash)
        (self.root / "train" / "changed.txt").write_text("改变训练输入", encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "已审计的 train"):
            self.check()

    def test_ineligible_source_cannot_prepare_statistics(self):
        audit = json.loads(self.audit_path.read_text())
        audit["sources"][0]["eligible"] = False
        write_json(self.audit_path, audit)
        proof = {**self.proof, "source_audit_sha256": digest(self.audit_path)}
        with self.assertRaisesRegex(ValueError, "来源审计"):
            self.check(proof, require_normalization=False)

    def test_raw_version_mismatch_is_rejected(self):
        manifest_path = self.root / "conversion_manifest.json"
        manifest = json.loads(manifest_path.read_text())
        manifest["episodes"][0]["data_convention_version"] = "other-raw-v1"
        write_json(manifest_path, manifest)
        proof = {**self.proof, "conversion_manifest_sha256": digest(manifest_path)}
        with self.assertRaisesRegex(ValueError, "来源与转换清单"):
            self.check(proof)

    def test_exported_segments_are_checked_against_original_sources(self):
        manifest_path = self.root / "conversion_manifest.json"
        manifest = json.loads(manifest_path.read_text())
        segment = copy.deepcopy(manifest["episodes"][0])
        segment["episode_id"] += "__segment_001"
        manifest["episodes"].append(segment)
        write_json(manifest_path, manifest)
        proof = copy.deepcopy(self.proof)
        proof["conversion_manifest_sha256"] = digest(manifest_path)
        proof["episode_skills"][segment["episode_id"]] = "banana_in_box"
        source_evidence = self.check(proof, require_normalization=False)
        self.assertEqual(source_evidence["counts"]["train_banana_in_box"], 10)
        self.assertEqual(len(source_evidence["train_episodes"]), 11)
        self.assertEqual(source_evidence["train_episodes"][1]["source_episode_id"], "banana-0")

    def test_segments_cannot_inflate_minimum_independent_demonstrations(self):
        manifest_path = self.root / "conversion_manifest.json"
        manifest = json.loads(manifest_path.read_text())
        manifest["episodes"][9] = {**manifest["episodes"][0], "episode_id": "banana-0__segment_001"}
        write_json(manifest_path, manifest)
        proof = copy.deepcopy(self.proof)
        proof["conversion_manifest_sha256"] = digest(manifest_path)
        del proof["episode_skills"]["banana-9"]
        proof["episode_skills"]["banana-0__segment_001"] = "banana_in_box"
        with self.assertRaisesRegex(ValueError, "至少需要 10"):
            self.check(proof, require_normalization=False)

    def test_checkpoint_must_contain_the_approved_statistics(self):
        evidence = self.check()
        checkpoint = self.root / "checkpoint"
        asset = checkpoint / CHECKPOINT_NORMALIZATION
        asset.parent.mkdir(parents=True)
        shutil.copyfile(self.root / "norm_stats.json", asset)
        # OpenPI may reserialize the same numerical statistics when saving its checkpoint.
        asset.write_text(json.dumps(sample_stats(), indent=2), encoding="utf-8")
        self.assertNotEqual(digest(asset), evidence["normalization"]["sha256"])
        binding = bind_checkpoint_normalization(checkpoint, evidence)
        record = {**evidence, **binding}
        verify_checkpoint_normalization(checkpoint, record)
        with self.assertRaisesRegex(ValueError, "独立统计版本"):
            verify_checkpoint_normalization(checkpoint, {**record, "normalization_version": "raw-units-v1"})
        changed = sample_stats()
        changed["norm_stats"]["state"]["mean"][0] += 1
        write_json(asset, changed)
        with self.assertRaisesRegex(ValueError, "核准资产"):
            bind_checkpoint_normalization(checkpoint, evidence)
        with self.assertRaisesRegex(ValueError, "SHA-256"):
            verify_checkpoint_normalization(checkpoint, record)

    def test_work_dir_cannot_write_into_sources(self):
        for work in (self.root / "train" / "work", Path(self.episodes[0]["source"]) / "work"):
            with self.subTest(work=work), self.assertRaisesRegex(ValueError, "来源只读"):
                check_work_dir(work, self.root)
            self.assertFalse(work.exists())

    def test_policy_load_rejects_legacy_record_without_normalization_binding(self):
        checkpoint = self.root / "checkpoint"
        checkpoint.mkdir()
        (checkpoint / "weights").write_bytes(b"synthetic")
        record_path, evaluation_path = self.root / "record.json", self.root / "evaluation.json"
        write_json(record_path, {"status": "trained_unvalidated", "checkpoint_sha256": checkpoint_hashes(checkpoint)})
        write_json(evaluation_path, {"status": "piper_offline_validated", "training_record_sha256": digest(record_path)})
        with self.assertRaisesRegex(ValueError, "checkpoint 绑定"):
            load_validated_policy(self.root, checkpoint, record_path, evaluation_path)

    def run_stage(self, prepare):
        temporary = tempfile.TemporaryDirectory(prefix="piper-norm-stage-")
        self.addCleanup(temporary.cleanup)
        work = Path(temporary.name) / "work"
        cfg = types.SimpleNamespace(assets_dirs=work / "assets" / "config", checkpoint_dir=work / "checkpoints")
        dataset_module = types.ModuleType("lerobot.datasets.lerobot_dataset")
        dataset_module.HF_LEROBOT_HOME = self.root / "previous-home"
        datasets_module = types.ModuleType("lerobot.datasets")
        datasets_module.lerobot_dataset = dataset_module
        lerobot_module = types.ModuleType("lerobot")
        lerobot_module.datasets = datasets_module
        argv = ["piper_pi05_openpi.py", "--openpi-root", str(self.root / "openpi"),
                "--base-checkpoint", str(self.root / "base"), "--dataset", str(self.root),
                "--evidence", str(self.proof_path), "--audit-report", str(self.audit_path),
                "--work-dir", str(work), "--lerobot-v3-site-packages", str(self.root / "site-packages")]
        argv += (["--prepare-normalization", "--normalization-version", "prepared-statistics-v2"]
                 if prepare else ["--train"])
        if prepare:
            source_proof = {key: value for key, value in self.proof.items() if not key.startswith("normalization")}
            source_proof["status"] = "verified_for_piper_pi05_source_data"
            write_json(self.proof_path, source_proof)
        calls = []

        def fake_script(root, script, config):
            calls.append(script)
            self.assertEqual((work / "lerobot" / ASSET_ID).resolve(), (self.root / "train").resolve())
            if prepare:
                asset = cfg.assets_dirs / ASSET_ID / "norm_stats.json"
                asset.parent.mkdir(parents=True)
                write_json(asset, sample_stats())
            else:
                asset = cfg.checkpoint_dir / "1" / CHECKPOINT_NORMALIZATION
                asset.parent.mkdir(parents=True)
                shutil.copyfile(cfg.assets_dirs / ASSET_ID / "norm_stats.json", asset)
                (cfg.checkpoint_dir / "1" / "weights").write_bytes(b"synthetic checkpoint")

        with ExitStack() as stack:
            stack.enter_context(mock.patch.dict(sys.modules, {"lerobot": lerobot_module,
                "lerobot.datasets": datasets_module, "lerobot.datasets.lerobot_dataset": dataset_module}))
            stack.enter_context(mock.patch.dict(os.environ))
            stack.enter_context(mock.patch.object(sys, "argv", argv))
            for name in ("check_openpi", "install_lerobot_v3_compat", "verify_openpi_v3_reader", "verify_base_checkpoint"):
                stack.enter_context(mock.patch(f"piper_pi05_openpi.{name}", return_value=OPENPI_COMMIT))
            config_mock = stack.enter_context(mock.patch("piper_pi05_openpi.build_config", return_value=cfg))
            stack.enter_context(mock.patch("piper_pi05_openpi.run_openpi_script", side_effect=fake_script))
            output = stack.enter_context(redirect_stdout(io.StringIO()))
            errors = stack.enter_context(redirect_stderr(io.StringIO()))
            code = main()
        self.assertEqual(code, 0, errors.getvalue())
        self.assertEqual(config_mock.call_args.args[4], 1 if prepare else 8)
        return work, calls, json.loads(output.getvalue())

    def test_prepare_only_reads_train_and_does_not_grant_training(self):
        work, calls, result = self.run_stage(prepare=True)
        self.assertEqual(calls, ["compute_norm_stats.py"])
        self.assertEqual(result["status"], "normalization_computed_pending_independent_review")
        self.assertFalse((work / "training_record.json").exists())
        provenance = json.loads((work / "normalization_provenance.json").read_text())
        self.assertEqual(provenance["source_split"], "train")
        self.assertEqual(len(provenance["train_episodes"]), 10)

    def test_training_consumes_reviewed_asset_without_recomputing(self):
        work, calls, record = self.run_stage(prepare=False)
        self.assertEqual(calls, ["train.py"])
        self.assertEqual(record["normalization_version"], "train-statistics-v1")
        self.assertEqual(record["checkpoint_normalization_sha256"], self.proof["normalization"]["sha256"])
        self.assertEqual(record["status"], "trained_unvalidated")
        self.assertTrue((work / "training_record.json").is_file())
        verify_checkpoint_normalization(Path(record["checkpoint_dir"]), record)


if __name__ == "__main__":
    unittest.main()
