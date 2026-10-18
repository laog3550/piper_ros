"""运行：python3 -m unittest discover -s tools/vla_contract -p 'test_piper_pi05_adapter.py' -v。"""

import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from piper_pi05_adapter import PiperInputs, PiperOutputs
from piper_pi05_openpi import check_dataset, digest, episode_source_digest


class PiperAdapterTest(unittest.TestCase):
    def test_left_right_and_absolute_action(self):
        image = np.zeros((3, 480, 640), dtype=np.float32)
        actions = np.full((16, 7), 0.1, dtype=np.float32)
        for arm in ("left", "right"):
            with self.subTest(arm=arm):
                result = PiperInputs(arm)({"images": {"top_rgb": image, "wrist_rgb": image},
                                           "state": np.zeros(7), "actions": actions,
                                           "prompt": "pick"})
                self.assertTrue(result["image_mask"][f"{arm}_wrist_0_rgb"])
                np.testing.assert_array_equal(result["actions"], actions)
        output = PiperOutputs()({"actions": np.full((16, 32), 0.1)})
        np.testing.assert_array_equal(output["actions"], actions)

    def test_reject_wrong_shape_nonfinite_negative_gripper(self):
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        base = {"images": {"top_rgb": image, "wrist_rgb": image},
                "state": np.zeros(7), "prompt": "pick"}
        for state in (np.zeros(8), np.array([0] * 6 + [float("nan")]),
                      np.array([0] * 6 + [-0.01])):
            with self.assertRaises(ValueError):
                PiperInputs()(dict(base, state=state))
        with self.assertRaises(ValueError):
            PiperOutputs()({"actions": np.zeros((15, 32))})

    def test_real_data_gate_rejects_missing_manifest(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / "evidence.json").write_text("{}", encoding="utf-8")
            (root / "audit.json").write_text("{}", encoding="utf-8")
            with self.assertRaises(FileNotFoundError):
                check_dataset(root, root / "evidence.json", root / "audit.json")

    def test_training_gate_requires_independent_provenance(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            for split in ("train", "test"):
                (root / split / "meta").mkdir(parents=True)
                (root / split / "meta" / "info.json").write_text("{}", encoding="utf-8")
            episodes = []
            labels = {}
            for skill in ("pick", "place"):
                for index in range(13):
                    identifier = f"{skill}-{index}"
                    labels[identifier] = skill
                    source = root / "sources" / identifier
                    (source / "bag").mkdir(parents=True)
                    (source / "episode.json").write_text("{}", encoding="utf-8")
                    (source / "bag" / "metadata.yaml").write_text("x", encoding="utf-8")
                    (source / "bag" / "sample.db3").write_bytes(b"test")
                    episodes.append({"episode_id": identifier, "split": "train" if index < 10 else "test",
                                     "frames": 16, "source": str(source),
                                     "source_sha256": episode_source_digest(source),
                                     "calibration_version": "verified-v1", "normalization_version": "piper-v1"})
            manifest = {"format": "LeRobotDataset v3.0", "arm_mode": "LEFT",
                        "action_space": "absolute_joint_position_7",
                        "joint_names": [f"left.joint{i}" for i in range(1, 7)] + ["left.gripper"],
                        "episodes": episodes}
            manifest_path = root / "conversion_manifest.json"
            manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
            audit_path = root / "audit.json"
            audit_path.write_text(json.dumps({"schema_version": "piper_real_data_audit_v1",
                                              "eligible_episode_count": len(episodes),
                                              "sources": [{"episode_id": row["episode_id"], "eligible": True}
                                                          for row in episodes]}), encoding="utf-8")
            proof = {"status": "verified_for_piper_pi05_training", "arm_mode": "LEFT",
                     "action_space": "absolute_joint_position_7",
                     "conversion_manifest_sha256": digest(manifest_path),
                     "source_audit_sha256": digest(audit_path),
                     "action_provenance": "human_demonstration", "gripper_mapping_verified": True,
                     "calibration_verified": True, "calibration_version": "verified-v1",
                     "normalization_version": "piper-v1", "episode_skills": labels}
            proof_path = root / "evidence.json"
            proof_path.write_text(json.dumps(proof), encoding="utf-8")
            self.assertEqual(check_dataset(root, proof_path, audit_path)["counts"]["train_pick"], 10)
            proof["gripper_mapping_verified"] = False
            proof_path.write_text(json.dumps(proof), encoding="utf-8")
            with self.assertRaises(ValueError):
                check_dataset(root, proof_path, audit_path)


if __name__ == "__main__":
    unittest.main()
