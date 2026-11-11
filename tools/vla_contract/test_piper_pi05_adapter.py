"""运行：python3 -m unittest discover -s tools/vla_contract -p 'test_piper_pi05_adapter.py' -v。"""

import json
import io
from pathlib import Path
import sys
import tempfile
import unittest
from contextlib import redirect_stderr
from unittest import mock

import numpy as np

from piper_pi05_adapter import PiperInputs, PiperOutputs
from piper_pi05_openpi import check_dataset, digest, main


class PiperAdapterTest(unittest.TestCase):
    def test_v3_training_requires_reader_environment(self):
        argv = ["piper_pi05_openpi.py", "--openpi-root", "openpi", "--base-checkpoint", "checkpoint",
                "--dataset", "dataset", "--evidence", "evidence.json", "--audit-report", "audit.json",
                "--work-dir", "work", "--train"]
        output = io.StringIO()
        with mock.patch.object(sys, "argv", argv), \
                mock.patch("piper_pi05_openpi.check_dataset", return_value={"arm_mode": "LEFT"}), \
                mock.patch("piper_pi05_openpi.check_openpi", return_value="fixed-commit"), \
                redirect_stderr(output):
            self.assertEqual(main(), 1)
        self.assertIn("--lerobot-v3-site-packages", output.getvalue())

    def test_v3_reader_preflight_rejects_missing_dataset(self):
        from piper_pi05_openpi import verify_openpi_v3_reader
        with tempfile.TemporaryDirectory() as temporary:
            with self.assertRaises(FileNotFoundError):
                verify_openpi_v3_reader(Path(temporary), Path(temporary), "LEFT")

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
            from test_piper_normalization_gate import audited_dataset
            proof, _ = audited_dataset(root)
            audit_path, proof_path = root / "audit.json", root / "evidence.json"
            self.assertEqual(check_dataset(root, proof_path, audit_path)["counts"]["train_banana_in_box"], 10)
            audit = json.loads(audit_path.read_text(encoding="utf-8"))
            audit["sources"][0]["suggested_split"] = "test"
            audit_path.write_text(json.dumps(audit), encoding="utf-8")
            proof["source_audit_sha256"] = digest(audit_path)
            proof_path.write_text(json.dumps(proof), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "审计切分"):
                check_dataset(root, proof_path, audit_path)
            audit["sources"][0]["suggested_split"] = "train"
            audit_path.write_text(json.dumps(audit), encoding="utf-8")
            proof["source_audit_sha256"] = digest(audit_path)
            proof["gripper_mapping_verified"] = False
            proof_path.write_text(json.dumps(proof), encoding="utf-8")
            with self.assertRaises(ValueError):
                check_dataset(root, proof_path, audit_path)


if __name__ == "__main__":
    unittest.main()
