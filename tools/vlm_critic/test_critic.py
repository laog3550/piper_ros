"""运行：python3 -m unittest tools.vlm_critic.test_critic。仅用合成图像验证链路。"""

import copy
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
import subprocess

from PIL import Image

from tools.vlm_critic.critic import evaluate, parse_model, validate_input, validate_output


class CriticTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        base = Path(self.tmp.name)
        for name in ("pre.png", "post.png"):
            Image.new("RGB", (16, 16), "white").save(base / name)
        self.sample = validate_input({
            "sample_id": "synthetic-1", "skill_id": "pick", "arm_mode": "LEFT", "target_id": "object-1", "calibration_id": "synthetic-calib",
            "pre_rgb": "pre.png", "post_rgb": "post.png",
            "geometry": {"pre_in_gripper": False, "post_in_gripper": True,
                         "post_in_region": None, "post_target_visible": None},
        }, base)

    def tearDown(self):
        self.tmp.cleanup()

    def test_geometry_pick_and_place(self):
        pick = evaluate(self.sample, None, 1)
        self.assertEqual((pick["result"], pick["failure_type"]), ("SUCCESS", "NONE"))
        self.assertEqual(pick["recommended_transition"], "HUMAN_REVIEW")
        place = copy.deepcopy(self.sample)
        place["skill_id"] = "place"
        place["geometry"].update(post_in_gripper=True, post_in_region=False)
        result = evaluate(place, None, 1)
        self.assertEqual((result["result"], result["failure_type"]), ("FAILURE", "PLACE_MISS"))

    def test_missing_model_is_unknown(self):
        self.sample["geometry"] = {key: None for key in self.sample["geometry"]}
        result = evaluate(self.sample, "/no/such/model", 0.1)
        self.assertEqual((result["result"], result["confidence"], result["recommended_transition"], result["backend"]),
                         ("UNKNOWN", 0.0, "HUMAN_REVIEW", "vlm_zero_shot"))

    def test_model_timeout_and_bad_output_are_unknown(self):
        self.sample["geometry"] = {key: None for key in self.sample["geometry"]}
        with patch("tools.vlm_critic.critic.subprocess.run", side_effect=subprocess.TimeoutExpired("model", 1)):
            result = evaluate(self.sample, self.tmp.name, 1)
        self.assertEqual(result["result"], "UNKNOWN")
        with patch("tools.vlm_critic.critic.subprocess.run", return_value=subprocess.CompletedProcess([], 0, "not JSON", "")):
            result = evaluate(self.sample, self.tmp.name, 1)
        self.assertEqual(result["result"], "UNKNOWN")

    def test_low_confidence_is_unknown(self):
        self.sample["geometry"] = {key: None for key in self.sample["geometry"]}
        raw = json.dumps({"result": "SUCCESS", "failure_type": "NONE", "confidence": 0.4,
                          "evidence": [{"source": "post_rgb", "observation": "目标在夹爪附近"}]})
        with patch("tools.vlm_critic.critic.subprocess.run", return_value=subprocess.CompletedProcess([], 0, raw, "")):
            result = evaluate(self.sample, self.tmp.name, 1, min_confidence=0.7)
        self.assertEqual((result["result"], result["recommended_transition"]), ("UNKNOWN", "HUMAN_REVIEW"))

    def test_reject_invalid_model_response(self):
        raw = json.dumps({"result": "SUCCESS", "failure_type": "NONE", "confidence": 0.8,
                          "evidence": [{"source": "system", "observation": "凭空判定"}]})
        with self.assertRaises(ValueError):
            parse_model(raw, self.sample)
        result = evaluate(self.sample, None, 1)
        result["failure_type"] = "PLACE_MISS"
        with self.assertRaises(ValueError):
            validate_output(result, self.sample)

    def test_invalid_geometry_and_image(self):
        invalid = copy.deepcopy(self.sample)
        invalid["geometry"]["post_in_gripper"] = "yes"
        with self.assertRaises(ValueError):
            validate_input(invalid, Path(self.tmp.name))
        invalid = copy.deepcopy(self.sample)
        invalid["post_rgb"] = "missing.png"
        with self.assertRaises(ValueError):
            validate_input(invalid, Path(self.tmp.name))


if __name__ == "__main__":
    unittest.main()
