"""影子入口的合成整链和拒绝路径。"""

from copy import deepcopy
from pathlib import Path
import tempfile
import unittest

from PIL import Image

from shadow_run import inspect_snapshot, run_fixture
from validate_contract import synthetic


def record():
    obs = synthetic()["observation"]
    return {"contract_observation_valid": True, "observation": obs,
            "sources": {role: {field: {"valid": True} for field in
                               ("rgb", "depth", "camera_info", "roi_depth", "world_geometry")}
                        for role in ("top", "left_wrist")}}


class ShadowRunTests(unittest.TestCase):
    def test_fixture_reaches_review_after_supervision(self):
        result = run_fixture()
        self.assertEqual(result["phase"], "SAFE_STOP")
        self.assertEqual(result["supervisor_state"], "STOP")
        self.assertEqual(result["critic_transition"], "HUMAN_REVIEW")
        self.assertEqual(result["preflight_count"], 1)
        self.assertEqual(len(result["proposals"]), 2)
        self.assertTrue(result["sink_cancelled"])

    def test_missing_state_rejected_before_policy(self):
        item = record()
        item["contract_observation_valid"] = False
        item["observation"] = None
        item["rejection_reasons"] = ["left.state:missing"]
        output = inspect_snapshot(item)
        self.assertEqual(output[0]["stage"], "OBSERVE")
        self.assertEqual(output[0]["rejection_reasons"], ["left.state:missing"])

    def test_stale_rejected_before_calibration(self):
        item = record()
        timestamp = item["observation"]["observation_timestamp_ns"]
        output = inspect_snapshot(item, now_ns=timestamp + 200_000_000)
        self.assertEqual(len(output), 1)
        self.assertIn("过期", output[0]["reason"])

    def test_calibration_and_mapping_fail_closed(self):
        item = record()
        timestamp = item["observation"]["observation_timestamp_ns"]
        output = inspect_snapshot(item, now_ns=timestamp)
        self.assertEqual(output[-1]["stage"], "CALIBRATION")
        self.assertIn("gripper_mapping_unverified", output[-1]["rejection_reasons"])
        self.assertIn("tf_or_world_geometry_unverified", output[-1]["rejection_reasons"])
        self.assertTrue(all(part["status"] != "PROPOSED" for part in output))

    def test_verified_claim_still_rejects_missing_policy_input(self):
        item = record()
        item.update(gripper_mapping_verified=True, calibration_verified=True,
                    tf_verified=True, world_geometry_valid=True)
        timestamp = item["observation"]["observation_timestamp_ns"]
        output = inspect_snapshot(item, now_ns=timestamp)
        self.assertEqual(output[-1]["stage"], "POLICY_INPUT")
        self.assertEqual(output[-1]["status"], "REJECTED")

    def test_source_skew_rejected(self):
        item = deepcopy(record())
        timestamp = item["observation"]["observation_timestamp_ns"]
        item["observation"]["state"]["left"]["header_timestamp_ns"] -= 100_000_000
        output = inspect_snapshot(item, now_ns=timestamp)
        self.assertEqual(output[0]["stage"], "OBSERVE")
        self.assertIn("偏差", output[0]["reason"])

    def test_droid_action_space_never_becomes_piper_target(self):
        item = record()
        item.update(gripper_mapping_verified=True, calibration_verified=True,
                    tf_verified=True, world_geometry_valid=True,
                    skill_goal=synthetic()["skill_goal"])
        with tempfile.TemporaryDirectory() as temporary:
            for name, frame in item["observation"]["images"].items():
                frame["uri"] = f"{name}.png"
                Image.new("RGB", (640, 480)).save(Path(temporary) / frame["uri"])
            timestamp = item["observation"]["observation_timestamp_ns"]
            output = inspect_snapshot(item, image_dir=temporary, now_ns=timestamp)
        self.assertEqual(output[-1]["stage"], "PLAN")
        self.assertEqual(output[-1]["reason"], "model_action_space_mismatch")
        self.assertTrue(all(part["status"] != "PROPOSED" for part in output))


if __name__ == "__main__":
    unittest.main()
