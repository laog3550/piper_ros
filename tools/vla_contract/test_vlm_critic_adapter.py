"""合成评估结果的编排状态转移测试。"""

import tempfile
import unittest
from pathlib import Path

from PIL import Image

from offline_task_demo import make_task
from task_orchestrator import Skill
from vlm_critic_adapter import OfflineVlmCriticAdapter
from tools.vlm_critic.critic import verdict


class OfflineVlmCriticAdapterTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        for name in ("before.png", "after.png"):
            Image.new("RGB", (640, 480), "white").save(self.root / name)

    def make_task(self, evaluator):
        task, world = make_task(skills=(Skill("pick", "object-1", max_retries=2),
                                         Skill("place", "region-1")))
        original = world.observe

        def observe(skill, mode, now_ns):
            obs, feedback = original(skill, mode, now_ns)
            obs["images"]["top_rgb"]["uri"] = "after.png"
            return obs, feedback

        world.observe = observe

        def sample_builder(skill, obs):
            return {
                "sample_id": obs["observation_id"],
                "skill_id": skill.name,
                "arm_mode": obs["arm_mode"],
                "target_id": skill.target,
                "calibration_id": obs["calibration_version"],
                "pre_rgb": "before.png",
                "post_rgb": "after.png",
                "geometry": {"pre_in_gripper": None, "post_in_gripper": None,
                             "post_in_region": None, "post_target_visible": None},
            }

        task.critic = OfflineVlmCriticAdapter(sample_builder, evaluator, self.root)
        return task, world

    def test_all_results_require_review_without_retry_or_next_skill(self):
        cases = (("SUCCESS", "NONE", 1.0),
                 ("FAILURE", "EMPTY_GRASP", 1.0),
                 ("UNKNOWN", "UNKNOWN", 0.0))
        for status, failure, confidence in cases:
            with self.subTest(status=status):
                task, world = self.make_task(
                    lambda sample: verdict(sample, status, failure, confidence))
                board = task.run()
                self.assertEqual(board.phase, "SAFE_STOP")
                self.assertEqual(board.skill_index, 0)
                self.assertEqual(board.attempt, 0)
                self.assertEqual(len(world.dispatched), 2)
                self.assertFalse(any(event[2] == "RECOVER" for event in board.history))
                self.assertEqual(task.supervisor.state, "STOP")
                self.assertTrue(world.cancelled)
                self.assertEqual(task.critic.last_output["result"], status)

    def test_invalid_transition_and_reference_fail_closed(self):
        def invalid_transition(sample):
            result = verdict(sample, "SUCCESS", "NONE", 1.0)
            result["recommended_transition"] = "DONE"
            return result

        for evaluator in (invalid_transition,
                          lambda sample: {**verdict(sample), "sample_id": "other"}):
            with self.subTest(evaluator=evaluator):
                task, world = self.make_task(evaluator)
                board = task.run()
                self.assertEqual(board.phase, "SAFE_STOP")
                self.assertEqual(board.skill_index, 0)
                self.assertEqual(len(world.dispatched), 2)
                self.assertIsNone(task.critic.last_output)

    def test_sample_mismatch_skips_evaluator(self):
        calls = []
        task, world = self.make_task(lambda sample: calls.append(sample))
        task.critic.sample_builder = lambda skill, obs: {
            "sample_id": obs["observation_id"], "skill_id": skill.name,
            "arm_mode": obs["arm_mode"], "target_id": "other",
            "calibration_id": obs["calibration_version"],
            "pre_rgb": "before.png", "post_rgb": "after.png",
            "geometry": {"pre_in_gripper": None, "post_in_gripper": None,
                         "post_in_region": None, "post_target_visible": None},
        }
        board = task.run()
        self.assertEqual(board.phase, "SAFE_STOP")
        self.assertEqual(calls, [])
        self.assertEqual(len(world.dispatched), 2)


if __name__ == "__main__":
    unittest.main()
