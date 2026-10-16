#!/usr/bin/env python3
"""离线编排回归测试。运行：python3 -m unittest discover -s tools/vla_contract -p 'test_*.py'。"""

import unittest

from offline_task_demo import make_task
from task_orchestrator import Skill, SnapshotObserverAdapter


class TaskOrchestratorTests(unittest.TestCase):
    def test_four_step_sequence(self):
        task, world = make_task()
        board = task.run()
        self.assertEqual((board.phase, board.skill_index), ("DONE", 4))
        self.assertEqual(len(world.dispatched), 8)
        self.assertFalse(world.cancelled)
        self.assertEqual(sum(event[2] == "VERIFY" for event in board.history), 4)

    def test_right_arm_independent_sequence(self):
        task, world = make_task(mode="RIGHT", skills=(Skill("pick", "object-1"), Skill("place", "region-1")))
        board = task.run()
        self.assertEqual((board.phase, board.skill_index), ("DONE", 2))
        self.assertTrue(all(set(target) == {"right"} for target in world.dispatched))

    def test_snapshot_adapter_rejects_invalid_record(self):
        class Builder:
            def build(self, mode, anchor, now):
                return {"contract_observation_valid": False, "rejection_reasons": ["missing_frame"]}

        with self.assertRaisesRegex(ValueError, "missing_frame"):
            SnapshotObserverAdapter(Builder(), None).observe(Skill("pick", "x"), "LEFT", 1)

    def test_failed_verification_reobserves_and_recovers(self):
        task, world = make_task(results=("FAIL", "SUCCESS"), skills=(Skill("pick", "object-1"),))
        board = task.run()
        self.assertEqual(board.phase, "DONE")
        self.assertEqual(len(world.dispatched), 4)
        self.assertEqual(sum(event[2] == "RECOVER" for event in board.history), 1)
        self.assertGreaterEqual(world.count, 4)

    def test_retry_exhaustion_stops(self):
        task, world = make_task(results=("FAIL", "FAIL"), skills=(Skill("pick", "object-1"),))
        board = task.run()
        self.assertEqual(board.phase, "SAFE_STOP")
        self.assertTrue(world.cancelled)
        self.assertEqual(task.supervisor.state, "STOP")

    def test_timeout_cancels(self):
        task, world = make_task(skills=(Skill("pick", "object-1", timeout_sec=0.3),))
        world.delay_ns = 250_000_000
        board = task.run()
        self.assertEqual(board.phase, "SAFE_STOP")
        self.assertIn("超时", board.reason)
        self.assertTrue(world.cancelled)

    def test_rejected_action_never_dispatched(self):
        task, world = make_task(collision=lambda paths: False)
        board = task.run()
        self.assertEqual(board.phase, "SAFE_STOP")
        self.assertEqual(world.dispatched, [])

    def test_unknown_critic_stops(self):
        task, world = make_task(results=("UNKNOWN",))
        board = task.run()
        self.assertEqual(board.phase, "SAFE_STOP")
        self.assertEqual(len(world.dispatched), 2)

    def test_human_takeover(self):
        task, world = make_task()
        task.request_takeover()
        board = task.run()
        self.assertEqual(board.phase, "HUMAN_TAKEOVER")
        self.assertEqual(world.dispatched, [])
        self.assertTrue(world.cancelled)

    def test_verify_feedback_stop_skips_critic(self):
        task, world = make_task(skills=(Skill("pick", "object-1"),))
        original = world.observe
        calls = []

        def failing_observation(skill, mode, now_ns):
            obs, feedback = original(skill, mode, now_ns)
            if world.count == 2:
                arm = mode.lower()
                pos, vel, stamp, _ = feedback[arm]
                feedback[arm] = (pos, vel, stamp, False)
            return obs, feedback

        world.observe = failing_observation
        task.critic.verify = lambda skill, obs: calls.append(True)
        board = task.run()
        self.assertEqual(board.phase, "SAFE_STOP")
        self.assertEqual(calls, [])

    def test_invalid_verify_observation_skips_critic(self):
        task, world = make_task(skills=(Skill("pick", "object-1"),))
        original = world.observe
        calls = []

        def invalid_observation(skill, mode, now_ns):
            obs, feedback = original(skill, mode, now_ns)
            if world.count == 2:
                obs["images"].pop("top_rgb")
            return obs, feedback

        world.observe = invalid_observation
        task.critic.verify = lambda skill, obs: calls.append(True)
        board = task.run()
        self.assertEqual(board.phase, "SAFE_STOP")
        self.assertEqual(calls, [])

    def test_cancel_failure_keeps_stop_latched(self):
        task, world = make_task(collision=lambda paths: False)

        def broken_cancel():
            raise RuntimeError("合成取消故障")

        world.cancel = broken_cancel
        board = task.run()
        self.assertEqual(board.phase, "SAFE_STOP")
        self.assertEqual(task.supervisor.state, "STOP")
        self.assertIn("取消执行器失败", board.reason)


if __name__ == "__main__":
    unittest.main()
