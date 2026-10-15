#!/usr/bin/env python3
"""离线安全监督回归测试。运行：python3 -m unittest discover -s tools/vla_contract -p 'test_*.py'。"""

from dataclasses import replace
import unittest

from action_supervisor import SafetyError
from offline_simulator import DT, make_case, run_scenarios


class ActionSupervisorTests(unittest.TestCase):
    def test_scenarios(self):
        self.assertEqual(set(run_scenarios()), {"正常动作", "陈旧动作", "反馈掉线", "双臂共享区冲突", "单侧故障联动与复位"})

    def test_missing_parameters_fail_closed(self):
        supervisor, data, now = make_case()
        supervisor.profile = replace(supervisor.profile, swept_collision_free=None)
        with self.assertRaises(SafetyError):
            supervisor.submit(data, now)
        self.assertEqual(supervisor.state, "HOLD")

    def test_contract_and_units(self):
        for mutation in (
            lambda d: d["action_chunk"].__setitem__("contract_version", "other"),
            lambda d: d["action_chunk"].__setitem__("action_space", "joint_velocity_7"),
            lambda d: d["action_chunk"]["targets"]["left"][0].pop(),
            lambda d: d["action_chunk"].__setitem__("observation_timestamp_ns", 1),
        ):
            supervisor, data, now = make_case()
            mutation(data)
            with self.assertRaises(SafetyError):
                supervisor.submit(data, now)
            self.assertEqual(supervisor.state, "HOLD")

    def test_old_source_inside_fresh_snapshot(self):
        supervisor, data, now = make_case()
        data["observation"]["images"]["top_rgb"]["header_timestamp_ns"] -= 100_000_000
        with self.assertRaisesRegex(SafetyError, "源数据过期"):
            supervisor.submit(data, now)

    def test_position_velocity_acceleration(self):
        cases = ((2.0, "位置"), (0.5, "速度"), (0.05, "加速度"))
        for value, reason in cases:
            supervisor, data, now = make_case()
            data["action_chunk"]["targets"]["left"] = [[value] + [0.0] * 5 + [0.04]] * 2
            if reason == "速度":
                supervisor.profile = replace(supervisor.profile, limits={"left": replace(
                    supervisor.profile.limits["left"], max_velocity=(0.1,) * 7)})
            if reason == "加速度":
                supervisor.profile = replace(supervisor.profile, limits={"left": replace(
                    supervisor.profile.limits["left"], max_acceleration=(1.0,) * 7)})
            with self.assertRaisesRegex(SafetyError, reason):
                supervisor.submit(data, now)

    def test_duration_and_timeout(self):
        supervisor, data, now = make_case()
        data["skill_goal"]["timeout_sec"] = 0.1
        with self.assertRaisesRegex(SafetyError, "时限"):
            supervisor.submit(data, now)
        supervisor, data, now = make_case()
        supervisor.submit(data, now)
        self.assertIsNone(supervisor.tick(now + 1_100_000_000))
        self.assertEqual(supervisor.state, "STOP")

    def test_scheduler_fault_and_recovery(self):
        supervisor, data, now = make_case()
        supervisor.submit(data, now)
        supervisor.update_feedback("left", [0.0] * 6 + [0.04], [0.0] * 7, now + DT)
        self.assertIsNone(supervisor.tick(now + DT + 20_000_000))
        self.assertEqual(supervisor.state, "STOP")
        with self.assertRaises(SafetyError):
            supervisor.submit(data, now)
        supervisor.update_feedback("left", [0.0] * 6 + [0.04], [0.0] * 7, now + DT + 21_000_000)
        supervisor.reset(now + DT + 21_000_000)
        self.assertEqual(supervisor.state, "HOLD")

    def test_collision_unknown_and_callback_failure(self):
        for checker in (lambda paths: None, lambda paths: False, lambda paths: 1 / 0):
            supervisor, data, now = make_case()
            supervisor.profile = replace(supervisor.profile, swept_collision_free=checker)
            with self.assertRaises(SafetyError):
                supervisor.submit(data, now)

    def test_one_arm_fault_stops_both(self):
        supervisor, data, now = make_case("BIMANUAL")
        supervisor.submit(data, now)
        supervisor.update_feedback("right", [0.0] * 6 + [0.04], [0.0] * 7, now + 1, healthy=False)
        self.assertEqual(supervisor.state, "STOP")
        self.assertIsNone(supervisor.tick(now + DT))

    def test_live_feedback_limit(self):
        supervisor, data, now = make_case()
        supervisor.submit(data, now)
        supervisor.update_feedback("left", [2.0] + [0.0] * 5 + [0.04], [0.0] * 7, now + 1)
        self.assertEqual(supervisor.state, "STOP")

    def test_live_collision_and_final_ack(self):
        supervisor, data, now = make_case()
        supervisor.submit(data, now)
        supervisor.profile = replace(supervisor.profile, swept_collision_free=lambda paths: False)
        supervisor.update_feedback("left", [0.0] * 6 + [0.04], [0.0] * 7, now + 1)
        self.assertEqual(supervisor.state, "STOP")

        supervisor, data, now = make_case()
        supervisor.submit(data, now)
        self.assertIsNotNone(supervisor.tick(now + DT))
        supervisor.update_feedback("left", data["action_chunk"]["targets"]["left"][0],
                                   [0.1] + [0.0] * 6, now + DT + 1)
        self.assertIsNotNone(supervisor.tick(now + 2 * DT))
        self.assertEqual(supervisor.state, "RUNNING")
        self.assertIsNone(supervisor.tick(now + 2 * DT + 1))
        self.assertEqual(supervisor.state, "RUNNING")


if __name__ == "__main__":
    unittest.main()
