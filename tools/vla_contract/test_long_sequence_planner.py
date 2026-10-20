#!/usr/bin/env python3
"""长序列规划与合成执行验收。运行：python3 -m unittest discover -s tools/vla_contract -p 'test_long_sequence_planner.py'。"""

from copy import deepcopy
import json
from pathlib import Path
import tempfile
import unittest

from PIL import Image

from action_supervisor import ActionSupervisor
from long_sequence_planner import LongSequencePlanner, PlanError, SequenceRunner, VERSION, ground_context, parse_plan
from local_vlm_planner import LocalVlmBackend
from offline_simulator import synthetic_profile
from offline_task_demo import FakeClock, SyntheticCritic, SyntheticPolicy, SyntheticWorld
from task_orchestrator import TaskOrchestrator
from validate_contract import synthetic
from vlm_critic_adapter import OfflineVlmCriticAdapter
from tools.vlm_critic.critic import verdict


def context(clock):
    obs = synthetic()["observation"]
    obs["observation_timestamp_ns"] = clock.now_ns()
    return {"request_id": "request-1", "instruction": "依次把 object-1 放入 region-1、object-2 放入 region-2",
            "observation": obs, "object_ids": ["object-1", "object-2"],
            "region_ids": ["region-1", "region-2"], "available_skills": ["pick", "place"],
            "object_state": {name: {"status": "available", "region_id": None}
                             for name in ("object-1", "object-2")},
            "goal_state": {"object-1": "region-1", "object-2": "region-2"},
            "completed_step_ids": []}


def step(sid, skill, obj, region=None, dependency=None):
    return {"step_id": sid, "skill": skill, "arm": "LEFT", "object_id": obj,
            "region_id": region, "depends_on": [] if dependency is None else [dependency],
            "completion": {"type": "object_held" if skill == "pick" else "object_in_region",
                           "object_id": obj, "region_id": region}}


def plan(ctx, steps):
    return {"schema_version": VERSION, "request_id": ctx["request_id"],
            "observation_id": ctx["observation"]["observation_id"], "steps": steps}


def four_steps():
    return [step("s1", "pick", "object-1"), step("s2", "place", "object-1", "region-1", "s1"),
            step("s3", "pick", "object-2", dependency="s2"),
            step("s4", "place", "object-2", "region-2", "s3")]


def scene_context(explicit):
    request = {key: deepcopy(value) for key, value in explicit.items()
               if key not in ("object_ids", "region_ids", "goal_state")}
    request["instruction"] = "把红杯放到左盒，再把蓝杯放到右盒"
    request["scene_objects"] = [{"id": "object-1", "aliases": ["红杯"]},
                                {"id": "object-2", "aliases": ["蓝杯"]}]
    request["scene_regions"] = [{"id": "region-1", "aliases": ["左盒"]},
                                {"id": "region-2", "aliases": ["右盒"]}]
    return request


class FixtureBackend:
    def __init__(self, sequences):
        self.sequences = list(sequences)

    def generate(self, ctx):
        return json.dumps(plan(ctx, self.sequences.pop(0)))


class FixtureWorldReader:
    def __init__(self, world, initial_state, replan_at=0, mismatch=False):
        self.world = world
        self.state = deepcopy(initial_state)
        self.calls = 0
        self.replan_at = replan_at
        self.mismatch = mismatch

    def __call__(self, current_step):
        self.calls += 1
        obj = current_step["object_id"]
        if not self.mismatch:
            self.state[obj] = ({"status": "held", "region_id": None} if current_step["skill"] == "pick"
                               else {"status": "placed", "region_id": current_step["region_id"]})
        return deepcopy(self.state), self.calls == self.replan_at

    def refresh_observation(self):
        obs, _ = self.world.observe(type("S", (), {"name": "pick"})(), "LEFT", self.world.clock.now_ns())
        return obs


class PlannerTests(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.ctx = context(self.clock)

    def test_valid_four_step_plan(self):
        self.assertEqual(len(parse_plan(json.dumps(plan(self.ctx, four_steps())), self.ctx)["steps"]), 4)

    def test_chinese_task_grounding_and_end_to_end(self):
        scene = scene_context(self.ctx)
        grounded = ground_context(scene)
        self.assertEqual(grounded["goal_state"], {"object-1": "region-1", "object-2": "region-2"})
        self.assertIn("object-1→region-1", grounded["instruction"])
        arrow = scene_context(self.ctx)
        arrow["instruction"] = "红杯→左盒，蓝杯→右盒"
        self.assertEqual(ground_context(arrow)["goal_state"], grounded["goal_state"])
        runner, _, _ = self.make_runner([four_steps()])
        result = runner.run(scene, self.clock)
        self.assertEqual((result["status"], result["completed"]),
                         ("DONE", ["s1", "s2", "s3", "s4"]))
        with self.assertRaisesRegex(PlanError, "未覆盖"):
            parse_plan(plan(grounded, four_steps()[:2]), grounded)
        wrong = four_steps()
        wrong[1]["region_id"] = "region-2"
        wrong[1]["completion"]["region_id"] = "region-2"
        with self.assertRaisesRegex(PlanError, "矛盾"):
            parse_plan(plan(grounded, wrong), grounded)

    def test_chinese_task_ambiguity_and_unmatched_clause(self):
        scene = scene_context(self.ctx)
        scene["scene_objects"][1]["aliases"].append("红杯")
        with self.assertRaisesRegex(PlanError, "消歧"):
            ground_context(scene)
        scene = scene_context(self.ctx)
        scene["instruction"] = "把红杯放到左盒，再把绿杯放到右盒"
        with self.assertRaisesRegex(PlanError, "未匹配"):
            ground_context(scene)
        scene = scene_context(self.ctx)
        scene["instruction"] += "，然后检查安全"
        with self.assertRaisesRegex(PlanError, "无法完整消歧"):
            ground_context(scene)
        runner, _, _ = self.make_runner([four_steps()])
        self.assertEqual(runner.run(scene, self.clock)["status"], "HUMAN_REVIEW")

    def test_reject_ambiguous_target_and_prefix_collision(self):
        ambiguous = deepcopy(self.ctx)
        ambiguous["instruction"] = "把两个物体依次放入各自区域"
        with self.assertRaisesRegex(PlanError, "消歧"):
            parse_plan(plan(ambiguous, four_steps()), ambiguous)
        missing_region = deepcopy(self.ctx)
        missing_region["instruction"] = "依次拿起 object-1 和 object-2，再放到指定区域"
        with self.assertRaisesRegex(PlanError, "消歧"):
            parse_plan(plan(missing_region, four_steps()), missing_region)
        prefix = deepcopy(self.ctx)
        prefix["object_ids"] = ["object-1", "object-10"]
        prefix["object_state"]["object-10"] = prefix["object_state"].pop("object-2")
        prefix["instruction"] = "把 object-10 放入 region-1，再放入 region-2"
        with self.assertRaisesRegex(PlanError, "消歧"):
            parse_plan(plan(prefix, [step("s1", "pick", "object-1")]), prefix)

    def test_reject_partial_plan_and_missing_goal_binding(self):
        partial = four_steps()[:2]
        with self.assertRaisesRegex(PlanError, "未覆盖"):
            parse_plan(plan(self.ctx, partial), self.ctx)
        missing = deepcopy(self.ctx)
        del missing["goal_state"]
        with self.assertRaisesRegex(PlanError, "字段"):
            LongSequencePlanner(FixtureBackend([four_steps()])).plan(missing, self.clock.now_ns())
        with self.assertRaisesRegex(PlanError, "目标终态"):
            parse_plan(plan(missing, four_steps()), missing)
        extra = deepcopy(self.ctx)
        extra["goal_state"] = {"object-1": "region-1"}
        with self.assertRaisesRegex(PlanError, "目标之外"):
            parse_plan(plan(extra, four_steps()), extra)

    def test_reject_unknown_duplicate_conflict_and_overlong(self):
        mutations = [
            lambda p: p["steps"][0].__setitem__("object_id", "unknown"),
            lambda p: p["steps"][1].__setitem__("step_id", "s1"),
            lambda p: p["steps"][1].__setitem__("region_id", "region-2"),
            lambda p: p["steps"][2].__setitem__("arm", "BIMANUAL"),
            lambda p: p["steps"][2].__setitem__("skill", "move"),
            lambda p: p["steps"][2].__setitem__("depends_on", ["s1"]),
            lambda p: p["steps"].append(deepcopy(p["steps"][3])),
            lambda p: p["steps"].extend([deepcopy(p["steps"][3])] * 2),
            lambda p: p["steps"][0]["completion"].__setitem__("type", "object_in_region"),
            lambda p: p["steps"][2].__setitem__("object_id", "object-1"),
        ]
        for mutate in mutations:
            with self.subTest(mutate=mutate):
                proposal = plan(self.ctx, four_steps())
                mutate(proposal)
                with self.assertRaises(PlanError):
                    parse_plan(proposal, self.ctx)

    def test_bad_json_stale_observation_and_backend_failure(self):
        with self.assertRaises(PlanError):
            parse_plan("not json", self.ctx)
        planner = LongSequencePlanner(FixtureBackend([four_steps()]))
        with self.assertRaisesRegex(PlanError, "观测过期"):
            planner.plan(self.ctx, self.clock.now_ns() + 100_000_000)
        planner.backend.generate = lambda ctx: 1 / 0
        with self.assertRaisesRegex(PlanError, "后端失败"):
            planner.plan(self.ctx, self.clock.now_ns())

    def test_local_model_rejects_image_dimension_mismatch(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            Image.new("RGB", (32, 32), "white").save(root / "top.png")
            local = deepcopy(self.ctx)
            local["observation"]["images"]["top_rgb"]["uri"] = "top.png"
            backend = LocalVlmBackend(root, root)
            with self.assertRaisesRegex(PlanError, "尺寸"):
                backend.generate(local)

    def test_observation_expires_during_model_inference(self):
        class SlowBackend:
            def generate(inner, ctx):
                self.clock.value += 100_000_000
                return json.dumps(plan(ctx, four_steps()))

        runner = SequenceRunner(LongSequencePlanner(SlowBackend()),
                                lambda skill, current_step: self.fail("不应开始执行"), None)
        result = runner.run(self.ctx, self.clock)
        self.assertEqual(result["status"], "REOBSERVE")
        self.assertEqual(result["completed"], [])

    def make_runner(self, plans, critic_results=(), replan_at=0, mismatch=False):
        world = SyntheticWorld(self.clock)
        supervisor = ActionSupervisor(synthetic_profile())
        results = list(critic_results)
        tasks = []

        def factory(skill, current_step):
            result = results.pop(0) if results else "SUCCESS"
            task = TaskOrchestrator((skill,), "LEFT", world, SyntheticPolicy(),
                                    SyntheticCritic(result if isinstance(result, list) else [result, result]),
                                    world, self.clock, supervisor)
            tasks.append(task)
            return task

        reader = FixtureWorldReader(world, self.ctx["object_state"], replan_at, mismatch)
        runner = SequenceRunner(LongSequencePlanner(FixtureBackend(plans)), factory, reader)
        return runner, world, tasks

    def test_end_to_end_four_steps_via_supervisor(self):
        runner, world, tasks = self.make_runner([four_steps()])
        result = runner.run(self.ctx, self.clock)
        self.assertEqual((result["status"], result["completed"]), ("DONE", ["s1", "s2", "s3", "s4"]))
        self.assertEqual(len(world.dispatched), 8)
        self.assertEqual(len(tasks), 4)
        self.assertTrue(all(task.supervisor.state == "HOLD" for task in tasks))

    def test_bounded_replan_after_world_update(self):
        remaining = [step("s3", "pick", "object-2"), step("s4", "place", "object-2", "region-2", "s3")]
        runner, world, _ = self.make_runner([four_steps(), remaining], replan_at=2)
        result = runner.run(self.ctx, self.clock)
        self.assertEqual((result["status"], result["replans"]), ("DONE", 1))
        self.assertEqual(result["completed"], ["s1", "s2", "s3", "s4"])

    def test_single_failure_recovers_then_continues(self):
        runner, world, tasks = self.make_runner([four_steps()], critic_results=[["FAIL", "SUCCESS"]])
        result = runner.run(self.ctx, self.clock)
        self.assertEqual((result["status"], result["completed"]),
                         ("DONE", ["s1", "s2", "s3", "s4"]))
        self.assertEqual(len(world.dispatched), 10)
        self.assertIn((0, 1, "OBSERVE", "重新观测后重试"), tasks[0].board.history)

    def test_failure_and_human_review_do_not_advance(self):
        for critic_result in ("FAIL", "HUMAN_REVIEW"):
            with self.subTest(result=critic_result):
                self.setUp()
                runner, world, tasks = self.make_runner([four_steps()], [critic_result])
                result = runner.run(self.ctx, self.clock)
                self.assertEqual(result["status"], "HUMAN_REVIEW")
                self.assertEqual(result["completed"], [])
                self.assertEqual(len(tasks), 1)
                self.assertEqual(tasks[0].supervisor.state, "STOP")
                self.assertLessEqual(len(world.dispatched), 4)

    def test_world_contradiction_stops(self):
        runner, _, tasks = self.make_runner([four_steps()], mismatch=True)
        result = runner.run(self.ctx, self.clock)
        self.assertEqual(result["status"], "HUMAN_REVIEW")
        self.assertEqual(tasks[0].supervisor.state, "STOP")
        self.assertEqual(result["completed"], [])

    def test_real_critic_adapter_requires_review_before_next_step(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for name in ("before.png", "after.png"):
                Image.new("RGB", (640, 480), "white").save(root / name)
            world = SyntheticWorld(self.clock)
            original = world.observe

            def observe(skill, mode, now_ns):
                obs, feedback = original(skill, mode, now_ns)
                obs["images"]["top_rgb"]["uri"] = "after.png"
                return obs, feedback

            world.observe = observe
            supervisor = ActionSupervisor(synthetic_profile())

            def sample_builder(skill, obs):
                return {"sample_id": obs["observation_id"], "skill_id": skill.name,
                        "arm_mode": obs["arm_mode"], "target_id": skill.target,
                        "calibration_id": obs["calibration_version"],
                        "pre_rgb": "before.png", "post_rgb": "after.png",
                        "geometry": {"pre_in_gripper": None, "post_in_gripper": None,
                                     "post_in_region": None, "post_target_visible": None}}

            def factory(skill, current_step):
                critic = OfflineVlmCriticAdapter(sample_builder,
                                                  lambda sample: verdict(sample, "SUCCESS", "NONE", 1.0), root)
                return TaskOrchestrator((skill,), "LEFT", world, SyntheticPolicy(), critic,
                                        world, self.clock, supervisor)

            runner = SequenceRunner(LongSequencePlanner(FixtureBackend([four_steps()])), factory,
                                    FixtureWorldReader(world, self.ctx["object_state"]))
            result = runner.run(self.ctx, self.clock)
            self.assertEqual(result["status"], "HUMAN_REVIEW")
            self.assertEqual(result["completed"], [])
            self.assertEqual(supervisor.state, "STOP")
            self.assertEqual(len(world.dispatched), 2)


if __name__ == "__main__":
    unittest.main()
