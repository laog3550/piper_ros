#!/usr/bin/env python3
"""运行轨迹与失败路径验收。运行：python3 -m unittest discover -s tools/vla_contract -p 'test_runtime_trace.py'。"""

import json
from pathlib import Path
import stat
import tempfile
import unittest

from action_supervisor import ActionSupervisor
from long_sequence_planner import LongSequencePlanner, SequenceRunner
from offline_simulator import synthetic_profile
from offline_task_demo import FakeClock, SyntheticCritic, SyntheticPolicy, SyntheticWorld
from runtime_trace import TraceError, TraceSession, read_trace
from task_orchestrator import TaskOrchestrator
from test_long_sequence_planner import context, FixtureBackend, FixtureWorldReader, four_steps


class RuntimeTraceTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.clock = FakeClock()
        self.context = context(self.clock)

    def trace(self, run_id="trace-1", source_kind="synthetic", recovery_id=None, **kwargs):
        links = {"episode_id": "episode-test", "task_group_id": "group-test",
                 "plan_id": "plan-test"}
        if recovery_id is not None:
            links["recovery_id"] = recovery_id
        return TraceSession(self.root, run_id=run_id, source_kind=source_kind,
                            models={"planner": "fixture-planner", "policy": "synthetic-policy"},
                            links=links, **kwargs)

    def runner(self, trace, critic_result="SUCCESS"):
        world = SyntheticWorld(self.clock)
        reader = FixtureWorldReader(world, self.context["object_state"])
        supervisor = ActionSupervisor(synthetic_profile())
        tasks = []

        def factory(skill, step):
            task = TaskOrchestrator((skill,), "LEFT", world, SyntheticPolicy(),
                                    SyntheticCritic(critic_result if isinstance(critic_result, list)
                                                    else [critic_result, critic_result]),
                                    world, self.clock, supervisor)
            tasks.append(task)
            return task

        runner = SequenceRunner(LongSequencePlanner(FixtureBackend([four_steps()])),
                                factory, reader, trace_hook=trace)
        return runner, world, tasks

    def test_success_records_planning_execution_and_verification(self):
        trace = self.trace()
        runner, world, tasks = self.runner(trace)
        result = runner.run(self.context, self.clock)
        self.assertEqual(result["status"], "DONE")
        manifest, events = read_trace(trace.path)
        self.assertEqual(stat.S_IMODE(trace.path.stat().st_mode), 0o700)
        self.assertEqual(stat.S_IMODE(trace.events.stat().st_mode), 0o600)
        self.assertEqual(stat.S_IMODE(trace.manifest_path.stat().st_mode), 0o600)
        self.assertEqual(manifest["source_kind"], "synthetic")
        self.assertEqual(manifest["terminal_status"], "DONE")
        self.assertFalse(manifest["training_eligible"])
        self.assertFalse(manifest["real_success_verified"])
        self.assertEqual(manifest["links"]["episode_id"], "episode-test")
        types = [event["event_type"] for event in events]
        for name, count in (("plan_request", 1), ("plan_result", 1), ("plan_preflight", 1),
                            ("step_preflight", 4), ("policy_request", 4), ("policy_result", 4),
                            ("preflight_accepted", 4), ("execution_request", 8),
                            ("execution_result", 8), ("verify_result", 4), ("world_verify", 4)):
            self.assertEqual(types.count(name), count, name)
        self.assertEqual(events[-1]["event_type"], "run_terminal")
        self.assertEqual(next(event for event in events if event["event_type"] == "policy_request")
                         ["ids"]["step_id"], "s1")
        self.assertTrue(all(event["source_kind"] == "synthetic" for event in events))
        self.assertEqual(len(world.dispatched), 8)

    def test_human_review_stays_fail_closed_and_is_recorded(self):
        trace = self.trace()
        runner, world, tasks = self.runner(trace, "HUMAN_REVIEW")
        result = runner.run(self.context, self.clock)
        self.assertEqual(result["status"], "HUMAN_REVIEW")
        manifest, events = read_trace(trace.path)
        self.assertEqual(manifest["terminal_status"], "HUMAN_REVIEW")
        self.assertIn("HUMAN_REVIEW", [event["data"]["result"] for event in events
                                      if event["event_type"] == "verify_result"])
        self.assertIn("run_failure", [event["event_type"] for event in events])
        self.assertEqual(tasks[0].supervisor.state, "STOP")
        self.assertLessEqual(len(world.dispatched), 2)

    def test_invalid_model_output_is_preserved_without_execution(self):
        class BadBackend:
            last_raw = "not-json"

            def generate(self, context):
                return self.last_raw

        trace = self.trace()
        runner, world, tasks = self.runner(trace)
        runner.planner = LongSequencePlanner(BadBackend())
        result = runner.run(self.context, self.clock)
        self.assertEqual(result["status"], "HUMAN_REVIEW")
        manifest, events = read_trace(trace.path)
        rejected = next(event for event in events if event["event_type"] == "plan_preflight")
        self.assertFalse(rejected["data"]["accepted"])
        self.assertEqual(rejected["data"]["raw_model_output"], "not-json")
        self.assertEqual(world.dispatched, [])
        self.assertEqual(tasks, [])

    def test_external_recovery_id_is_retained(self):
        recovery_id = "abcdef1234567890abcdef1234567890"
        trace = self.trace(recovery_id=recovery_id)
        runner, _, _ = self.runner(trace)
        self.assertEqual(runner.run(self.context, self.clock)["status"], "DONE")
        manifest, events = read_trace(trace.path)
        self.assertEqual(manifest["links"]["recovery_id"], recovery_id)
        self.assertTrue(all(event["ids"]["recovery_id"] == recovery_id for event in events))

    def test_retry_events_share_a_recovery_id(self):
        trace = self.trace()
        runner, _, _ = self.runner(trace, ["FAIL", "SUCCESS"])
        self.assertEqual(runner.run(self.context, self.clock)["status"], "DONE")
        _, events = read_trace(trace.path)
        requested = next(event for event in events if event["event_type"] == "recovery_requested")
        started = next(event for event in events if event["event_type"] == "recovery_started")
        self.assertEqual(requested["ids"]["recovery_id"], started["ids"]["recovery_id"])
        self.assertTrue(requested["ids"]["recovery_id"].startswith("recovery-"))

    def test_trace_failure_during_execution_stops_before_next_target(self):
        trace = self.trace()
        original = trace.record

        def fail_once(event_type, data, **kwargs):
            if event_type == "execution_result":
                raise TraceError("模拟磁盘故障")
            return original(event_type, data, **kwargs)

        trace.record = fail_once
        runner, world, tasks = self.runner(trace)
        result = runner.run(self.context, self.clock)
        self.assertEqual(result["status"], "HUMAN_REVIEW")
        self.assertEqual(len(world.dispatched), 1)
        self.assertEqual(tasks[0].supervisor.state, "STOP")
        manifest, events = read_trace(trace.path)
        self.assertEqual(manifest["terminal_status"], "HUMAN_REVIEW")
        failure = next(event for event in events if event["event_type"] == "skill_failure")
        self.assertEqual(failure["data"]["failure_category"], "TRACE_FAILURE")

    def test_supervisor_rejection_is_recorded_without_dispatch(self):
        class UnsafePolicy(SyntheticPolicy):
            def plan(self, observation, goal):
                action = super().plan(observation, goal)
                action["targets"]["left"][0][0] = 999.0
                return action

        trace = self.trace()
        runner, world, tasks = self.runner(trace)
        original_factory = runner.task_factory

        def factory(skill, step):
            task = original_factory(skill, step)
            task.policy = UnsafePolicy()
            return task

        runner.task_factory = factory
        result = runner.run(self.context, self.clock)
        self.assertEqual(result["status"], "HUMAN_REVIEW")
        self.assertEqual(world.dispatched, [])
        self.assertEqual(tasks[0].supervisor.state, "STOP")
        _, events = read_trace(trace.path)
        rejected = next(event for event in events if event["event_type"] == "preflight_rejected")
        self.assertEqual(rejected["data"]["failure_category"], "SUPERVISOR_REJECTED")

    def test_restart_and_corruption_are_rejected(self):
        trace = self.trace()
        trace.record("plan_request", {"input": "synthetic"}, ids={"observation_id": "观测:1"})
        with self.assertRaises(TraceError):
            read_trace(trace.path)
        manifest, events = read_trace(trace.path, allow_incomplete=True)
        self.assertEqual((manifest["status"], len(events)), ("incomplete", 1))
        self.assertEqual(events[0]["ids"]["observation_id"], "观测:1")
        with self.assertRaises(FileExistsError):
            self.trace()
        with trace.events.open("ab") as stream:
            stream.write(b'{"partial":')
        with self.assertRaises(TraceError):
            read_trace(trace.path, allow_incomplete=True)

    def test_real_source_requires_digest_and_never_claims_success(self):
        with self.assertRaises(TraceError):
            self.trace(source_kind="real")
        trace = self.trace(run_id="trace-real", source_kind="real",
                           episode_manifest_sha256="a" * 64)
        trace.finish("DONE")
        manifest, events = read_trace(trace.path)
        self.assertEqual(manifest["source_kind"], "real")
        self.assertFalse(manifest["real_success_verified"])
        self.assertFalse(manifest["training_eligible"])
        manifest["training_eligible"] = True
        trace.manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
        with self.assertRaises(TraceError):
            read_trace(trace.path)


if __name__ == "__main__":
    unittest.main()
