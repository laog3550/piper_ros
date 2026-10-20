#!/usr/bin/env python3
"""公开 VLM 的合成慢规划刷新验收。运行：python3 tools/vla_contract/verify_refreshed_vlm_runner.py --model /本地模型 --output /tmp/结果.json。"""

import argparse
from copy import deepcopy
import json
import math
from pathlib import Path
import tempfile

from action_supervisor import ActionSupervisor
from eval_local_vlm_planner import cases, make_case
from local_vlm_planner import LocalVlmBackend
from long_sequence_planner import LongSequencePlanner, SequenceRunner, ground_context
from offline_simulator import synthetic_profile
from offline_task_demo import FakeClock, SyntheticCritic, SyntheticPolicy, SyntheticWorld
from task_orchestrator import TaskOrchestrator


def main():
    parser = argparse.ArgumentParser(description="只用合成世界验证公开 VLM 的慢规划刷新；不连接机械臂")
    parser.add_argument("--model", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--timeout", type=float, default=120)
    args = parser.parse_args()
    if not args.model.is_dir() or not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("模型目录和正数超时必需")
    with tempfile.TemporaryDirectory(prefix="piper-vlm-refresh-") as directory:
        root = Path(directory)
        case_id, instruction, count, held, expected = cases()[0]
        request = make_case(root, case_id, instruction, count, held)
        request["scene_version"] = "synthetic-scene-v1"
        grounded = ground_context(request)
        clock = FakeClock()
        clock.value = request["observation"]["observation_timestamp_ns"]
        world = SyntheticWorld(clock)
        backend = LocalVlmBackend(args.model, root, args.timeout)

        class SlowBackend:
            def generate(self, context):
                raw = backend.generate(context)
                clock.value += int(backend.last_seconds * 1_000_000_000)
                return raw

        class Reader:
            def __init__(self):
                self.state = deepcopy(grounded["object_state"])
                self.refreshes = 0

            def refresh_plan_snapshot(self):
                self.refreshes += 1
                obs, _ = world.observe(type("S", (), {"name": "pick"})(), "LEFT", clock.now_ns())
                for frame in obs["images"].values():
                    frame["uri"] = f"synthetic://{frame['frame_id']}/{obs['observation_id']}"
                return {"trusted": True, "scene_version": request["scene_version"], "observation": obs,
                        "object_ids": deepcopy(grounded["object_ids"]),
                        "region_ids": deepcopy(grounded["region_ids"]),
                        "object_state": deepcopy(self.state), "goal_state": deepcopy(grounded["goal_state"])}

            def __call__(self, step):
                self.state[step["object_id"]] = (
                    {"status": "held", "region_id": None} if step["skill"] == "pick"
                    else {"status": "placed", "region_id": step["region_id"]})
                return deepcopy(self.state), False

        reader = Reader()
        supervisor = ActionSupervisor(synthetic_profile())

        def factory(skill, step):
            return TaskOrchestrator((skill,), "LEFT", world, SyntheticPolicy(),
                                    SyntheticCritic(["SUCCESS", "SUCCESS"]), world, clock, supervisor)

        runner = SequenceRunner(LongSequencePlanner(SlowBackend(), max_age_ns=5_000_000_000),
                                factory, reader)
        result = runner.run(request, clock)
        try:
            actual_steps = [(step["skill"], step["object_id"], step["region_id"])
                            for step in json.loads(backend.last_raw)["steps"]]
        except (TypeError, ValueError, KeyError):
            actual_steps = None
        report = {"data_kind": "synthetic_only", "model_dir": str(args.model.resolve()),
                  "result": result, "model_seconds": backend.last_seconds,
                  "raw_model_output": backend.last_raw, "refreshes": reader.refreshes,
                  "world_dispatched": len(world.dispatched), "expected_steps": expected,
                  "actual_steps": actual_steps, "semantic_correct": actual_steps == expected}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({key: value for key, value in report.items()
                      if key not in ("raw_model_output", "expected_steps")}, ensure_ascii=False))
    if (result["status"] != "DONE" or result["completed"] != ["s1", "s2", "s3", "s4"]
            or not report["semantic_correct"]):
        raise SystemExit(2)


if __name__ == "__main__":
    main()
