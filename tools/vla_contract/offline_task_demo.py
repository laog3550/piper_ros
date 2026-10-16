#!/usr/bin/env python3
"""合成四步 pick/place 离线验收。运行：python3 tools/vla_contract/offline_task_demo.py；绝不连接 ROS/CAN。"""

from copy import deepcopy

from offline_simulator import synthetic_profile
from action_supervisor import ActionSupervisor
from task_orchestrator import Skill, TaskOrchestrator
from validate_contract import synthetic


class FakeClock:
    def __init__(self):
        self.value = 1_800_000_000_100_000_000

    def now_ns(self):
        return self.value

    def sleep_until(self, when):
        self.value = max(self.value, when)


class SyntheticWorld:
    def __init__(self, clock, mode="LEFT"):
        self.clock, self.mode = clock, mode
        self.position = [0.0] * 6 + [0.04]
        self.count = 0
        self.cancelled = False
        self.dispatched = []
        self.delay_ns = 1_000_000

    def observe(self, skill, mode, now_ns):
        self.clock.value += 1_000_000
        self.count += 1
        obs = synthetic(mode, skill.name)["observation"]
        obs["observation_id"] = f"synthetic-{self.count}"
        obs["observation_timestamp_ns"] = self.clock.now_ns()
        for frame in obs["images"].values():
            frame["header_timestamp_ns"] = self.clock.now_ns() - 2_000_000
        for state in obs["state"].values():
            state["header_timestamp_ns"] = self.clock.now_ns() - 1_000_000
            state["position"] = self.position.copy()
        return obs, {mode.lower(): (self.position.copy(), [0.0] * 7, self.clock.now_ns(), True)}

    def dispatch(self, approved, issue_ns, deadline_ns):
        self.dispatched.append(deepcopy(approved))
        self.clock.value += self.delay_ns
        self.position = list(approved[self.mode.lower()])
        return self.position.copy(), [0.0] * 7, self.clock.now_ns(), True

    def cancel(self):
        self.cancelled = True


class SyntheticPolicy:
    def plan(self, obs, goal):
        action = synthetic(obs["arm_mode"], goal["skill_id"])["action_chunk"]
        action["observation_id"] = obs["observation_id"]
        action["observation_timestamp_ns"] = obs["observation_timestamp_ns"]
        action["inference_timestamp_ns"] = obs["observation_timestamp_ns"]
        arm = obs["arm_mode"].lower()
        first = obs["state"][arm]["position"].copy()
        first[0] += 0.01
        action["targets"][arm] = [first.copy(), first.copy()]
        return action


class SyntheticCritic:
    def __init__(self, results=()):
        self.results = list(results)

    def verify(self, skill, observation):
        return self.results.pop(0) if self.results else "SUCCESS"


def make_task(results=(), skills=None, collision=None, mode="LEFT"):
    clock = FakeClock()
    world = SyntheticWorld(clock, mode)
    profile = synthetic_profile(mode, collision=collision)
    supervisor = ActionSupervisor(profile)
    skills = skills or (Skill("pick", "object-1"), Skill("place", "region-1"),
                        Skill("pick", "object-2"), Skill("place", "region-2"))
    task = TaskOrchestrator(skills, mode, world, SyntheticPolicy(), SyntheticCritic(results),
                            world, clock, supervisor)
    return task, world


if __name__ == "__main__":
    task, world = make_task()
    board = task.run()
    assert board.phase == "DONE" and board.skill_index == 4 and len(world.dispatched) == 8
    print("合成四步 pick/place：DONE；8 个目标均经安全监督批准。非实机验收。")
