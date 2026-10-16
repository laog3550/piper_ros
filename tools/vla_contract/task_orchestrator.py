#!/usr/bin/env python3
"""纯离线 pick/place 任务编排。运行：python3 tools/vla_contract/task_orchestrator.py。仅接收注入的观测、策略、评估与合成执行器。"""

from dataclasses import dataclass, field
import math

from action_supervisor import ActionSupervisor, SafetyError
from validate_contract import VERSION, validate_observation


@dataclass(frozen=True)
class Skill:
    name: str
    target: str
    timeout_sec: float = 2.0
    max_retries: int = 1

    def __post_init__(self):
        if (self.name not in ("pick", "place") or not isinstance(self.target, str) or not self.target
                or type(self.timeout_sec) not in (int, float) or not math.isfinite(self.timeout_sec)
                or self.timeout_sec <= 0 or type(self.max_retries) is not int or self.max_retries < 0):
            raise ValueError("仅支持具有有效目标、时限和重试次数的 pick/place")


@dataclass
class Blackboard:
    arm_mode: str
    skills: tuple
    phase: str = "OBSERVE"
    skill_index: int = 0
    attempt: int = 0
    observation: dict | None = None
    goal: dict | None = None
    action: dict | None = None
    attempt_deadline_ns: int = 0
    reason: str = ""
    history: list = field(default_factory=list)


class SnapshotObserverAdapter:
    """对接只读 SnapshotBuilder.build；反馈由独立来源提供。"""

    def __init__(self, builder, feedback_provider):
        self.builder, self.feedback_provider = builder, feedback_provider

    def observe(self, skill, mode, now_ns):
        record = self.builder.build(mode, now_ns, now_ns)
        if not record["contract_observation_valid"]:
            raise ValueError(f"快照无效：{record['rejection_reasons']}")
        arm = mode.lower()
        return record["observation"], {arm: self.feedback_provider.read(arm, now_ns)}


class TaskOrchestrator:
    """依赖全部由调用方注入；目标发送路径只接受监督器 tick 的返回值。"""

    def __init__(self, skills, arm_mode, observer, policy, critic, sink, clock, supervisor: ActionSupervisor):
        if arm_mode not in ("LEFT", "RIGHT") or not skills:
            raise ValueError("MVP 只允许单臂且技能序列不能为空")
        self.board = Blackboard(arm_mode, tuple(skills))
        self.observer, self.policy, self.critic = observer, policy, critic
        self.sink, self.clock, self.supervisor = sink, clock, supervisor
        self.takeover = False

    def request_takeover(self):
        self.takeover = True

    def _transition(self, phase, reason=""):
        self.board.phase, self.board.reason = phase, reason
        self.board.history.append((self.board.skill_index, self.board.attempt, phase, reason))

    def _halt(self, reason, human=False):
        if human:
            self.supervisor.hold("人工接管")
            self._transition("HUMAN_TAKEOVER", reason)
        else:
            self.supervisor.stop(reason)
            self._transition("SAFE_STOP", reason)
        try:
            self.sink.cancel()
        except Exception as exc:
            self.board.reason += f"；取消执行器失败：{exc}"

    def _check_deadline(self):
        if self.clock.now_ns() > self.board.attempt_deadline_ns:
            raise TimeoutError("技能超时")

    def _check_observation(self, obs):
        validate_observation(obs)
        if obs["arm_mode"] != self.board.arm_mode:
            raise ValueError("观测执行臂不匹配")
        age = self.clock.now_ns() - obs["observation_timestamp_ns"]
        if not 0 <= age <= self.supervisor.profile.max_observation_age_ns:
            raise ValueError("观测过期或来自未来")
        for source in (*obs["images"].values(), *obs["state"].values()):
            if obs["observation_timestamp_ns"] - source["header_timestamp_ns"] > self.supervisor.profile.max_source_age_ns:
                raise ValueError("观测源数据过期")

    def _goal(self, skill, observation):
        return {
            "contract_version": VERSION, "skill_id": skill.name,
            "instruction": f"{skill.name} {skill.target}", "arm_mode": self.board.arm_mode,
            "target_object_id": skill.target if skill.name == "pick" else None,
            "target_region_id": skill.target if skill.name == "place" else None,
            "timeout_sec": skill.timeout_sec, "max_retries": skill.max_retries,
            "observation_id": observation["observation_id"],
            "safety_profile": self.supervisor.profile.profile_id,
        }

    def run(self):
        board = self.board
        while board.phase not in ("DONE", "SAFE_STOP", "HUMAN_TAKEOVER"):
            if self.takeover:
                self._halt("操作员请求接管", human=True)
                break
            skill = board.skills[board.skill_index]
            try:
                if board.phase == "OBSERVE":
                    board.attempt_deadline_ns = self.clock.now_ns() + int(skill.timeout_sec * 1e9)
                    obs, feedback = self.observer.observe(skill, board.arm_mode, self.clock.now_ns())
                    self._check_deadline()
                    self._check_observation(obs)
                    board.observation = obs
                    arm = board.arm_mode.lower()
                    pos, vel, stamp, healthy = feedback[arm]
                    self.supervisor.update_feedback(arm, pos, vel, stamp, healthy)
                    if self.supervisor.state == "STOP":
                        raise SafetyError(self.supervisor.reason)
                    board.goal = self._goal(skill, obs)
                    self._transition("PLAN")
                elif board.phase == "PLAN":
                    board.action = self.policy.plan(board.observation, board.goal)
                    self._check_deadline()
                    self.supervisor.submit({"observation": board.observation, "skill_goal": board.goal,
                                            "action_chunk": board.action}, self.clock.now_ns())
                    self._transition("EXECUTE")
                elif board.phase == "EXECUTE":
                    self._execute()
                    if board.phase == "EXECUTE":
                        self._transition("VERIFY")
                elif board.phase == "VERIFY":
                    obs, feedback = self.observer.observe(skill, board.arm_mode, self.clock.now_ns())
                    board.observation = obs
                    self._check_observation(obs)
                    arm = board.arm_mode.lower()
                    self.supervisor.update_feedback(arm, *feedback[arm])
                    self._check_deadline()
                    if self.supervisor.state == "STOP":
                        raise SafetyError(self.supervisor.reason)
                    result = self.critic.verify(skill, obs)
                    self._check_deadline()
                    if result == "SUCCESS":
                        board.skill_index += 1
                        board.attempt = 0
                        self._transition("DONE" if board.skill_index == len(board.skills) else "OBSERVE")
                    elif result == "FAIL":
                        self._transition("RECOVER", "验证失败")
                    else:
                        self._halt(f"评估结果不可执行：{result}")
                elif board.phase == "RECOVER":
                    self.supervisor.hold("重试前保持")
                    if board.attempt >= skill.max_retries:
                        self._halt("重试次数耗尽")
                    else:
                        board.attempt += 1
                        self._transition("OBSERVE", "重新观测后重试")
            except Exception as exc:
                self._halt(f"{board.phase}: {exc}")
        return board

    def _execute(self):
        board = self.board
        arm = board.arm_mode.lower()
        action = board.action
        deadline = board.attempt_deadline_ns
        while self.supervisor.state == "RUNNING":
            if self.takeover:
                self._halt("操作员请求接管", human=True)
                return
            due = self.supervisor.plan[2] + (self.supervisor.index + 1) * action["dt_ns"]
            if self.supervisor.index < action["horizon"]:
                self.clock.sleep_until(due)
            now = self.clock.now_ns()
            if now > deadline:
                raise TimeoutError("技能执行超时")
            approved = self.supervisor.tick(now)
            if self.supervisor.state == "STOP":
                raise SafetyError(self.supervisor.reason)
            if approved is None:
                if self.supervisor.state == "RUNNING":
                    raise SafetyError("目标尚未收到新反馈")
                break
            # 执行器无法获得 policy 的原始动作；只接收监督器批准的一个目标。
            pos, vel, stamp, healthy = self.sink.dispatch(approved, now, deadline)
            if self.clock.now_ns() > deadline:
                raise TimeoutError("技能执行超时")
            self.supervisor.update_feedback(arm, pos, vel, stamp, healthy)
            if self.supervisor.state == "STOP":
                raise SafetyError(self.supervisor.reason)
        if self.supervisor.state != "HOLD":
            raise SafetyError("执行未正常完成")
