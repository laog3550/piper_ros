#!/usr/bin/env python3
"""纯离线 pick/place 任务编排。运行：python3 tools/vla_contract/task_orchestrator.py。仅接收注入的观测、策略、评估与合成执行器。"""

from dataclasses import dataclass, field
import math

from action_supervisor import ActionSupervisor, SafetyError
from runtime_trace import TraceError, derived_recovery_id, observation_summary
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

    TOP_RGB = "/camera_top/D455/color/image_raw"

    def __init__(self, builder, feedback_provider):
        self.builder, self.feedback_provider = builder, feedback_provider

    def observe(self, skill, mode, now_ns):
        anchors = (stamp for stamp, *_ in self.builder.buffers[self.TOP_RGB] if stamp <= now_ns)
        anchor = max(anchors, default=None)
        if anchor is None:
            raise ValueError("缺少可用的顶部 RGB Header 锚点")
        record = self.builder.build(mode, anchor, now_ns)
        if not record["contract_observation_valid"]:
            raise ValueError(f"快照无效：{record['rejection_reasons']}")
        if any(frame["uri"].startswith("ros://") for frame in record["observation"]["images"].values()):
            raise ValueError("实时帧 URI 不可作为不可变离线观测")
        arm = mode.lower()
        return record["observation"], {arm: self.feedback_provider.read(arm, now_ns)}


class TaskOrchestrator:
    """依赖全部由调用方注入；目标发送路径只接受监督器 tick 的返回值。"""

    def __init__(self, skills, arm_mode, observer, policy, critic, sink, clock, supervisor: ActionSupervisor,
                 trace_hook=None, trace_ids=None):
        if arm_mode not in ("LEFT", "RIGHT") or not skills:
            raise ValueError("MVP 只允许单臂且技能序列不能为空")
        self.board = Blackboard(arm_mode, tuple(skills))
        self.observer, self.policy, self.critic = observer, policy, critic
        self.sink, self.clock, self.supervisor = sink, clock, supervisor
        self.trace_hook, self.trace_ids = trace_hook, dict(trace_ids or {})
        self.takeover = False

    def _trace(self, event_type, data, skill=None):
        if self.trace_hook is None:
            return
        ids = dict(self.trace_ids)
        if skill is not None:
            ids["skill_id"] = skill.name
        retry = self.board.attempt + (event_type == "recovery_requested")
        if retry and self.trace_hook.manifest["links"].get("recovery_id") is None:
            ids["recovery_id"] = derived_recovery_id(
                self.trace_hook.run_id, f"{ids.get('step_id', 'skill')}:retry:{retry}")
        self.trace_hook.record(event_type, data, ids=ids, clock_ns=self.clock.now_ns())

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
            if human:
                failure = f"人工接管取消执行器失败：{exc}"
                self.supervisor.stop(failure)
                self._transition("SAFE_STOP", failure)
            else:
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
                    self._trace("skill_observation", {"observation": observation_summary(obs),
                                                      "goal": board.goal, "attempt": board.attempt}, skill)
                    self._transition("PLAN")
                elif board.phase == "PLAN":
                    self._trace("policy_request", {"observation": observation_summary(board.observation),
                                                    "goal": board.goal, "attempt": board.attempt}, skill)
                    try:
                        board.action = self.policy.plan(board.observation, board.goal)
                    except Exception as exc:
                        self._trace("policy_failure", {"failure_category": "POLICY_FAILURE",
                                                       "reason": type(exc).__name__}, skill)
                        raise
                    self._check_deadline()
                    self._trace("policy_result", {"action_chunk": board.action,
                                                   "attempt": board.attempt}, skill)
                    try:
                        self.supervisor.submit({"observation": board.observation, "skill_goal": board.goal,
                                                "action_chunk": board.action}, self.clock.now_ns())
                    except Exception as exc:
                        self._trace("preflight_rejected", {"failure_category": "SUPERVISOR_REJECTED",
                                                          "reason": str(exc)}, skill)
                        raise
                    self._trace("preflight_accepted", {"safety_profile": self.supervisor.profile.profile_id,
                                                        "action_model_version": board.action["model_version"],
                                                        "attempt": board.attempt}, skill)
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
                    self._trace("verify_result", {"result": result, "critic_output": getattr(
                        self.critic, "last_output", None), "observation": observation_summary(obs),
                        "attempt": board.attempt}, skill)
                    if result == "SUCCESS":
                        board.skill_index += 1
                        board.attempt = 0
                        self._transition("DONE" if board.skill_index == len(board.skills) else "OBSERVE")
                    elif result == "FAIL":
                        self._trace("recovery_requested", {"failure_category": "VERIFY_FAILED",
                                                           "attempt": board.attempt}, skill)
                        self._transition("RECOVER", "验证失败")
                    else:
                        self._halt(f"评估结果不可执行：{result}")
                elif board.phase == "RECOVER":
                    self.supervisor.hold("重试前保持")
                    if board.attempt >= skill.max_retries:
                        self._halt("重试次数耗尽")
                    else:
                        board.attempt += 1
                        self._trace("recovery_started", {"attempt": board.attempt}, skill)
                        self._transition("OBSERVE", "重新观测后重试")
            except Exception as exc:
                phase = board.phase
                self._halt(f"{phase}: {exc}")
                try:
                    category = ("TRACE_FAILURE" if isinstance(exc, TraceError)
                                else "PREFLIGHT_REJECTED" if phase in ("OBSERVE", "PLAN")
                                else "EXECUTION_FAILURE" if phase == "EXECUTE"
                                else "VERIFY_FAILURE")
                    self._trace("skill_failure", {"failure_category": category,
                                                  "phase": phase, "reason": str(exc)}, skill)
                except Exception:
                    pass
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
            self._trace("execution_request", {"approved_target": approved, "deadline_ns": deadline,
                                              "attempt": board.attempt}, board.skills[board.skill_index])
            pos, vel, stamp, healthy = self.sink.dispatch(approved, now, deadline)
            self._trace("execution_result", {"feedback": {"position": [float(value) for value in pos],
                                                               "velocity": [float(value) for value in vel],
                                                               "timestamp_ns": int(stamp), "healthy": bool(healthy)},
                                             "attempt": board.attempt}, board.skills[board.skill_index])
            if self.clock.now_ns() > deadline:
                raise TimeoutError("技能执行超时")
            self.supervisor.update_feedback(arm, pos, vel, stamp, healthy)
            if self.supervisor.state == "STOP":
                raise SafetyError(self.supervisor.reason)
        if self.supervisor.state != "HOLD":
            raise SafetyError("执行未正常完成")
