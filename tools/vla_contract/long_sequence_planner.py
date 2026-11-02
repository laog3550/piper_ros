#!/usr/bin/env python3
"""离线长序列规划与严格验收。运行：python3 -m unittest discover -s tools/vla_contract -p 'test_long_sequence_planner.py'。"""

from copy import deepcopy
import json
import re

from task_orchestrator import Skill, TaskOrchestrator
from runtime_trace import TraceError, derived_recovery_id, observation_summary
from validate_contract import validate_observation


VERSION = "piper_vlm_plan/1.0"
MAX_STEPS = 5
TOP_RGB = "top_rgb"


class PlanError(ValueError):
    pass


def _fields(value, names, label):
    if type(value) is not dict or set(value) != set(names):
        raise PlanError(f"{label}: 字段不符合契约")


def _label(value, label):
    if type(value) is not str or not value.strip() or len(value) > 128:
        raise PlanError(f"{label}: 无效标识")


def _mentioned(identifier, instruction):
    boundary = r"[A-Za-z0-9_-]"
    return re.search(rf"(?<!{boundary}){re.escape(identifier)}(?!{boundary})", instruction) is not None


def _goals_satisfied(state, goals):
    return all(state.get(object_id) == {"status": "placed", "region_id": region_id}
               for object_id, region_id in goals.items())


def _catalog(entries, label):
    if type(entries) is not list or not 1 <= len(entries) <= 64:
        raise PlanError(f"{label}: 场景目录无效")
    ids, aliases = [], {}
    for entry in entries:
        _fields(entry, ("id", "aliases"), label)
        _label(entry["id"], f"{label}.id")
        names = entry["aliases"]
        if type(names) is not list or not 1 <= len(names) <= 16:
            raise PlanError(f"{label}: 别名无效")
        if entry["id"] in ids:
            raise PlanError(f"{label}: ID 重复")
        ids.append(entry["id"])
        for name in [entry["id"], *names]:
            _label(name, f"{label}.aliases")
            if name in aliases and aliases[name] != entry["id"]:
                raise PlanError(f"{label}: 别名对应多个 ID；需要人工消歧")
            aliases[name] = entry["id"]
    return ids, aliases


def ground_context(context):
    """把受限中文放置指令绑定到稳定 ID；显式 ID/goal_state 输入保持原样。"""
    if type(context) is not dict:
        raise PlanError("规划输入无效")
    if "scene_objects" not in context and "scene_regions" not in context:
        return context
    required = {"request_id", "instruction", "observation", "scene_objects", "scene_regions",
                "available_skills", "object_state", "completed_step_ids"}
    if set(context) not in (required, required | {"scene_version"}):
        raise PlanError("场景规划输入: 字段不符合契约")
    instruction = context["instruction"]
    if type(instruction) is not str or not instruction.strip() or len(instruction) > 700:
        raise PlanError("自然语言任务无效")
    object_ids, objects = _catalog(context["scene_objects"], "scene_objects")
    region_ids, regions = _catalog(context["scene_regions"], "scene_regions")
    goals = {}
    clauses = [item.strip() for item in re.split(r"[，,；;。]", instruction) if item.strip()]
    if not clauses or len(clauses) > MAX_STEPS:
        raise PlanError("任务步骤缺失或过多；需要人工复核")
    for clause in clauses:
        match = re.fullmatch(r"(?:先|再|然后)?\s*(?:把|将)\s*(.+?)\s*(?:放到|放入|放进)\s*(.+?)", clause)
        if match is None:
            match = re.fullmatch(r"(.+?)\s*(?:→|->)\s*(.+?)", clause)
        if match is None:
            raise PlanError("任务语句无法完整消歧；需要人工复核")
        object_name, region_name = (part.strip() for part in match.groups())
        object_id, region_id = objects.get(object_name), regions.get(region_name)
        if object_id is None or region_id is None:
            raise PlanError("任务目标未匹配已知场景；需要人工复核")
        if object_id in goals:
            raise PlanError("同一物体目标重复或冲突；需要人工复核")
        goals[object_id] = region_id
    bound = "；".join(f"{object_id}→{region_id}" for object_id, region_id in goals.items())
    resolved = {key: deepcopy(context[key]) for key in
                ("request_id", "observation", "available_skills", "object_state", "completed_step_ids")}
    if "scene_version" in context:
        resolved["scene_version"] = context["scene_version"]
    resolved.update(instruction=f"{instruction}\n目标绑定：{bound}", object_ids=object_ids,
                    region_ids=region_ids, goal_state=goals)
    if len(resolved["instruction"]) > 1000:
        raise PlanError("绑定后的任务文本过长")
    return resolved


def validate_context(context, now_ns, max_age_ns):
    required = {"request_id", "instruction", "observation", "object_ids", "region_ids",
                "available_skills", "object_state", "goal_state", "completed_step_ids"}
    if type(context) is not dict or set(context) not in (required, required | {"scene_version"}):
        raise PlanError("规划输入: 字段不符合契约")
    if "scene_version" in context:
        _label(context["scene_version"], "scene_version")
    _label(context["request_id"], "request_id")
    if type(context["instruction"]) is not str or not context["instruction"].strip() or len(context["instruction"]) > 1000:
        raise PlanError("任务文本无效")
    try:
        validate_observation(context["observation"])
    except (ValueError, KeyError, TypeError) as exc:
        raise PlanError(f"观测无效：{exc}") from exc
    obs = context["observation"]
    if obs["arm_mode"] not in ("LEFT", "RIGHT"):
        raise PlanError("仅支持单臂规划")
    if type(now_ns) is not int or type(max_age_ns) is not int or max_age_ns <= 0:
        raise PlanError("规划时间配置无效")
    if not 0 <= now_ns - obs["observation_timestamp_ns"] <= max_age_ns:
        raise PlanError("观测过期或来自未来；需要重新观察")
    if any(obs["observation_timestamp_ns"] - source["header_timestamp_ns"] > max_age_ns
           for source in (*obs["images"].values(), *obs["state"].values())):
        raise PlanError("观测源数据过期；需要重新观察")
    for key in ("object_ids", "region_ids"):
        values = context[key]
        if type(values) is not list or not values or len(values) > 64:
            raise PlanError(f"{key}: 缺失或过多")
        for value in values:
            _label(value, key)
        if len(set(values)) != len(values):
            raise PlanError(f"{key}: 重复")
    skills = context["available_skills"]
    if (type(skills) is not list or not skills or any(type(x) is not str for x in skills)
            or len(set(skills)) != len(skills) or not set(skills) <= {"pick", "place"}):
        raise PlanError("可用技能无效")
    state = context["object_state"]
    if type(state) is not dict or set(state) != set(context["object_ids"]):
        raise PlanError("物体状态与 ID 不匹配")
    held = 0
    for object_id, item in state.items():
        _fields(item, ("status", "region_id"), f"object_state.{object_id}")
        status, region = item["status"], item["region_id"]
        if status not in ("available", "held", "placed") or (status == "placed") != (region is not None):
            raise PlanError("物体状态无效")
        if region is not None and region not in context["region_ids"]:
            raise PlanError("物体区域未知")
        held += status == "held"
    if held > 1:
        raise PlanError("单臂不能同时持有两个物体")
    goals = context["goal_state"]
    if type(goals) is not dict or not goals or len(goals) > MAX_STEPS:
        raise PlanError("目标终态缺失或过多；需要人工消歧")
    if any(object_id not in context["object_ids"] or region_id not in context["region_ids"]
           for object_id, region_id in goals.items()):
        raise PlanError("目标终态含未知物体或区域")
    completed = context["completed_step_ids"]
    if type(completed) is not list or len(completed) > MAX_STEPS:
        raise PlanError("已完成步骤无效")
    for value in completed:
        _label(value, "completed_step_ids")
    if len(set(completed)) != len(completed):
        raise PlanError("已完成步骤重复")
    return context


def parse_plan(raw, context):
    if type(context) is not dict or type(context.get("goal_state")) is not dict or not context["goal_state"]:
        raise PlanError("缺少可核对的任务目标终态")
    try:
        plan = json.loads(raw) if type(raw) is str else deepcopy(raw)
    except (ValueError, TypeError) as exc:
        raise PlanError("模型输出不是 JSON") from exc
    _fields(plan, ("schema_version", "request_id", "observation_id", "steps"), "计划")
    obs = context["observation"]
    if (plan["schema_version"] != VERSION or plan["request_id"] != context["request_id"]
            or plan["observation_id"] != obs["observation_id"]):
        raise PlanError("计划版本或观测引用不匹配")
    steps = plan["steps"]
    if type(steps) is not list or not 1 <= len(steps) <= MAX_STEPS:
        raise PlanError("计划长度无效")
    if len(context["object_ids"]) > 1:
        for step in steps:
            if type(step) is dict and type(step.get("object_id")) is str and not _mentioned(step["object_id"], context["instruction"]):
                raise PlanError("多物体任务目标未在文本中明确绑定 ID；需要人工消歧")
    if len(context["region_ids"]) > 1:
        for step in steps:
            if (type(step) is dict and step.get("skill") == "place" and type(step.get("region_id")) is str
                    and not _mentioned(step["region_id"], context["instruction"])):
                raise PlanError("多区域任务目标未在文本中明确绑定 ID；需要人工消歧")
    state = deepcopy(context["object_state"])
    held = next((name for name, item in state.items() if item["status"] == "held"), None)
    seen, picked, placed = set(context["completed_step_ids"]), set(), set()
    previous = None
    for index, step in enumerate(steps):
        _fields(step, ("step_id", "skill", "arm", "object_id", "region_id", "depends_on", "completion"),
                f"steps[{index}]")
        sid, skill, arm = step["step_id"], step["skill"], step["arm"]
        _label(sid, "step_id")
        if sid in seen:
            raise PlanError("步骤 ID 重复或已完成")
        seen.add(sid)
        if skill not in context["available_skills"] or arm != obs["arm_mode"]:
            raise PlanError("非法技能或执行臂")
        if step["object_id"] not in context["object_ids"]:
            raise PlanError("未知物体")
        if step["object_id"] not in context["goal_state"]:
            raise PlanError("步骤操作了任务目标之外的物体")
        if step["depends_on"] != ([] if previous is None else [previous]):
            raise PlanError("依赖必须形成无跳步的线性序列")
        object_id, region = step["object_id"], step["region_id"]
        _fields(step["completion"], ("type", "object_id", "region_id"), "完成条件")
        condition = step["completion"]
        if condition["object_id"] != object_id:
            raise PlanError("完成条件物体不匹配")
        if skill == "pick":
            if (region is not None or condition != {"type": "object_held", "object_id": object_id,
                                                       "region_id": None} or held is not None
                    or state[object_id]["status"] != "available" or object_id in picked):
                raise PlanError("pick 前置条件、目标或完成条件矛盾")
            state[object_id] = {"status": "held", "region_id": None}
            held = object_id
            picked.add(object_id)
        else:
            if (region != context["goal_state"][object_id] or condition != {"type": "object_in_region",
                    "object_id": object_id, "region_id": region} or held != object_id
                    or object_id in placed):
                raise PlanError("place 前置条件、区域或完成条件矛盾")
            state[object_id] = {"status": "placed", "region_id": region}
            held = None
            placed.add(object_id)
        previous = sid
    if not _goals_satisfied(state, context["goal_state"]):
        raise PlanError("计划未覆盖任务目标终态")
    return plan


class LongSequencePlanner:
    """backend.generate(context) 只生成文本；校验失败时不产生技能。"""

    def __init__(self, backend, max_age_ns=50_000_000, now_provider=None):
        self.backend, self.max_age_ns, self.now_provider = backend, max_age_ns, now_provider

    def plan(self, context, now_ns):
        context, plan = self.propose(context, now_ns)
        if self.now_provider is not None:
            validate_context(context, self.now_provider(), self.max_age_ns)
        return plan

    def propose(self, context, now_ns):
        """返回待确认计划；推理后的时效和现场状态交由执行前刷新处理。"""
        context = ground_context(context)
        validate_context(context, now_ns, self.max_age_ns)
        try:
            raw = self.backend.generate(deepcopy(context))
        except Exception as exc:
            raise PlanError(f"规划后端失败：{type(exc).__name__}") from exc
        plan = parse_plan(raw, context)
        return context, plan


class SequenceRunner:
    """只在离线任务工厂中逐步运行；真实 critic 的 HUMAN_REVIEW 必须停机。"""

    def __init__(self, planner, task_factory, world_reader, max_replans=1, trace_hook=None):
        if type(max_replans) is not int or not 0 <= max_replans <= 2:
            raise ValueError("重新规划次数无效")
        self.planner, self.task_factory, self.world_reader = planner, task_factory, world_reader
        self.max_replans = max_replans
        self.trace_hook = trace_hook

    def _trace(self, event_type, data, current=None, step=None, replans=0, clock=None):
        if self.trace_hook is None:
            return
        ids = {}
        if current is not None:
            ids["request_id"] = current["request_id"]
            ids["observation_id"] = current["observation"]["observation_id"]
        if step is not None:
            ids.update(step_id=step["step_id"], skill_id=step["skill"])
        if replans and self.trace_hook.manifest["links"].get("recovery_id") is None:
            ids["recovery_id"] = derived_recovery_id(self.trace_hook.run_id, f"replan:{replans}")
        self.trace_hook.record(event_type, data, ids=ids,
                               clock_ns=clock.now_ns() if clock is not None else None)

    def run(self, context, clock):
        if self.trace_hook is None:
            return self._run(context, clock)
        self._trace_progress, self._trace_replans = [], 0
        try:
            result = self._run(context, clock)
        except TraceError as exc:
            result = {"status": "HUMAN_REVIEW", "reason": f"轨迹导出失败：{exc}",
                      "completed": self._trace_progress.copy(), "replans": self._trace_replans}
        try:
            if result["status"] != "DONE":
                self._trace("run_failure", {"failure_category": result["status"],
                                            "reason": result["reason"],
                                            "completed": result["completed"]}, clock=clock)
            self.trace_hook.finish(result["status"], clock_ns=clock.now_ns())
        except TraceError as exc:
            return {"status": "HUMAN_REVIEW", "reason": f"轨迹导出失败：{exc}",
                    "completed": result["completed"], "replans": result["replans"]}
        return result

    def _refresh(self, current, clock):
        if "scene_version" not in current:
            raise PlanError("缺少场景版本；需要人工复核")
        if self.world_reader is None or not callable(getattr(self.world_reader, "refresh_plan_snapshot", None)):
            raise PlanError("缺少可信现场刷新来源；需要人工复核")
        try:
            snapshot = self.world_reader.refresh_plan_snapshot()
        except Exception as exc:
            raise PlanError(f"现场刷新失败：{type(exc).__name__}") from exc
        _fields(snapshot, ("trusted", "scene_version", "observation", "object_ids", "region_ids",
                           "object_state", "goal_state"), "现场快照")
        if snapshot["trusted"] is not True:
            raise PlanError("现场快照来源不可信；需要人工复核")
        if snapshot["scene_version"] != current["scene_version"]:
            raise PlanError("场景版本已变化；需要人工复核")
        for name in ("object_ids", "region_ids", "object_state", "goal_state"):
            if snapshot[name] != current[name]:
                raise PlanError(f"现场{name}已变化；需要人工复核")
        old_obs, new_obs = current["observation"], snapshot["observation"]
        if type(new_obs) is not dict:
            raise PlanError("现场观测无效；需要人工复核")
        for name in ("contract_version", "time_domain", "arm_mode", "calibration_version",
                     "normalization_version"):
            if new_obs.get(name) != old_obs[name]:
                raise PlanError(f"现场{name}已变化；需要人工复核")
        for source_group in ("images", "state"):
            if type(new_obs.get(source_group)) is not dict or set(new_obs[source_group]) != set(old_obs[source_group]):
                raise PlanError("现场观测来源已变化；需要人工复核")
            for name, old_source in old_obs[source_group].items():
                new_source = new_obs[source_group][name]
                stable = (("topic", "frame_id", "encoding", "width", "height") if source_group == "images"
                          else ("source_topic", "joint_names"))
                if type(new_source) is not dict or any(new_source.get(key) != old_source[key] for key in stable):
                    raise PlanError("现场观测来源已变化；需要人工复核")
                if (type(new_source.get("header_timestamp_ns")) is not int
                        or new_source["header_timestamp_ns"] <= old_source["header_timestamp_ns"]):
                    raise PlanError("现场观测源数据未更新；需要人工复核")
                if source_group == "images" and new_source.get("uri") == old_source["uri"]:
                    raise PlanError("现场图像未更新；需要人工复核")
        if (new_obs.get("observation_id") == old_obs["observation_id"]
                or type(new_obs.get("observation_timestamp_ns")) is not int
                or new_obs["observation_timestamp_ns"] <= old_obs["observation_timestamp_ns"]):
            raise PlanError("现场观测未更新；需要人工复核")
        refreshed = deepcopy(current)
        refreshed["observation"] = deepcopy(new_obs)
        validate_context(refreshed, clock.now_ns(), self.planner.max_age_ns)
        return refreshed

    def _run(self, context, clock):
        progress = []
        replans = 0
        remaining = MAX_STEPS
        try:
            current = deepcopy(ground_context(context))
            validate_context(current, clock.now_ns(), self.planner.max_age_ns)
            if self.trace_hook is not None:
                self._trace_progress = progress
            self._trace("run_start", {"instruction": current.get("instruction"),
                                      "goal_state": current.get("goal_state"),
                                      "scene_version": current.get("scene_version")},
                        current=current, clock=clock)
            if "scene_version" not in current:
                raise PlanError("缺少场景版本；需要人工复核")
            if self.world_reader is None or not callable(getattr(self.world_reader, "refresh_plan_snapshot", None)):
                raise PlanError("缺少可信现场刷新来源；需要人工复核")
        except PlanError as exc:
            return {"status": "HUMAN_REVIEW", "reason": str(exc), "completed": progress, "replans": replans}
        while remaining > 0:
            try:
                validate_context(current, clock.now_ns(), self.planner.max_age_ns)
                self._trace("plan_request", {"instruction": current["instruction"],
                                             "scene_version": current["scene_version"],
                                             "object_ids": current["object_ids"],
                                             "region_ids": current["region_ids"],
                                             "goal_state": current["goal_state"],
                                             "object_state": current["object_state"],
                                             "available_skills": current["available_skills"],
                                             "completed_step_ids": current["completed_step_ids"],
                                             "observation": observation_summary(current["observation"])},
                            current=current, replans=replans, clock=clock)
                current, pending = self.planner.propose(current, clock.now_ns())
                raw = getattr(self.planner.backend, "last_raw", None)
                self._trace("plan_result", {"plan": pending,
                                            "raw_model_output": raw if type(raw) is str else None},
                            current=current, replans=replans, clock=clock)
                current = self._refresh(current, clock)
                plan = deepcopy(pending)
                plan["observation_id"] = current["observation"]["observation_id"]
                parse_plan(plan, current)
                self._trace("plan_preflight", {"accepted": True, "rebound_plan": plan,
                                               "fresh_observation": observation_summary(current["observation"])},
                            current=current, replans=replans, clock=clock)
            except PlanError as exc:
                status = "REOBSERVE" if "观测" in str(exc) and "过期" in str(exc) else "HUMAN_REVIEW"
                raw = getattr(self.planner.backend, "last_raw", None)
                self._trace("plan_preflight", {"accepted": False, "failure_category": "PLAN_REJECTED",
                                               "reason": str(exc),
                                               "raw_model_output": raw if type(raw) is str else None},
                            current=current, replans=replans, clock=clock)
                return {"status": status, "reason": str(exc), "completed": progress, "replans": replans}
            requested_replan = False
            for step in plan["steps"]:
                if remaining == 0:
                    return {"status": "HUMAN_REVIEW", "reason": "执行步数上限", "completed": progress, "replans": replans}
                try:
                    current = self._refresh(current, clock)
                    object_state = current["object_state"][step["object_id"]]
                    if (step["skill"] == "pick" and object_state != {"status": "available", "region_id": None}
                            or step["skill"] == "place" and object_state != {"status": "held", "region_id": None}):
                        raise PlanError("执行前置条件已变化；需要人工复核")
                    self._trace("step_preflight", {"accepted": True, "step": step,
                                                   "fresh_observation": observation_summary(current["observation"]),
                                                   "object_state": object_state},
                                current=current, step=step, replans=replans, clock=clock)
                except PlanError as exc:
                    status = "REOBSERVE" if "观测" in str(exc) and "过期" in str(exc) else "HUMAN_REVIEW"
                    self._trace("step_preflight", {"accepted": False,
                                                   "failure_category": "STEP_PREFLIGHT_REJECTED",
                                                   "reason": str(exc)},
                                current=current, step=step, replans=replans, clock=clock)
                    return {"status": status, "reason": str(exc), "completed": progress, "replans": replans}
                skill = Skill(step["skill"], step["object_id"] if step["skill"] == "pick" else step["region_id"],
                              max_retries=1)
                try:
                    task = self.task_factory(skill, step)
                    if not isinstance(task, TaskOrchestrator):
                        raise TypeError("必须使用 TaskOrchestrator")
                    if task.board.skills != (skill,) or task.board.arm_mode != step["arm"]:
                        raise ValueError("任务工厂与已校验步骤不匹配")
                    if self.trace_hook is not None:
                        if task.trace_hook not in (None, self.trace_hook):
                            raise ValueError("任务轨迹钩子不一致")
                        task.trace_hook = self.trace_hook
                        task.trace_ids = {"request_id": current["request_id"], "step_id": step["step_id"],
                                          "observation_id": current["observation"]["observation_id"]}
                    board = task.run()
                except Exception as exc:
                    self._trace("step_result", {"status": "HUMAN_REVIEW",
                                                "failure_category": "ORCHESTRATION_FAILURE",
                                                "reason": type(exc).__name__},
                                current=current, step=step, replans=replans, clock=clock)
                    return {"status": "HUMAN_REVIEW", "reason": f"离线编排异常：{type(exc).__name__}",
                            "completed": progress, "replans": replans}
                self._trace("step_result", {"phase": board.phase, "reason": board.reason,
                                            "history": board.history,
                                            "failure_category": None if board.phase == "DONE" else "TASK_SAFE_STOP"},
                            current=current, step=step, replans=replans, clock=clock)
                remaining -= 1
                if board.phase != "DONE":
                    return {"status": "HUMAN_REVIEW", "reason": board.reason,
                            "completed": progress, "replans": replans}
                try:
                    state, replan_required = self.world_reader(step)
                except Exception as exc:
                    task.supervisor.stop("世界状态读取失败")
                    try:
                        task.sink.cancel()
                    except Exception:
                        pass
                    self._trace("world_verify", {"accepted": False,
                                                 "failure_category": "WORLD_STATE_READ_FAILURE",
                                                 "reason": type(exc).__name__},
                                current=current, step=step, replans=replans, clock=clock)
                    return {"status": "HUMAN_REVIEW", "reason": f"世界状态读取失败：{type(exc).__name__}",
                            "completed": progress, "replans": replans}
                expected = ({"status": "held", "region_id": None} if step["skill"] == "pick"
                            else {"status": "placed", "region_id": step["region_id"]})
                if type(state) is not dict or state.get(step["object_id"]) != expected:
                    task.supervisor.stop("critic 与世界完成条件不一致")
                    try:
                        task.sink.cancel()
                    except Exception:
                        pass
                    self._trace("world_verify", {"accepted": False,
                                                 "failure_category": "WORLD_MISMATCH",
                                                 "expected": expected, "actual": state},
                                current=current, step=step, replans=replans, clock=clock)
                    return {"status": "HUMAN_REVIEW", "reason": "critic 与世界完成条件不一致",
                            "completed": progress, "replans": replans}
                predicted = deepcopy(current["object_state"])
                predicted[step["object_id"]] = expected
                replan_required = replan_required or state != predicted
                self._trace("world_verify", {"accepted": True, "expected": expected,
                                             "actual": state, "replan_required": replan_required},
                            current=current, step=step, replans=replans, clock=clock)
                progress.append(step["step_id"])
                current["object_state"] = deepcopy(state)
                current["completed_step_ids"] = progress.copy()
                if replan_required:
                    requested_replan = True
                    break
            if not requested_replan:
                if not _goals_satisfied(current["object_state"], current["goal_state"]):
                    return {"status": "HUMAN_REVIEW", "reason": "世界状态未达到任务目标终态",
                            "completed": progress, "replans": replans}
                return {"status": "DONE", "reason": "合成离线序列结束", "completed": progress,
                        "replans": replans}
            if replans >= self.max_replans:
                return {"status": "HUMAN_REVIEW", "reason": "重新规划次数耗尽",
                        "completed": progress, "replans": replans}
            replans += 1
            if self.trace_hook is not None:
                self._trace_replans = replans
            try:
                current = self._refresh(current, clock)
            except PlanError as exc:
                return {"status": "HUMAN_REVIEW", "reason": str(exc), "completed": progress, "replans": replans}
        return {"status": "HUMAN_REVIEW", "reason": "执行步数上限", "completed": progress, "replans": replans}
