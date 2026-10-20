#!/usr/bin/env python3
"""离线长序列规划与严格验收。运行：python3 -m unittest discover -s tools/vla_contract -p 'test_long_sequence_planner.py'。"""

from copy import deepcopy
import json

from task_orchestrator import Skill, TaskOrchestrator
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


def validate_context(context, now_ns, max_age_ns):
    _fields(context, ("request_id", "instruction", "observation", "object_ids", "region_ids",
                      "available_skills", "object_state", "completed_step_ids"), "规划输入")
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
    completed = context["completed_step_ids"]
    if type(completed) is not list or len(completed) > MAX_STEPS:
        raise PlanError("已完成步骤无效")
    for value in completed:
        _label(value, "completed_step_ids")
    if len(set(completed)) != len(completed):
        raise PlanError("已完成步骤重复")
    return context


def parse_plan(raw, context):
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
            if (region not in context["region_ids"] or condition != {"type": "object_in_region",
                    "object_id": object_id, "region_id": region} or held != object_id
                    or object_id in placed):
                raise PlanError("place 前置条件、区域或完成条件矛盾")
            state[object_id] = {"status": "placed", "region_id": region}
            held = None
            placed.add(object_id)
        previous = sid
    return plan


class LongSequencePlanner:
    """backend.generate(context) 只生成文本；校验失败时不产生技能。"""

    def __init__(self, backend, max_age_ns=50_000_000, now_provider=None):
        self.backend, self.max_age_ns, self.now_provider = backend, max_age_ns, now_provider

    def plan(self, context, now_ns):
        validate_context(context, now_ns, self.max_age_ns)
        try:
            raw = self.backend.generate(deepcopy(context))
        except Exception as exc:
            raise PlanError(f"规划后端失败：{type(exc).__name__}") from exc
        plan = parse_plan(raw, context)
        if self.now_provider is not None:
            validate_context(context, self.now_provider(), self.max_age_ns)
        return plan


class SequenceRunner:
    """只在离线任务工厂中逐步运行；真实 critic 的 HUMAN_REVIEW 必须停机。"""

    def __init__(self, planner, task_factory, world_reader, max_replans=1):
        if type(max_replans) is not int or not 0 <= max_replans <= 2:
            raise ValueError("重新规划次数无效")
        self.planner, self.task_factory, self.world_reader = planner, task_factory, world_reader
        self.max_replans = max_replans

    def run(self, context, clock):
        progress = []
        replans = 0
        remaining = MAX_STEPS
        current = deepcopy(context)
        while remaining > 0:
            try:
                plan = self.planner.plan(current, clock.now_ns())
                validate_context(current, clock.now_ns(), self.planner.max_age_ns)
            except PlanError as exc:
                status = "REOBSERVE" if "观测" in str(exc) and "过期" in str(exc) else "HUMAN_REVIEW"
                return {"status": status, "reason": str(exc), "completed": progress, "replans": replans}
            requested_replan = False
            for step in plan["steps"]:
                if remaining == 0:
                    return {"status": "HUMAN_REVIEW", "reason": "执行步数上限", "completed": progress, "replans": replans}
                skill = Skill(step["skill"], step["object_id"] if step["skill"] == "pick" else step["region_id"],
                              max_retries=1)
                try:
                    task = self.task_factory(skill, step)
                    if not isinstance(task, TaskOrchestrator):
                        raise TypeError("必须使用 TaskOrchestrator")
                    if task.board.skills != (skill,) or task.board.arm_mode != step["arm"]:
                        raise ValueError("任务工厂与已校验步骤不匹配")
                    board = task.run()
                except Exception as exc:
                    return {"status": "HUMAN_REVIEW", "reason": f"离线编排异常：{type(exc).__name__}",
                            "completed": progress, "replans": replans}
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
                    return {"status": "HUMAN_REVIEW", "reason": "critic 与世界完成条件不一致",
                            "completed": progress, "replans": replans}
                predicted = deepcopy(current["object_state"])
                predicted[step["object_id"]] = expected
                replan_required = replan_required or state != predicted
                progress.append(step["step_id"])
                current["object_state"] = deepcopy(state)
                current["completed_step_ids"] = progress.copy()
                if replan_required:
                    requested_replan = True
                    break
            if not requested_replan:
                return {"status": "DONE", "reason": "合成离线序列结束", "completed": progress,
                        "replans": replans}
            if replans >= self.max_replans:
                return {"status": "HUMAN_REVIEW", "reason": "重新规划次数耗尽",
                        "completed": progress, "replans": replans}
            replans += 1
            current["observation"] = self.world_reader.refresh_observation()
        return {"status": "HUMAN_REVIEW", "reason": "执行步数上限", "completed": progress, "replans": replans}
