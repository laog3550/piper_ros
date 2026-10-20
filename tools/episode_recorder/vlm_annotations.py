"""校验 VLM 人工标注和规划样本；由 episode_recorder.py 与 web_console.py 导入。"""

import re


DECISIONS = ("continue", "retry", "reobserve", "replan", "stop", "ask_human")
OUTCOMES = ("success", "failure", "unknown")
FAILURE_REASONS = ("execution_error", "grasp_failed", "object_not_found",
                   "placement_failed", "other", "unable_to_determine")
IDENTIFIER = re.compile(r"^[A-Za-z0-9_.-]{1,120}$")


def required_text(value, label, limit=2000):
    if not isinstance(value, str) or not value.strip() or len(value.strip()) > limit:
        raise ValueError(f"{label}须为 1 至 {limit} 字的文本")
    return value.strip()


def optional_text(value, label, limit=2000):
    if value is None or value == "":
        return None
    return required_text(value, label, limit)


def optional_id(value, label):
    if value is None or value == "":
        return None
    if not isinstance(value, str) or not IDENTIFIER.fullmatch(value):
        raise ValueError(f"{label}只能包含字母、数字、下划线、点和连字符，最多 120 字")
    return value


def id_list(value, label):
    if not isinstance(value, list) or len(value) > 100:
        raise ValueError(f"{label}须为最多 100 项的列表")
    result = [optional_id(item, label) for item in value]
    if any(item is None for item in result) or len(set(result)) != len(result):
        raise ValueError(f"{label}不能有空值或重复 ID")
    return result


def validate_plan(value):
    if not isinstance(value, dict):
        raise ValueError("规划样本须为 JSON 对象")
    object_ids = id_list(value.get("object_ids", []), "物体 ID")
    location_ids = id_list(value.get("location_ids", []), "位置 ID")
    steps = value.get("steps")
    if not isinstance(steps, list) or not 1 <= len(steps) <= 100:
        raise ValueError("规划步骤须为 1 至 100 项的列表")
    normalized = []
    for index, step in enumerate(steps, 1):
        if not isinstance(step, dict):
            raise ValueError(f"第 {index} 步须为 JSON 对象")
        normalized.append({
            "order": index,
            "skill": required_text(step.get("skill"), f"第 {index} 步技能名", 120),
            "target_object_id": optional_id(step.get("target_object_id"), "目标物体 ID"),
            "target_location_id": optional_id(step.get("target_location_id"), "目标位置 ID"),
            "completion_criteria": required_text(step.get("completion_criteria"),
                                                 f"第 {index} 步完成判据"),
        })
        if object_ids and normalized[-1]["target_object_id"] not in (None, *object_ids):
            raise ValueError(f"第 {index} 步目标物体不在场景目录中")
        if location_ids and normalized[-1]["target_location_id"] not in (None, *location_ids):
            raise ValueError(f"第 {index} 步目标位置不在场景目录中")
    return {
        "task_group_id": optional_id(value.get("task_group_id"), "task_group_id"),
        "long_task_description": required_text(value.get("long_task_description"), "长任务描述"),
        "scene": {"object_ids": object_ids, "location_ids": location_ids},
        "steps": normalized,
    }


def validate_skill_context(value):
    if not isinstance(value, dict):
        raise ValueError("技能上下文须为 JSON 对象")
    return {
        "task_group_id": optional_id(value.get("task_group_id"), "task_group_id"),
        "plan_id": optional_id(value.get("plan_id"), "plan_id"),
        "long_task_description": required_text(value.get("long_task_description"), "长任务描述"),
        "skill_instruction": required_text(value.get("skill_instruction"), "技能短指令"),
        "target_object_id": optional_id(value.get("target_object_id"), "目标物体 ID"),
        "target_location_id": optional_id(value.get("target_location_id"), "目标位置 ID"),
        "expected_state_change": required_text(value.get("expected_state_change"), "预期状态变化"),
        "completion_criteria": required_text(value.get("completion_criteria"), "完成判据"),
    }


def validate_plan_link(plan, context, skill):
    """仅允许技能片段关联到同一计划中的完整对应步骤。"""
    if context["long_task_description"] != plan["long_task_description"]:
        raise ValueError("技能片段长任务描述与规划样本不一致")
    expected = (skill, context["target_object_id"], context["target_location_id"],
                context["completion_criteria"])
    if not any((step["skill"], step["target_object_id"], step["target_location_id"],
                step["completion_criteria"]) == expected for step in plan["steps"]):
        raise ValueError("技能片段与 plan_id 的步骤、目标或完成判据不一致")


def validate_annotation(value):
    if not isinstance(value, dict):
        raise ValueError("结果标注须为 JSON 对象")
    outcome = value.get("outcome")
    if outcome not in OUTCOMES:
        raise ValueError("结果须为 success、failure 或 unknown")
    reason = value.get("failure_reason") or None
    if outcome == "failure" and reason not in FAILURE_REASONS:
        raise ValueError("失败样本须选择失败原因")
    if outcome != "failure" and reason is not None:
        raise ValueError("仅失败样本可以填写失败原因")
    decision = value.get("next_decision")
    if decision not in DECISIONS:
        raise ValueError("请选择人工确认的下一决策")
    detail = optional_text(value.get("failure_reason_detail"), "失败原因补充")
    if detail and reason != "other":
        raise ValueError("失败原因补充仅用于“其他”")
    return {
        "source": "human", "outcome": outcome, "failure_reason": reason,
        "failure_reason_detail": detail,
        "observed_state_change": optional_text(value.get("observed_state_change"), "实际状态变化"),
        "next_decision": decision,
        "next_skill": optional_text(value.get("next_skill"), "下一技能", 120),
    }
