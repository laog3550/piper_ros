#!/usr/bin/env python3
"""离线检查 VLA MVP JSON 契约。运行：python3 tools/vla_contract/validate_contract.py --self-test 或 sample.json。"""

import argparse
import copy
import json
import math
from pathlib import Path


VERSION = "piper_vla_mvp/1.0"
JOINTS = [f"joint{i}" for i in range(1, 7)] + ["gripper"]
ARMS = {"LEFT": ["left"], "RIGHT": ["right"], "BIMANUAL": ["left", "right"]}
TOPIC = {
    "top_rgb": "/camera_top/D455/color/image_raw",
    "left_wrist_rgb": "/camera_wrist_left/color/image_raw",
    "right_wrist_rgb": "/camera_wrist_right/color/image_raw",
}


def require(condition, message):
    if not condition:
        raise ValueError(message)


def fields(value, names, where):
    require(isinstance(value, dict) and set(value) == set(names), f"{where}: 字段不匹配")


def integer(value, where, minimum=0):
    require(type(value) is int and value >= minimum, f"{where}: 需要大于等于 {minimum} 的整数")


def number(value, where, positive=False):
    require(type(value) in (int, float) and math.isfinite(value)
            and (not positive or value > 0), f"{where}: 需要有限数")


def label(value, where):
    require(isinstance(value, str) and bool(value.strip()), f"{where}: 需要非空字符串")


def version(value):
    require(value == VERSION, "contract_version: 不支持的版本")


def validate_observation(obs):
    fields(obs, ("contract_version", "observation_id", "observation_timestamp_ns", "time_domain",
                 "arm_mode", "calibration_version", "normalization_version", "images", "state"), "observation")
    version(obs["contract_version"])
    for key in ("observation_id", "calibration_version", "normalization_version"):
        label(obs[key], key)
    integer(obs["observation_timestamp_ns"], "observation_timestamp_ns", 1)
    require(obs["time_domain"] == "unix_ns", "time_domain: 仅支持 unix_ns")
    require(type(obs["arm_mode"]) is str and obs["arm_mode"] in ARMS, "arm_mode: 无效")
    arms = ARMS[obs["arm_mode"]]
    image_names = ["top_rgb"] + [f"{arm}_wrist_rgb" for arm in arms]
    fields(obs["images"], image_names, "images")
    for name, frame in obs["images"].items():
        fields(frame, ("topic", "header_timestamp_ns", "frame_id", "encoding", "width", "height", "uri"), name)
        require(frame["topic"] == TOPIC[name], f"{name}: topic 不匹配")
        integer(frame["header_timestamp_ns"], f"{name}.header_timestamp_ns", 1)
        require(frame["header_timestamp_ns"] <= obs["observation_timestamp_ns"], f"{name}: 时间晚于快照")
        for key in ("frame_id", "uri"):
            label(frame[key], f"{name}.{key}")
        require(frame["encoding"] == "rgb8" and frame["width"] == 640
                and frame["height"] == 480, f"{name}: 图像格式不匹配")
    fields(obs["state"], arms, "state")
    for arm, state in obs["state"].items():
        fields(state, ("source_topic", "header_timestamp_ns", "joint_names", "position"), f"state.{arm}")
        require(state["source_topic"] == f"/joint_{arm}", f"state.{arm}: 来源不匹配")
        integer(state["header_timestamp_ns"], f"state.{arm}.header_timestamp_ns", 1)
        require(state["header_timestamp_ns"] <= obs["observation_timestamp_ns"], f"state.{arm}: 时间晚于快照")
        require(state["joint_names"] == JOINTS, f"state.{arm}: 关节顺序不匹配")
        require(isinstance(state["position"], list) and len(state["position"]) == 7,
                f"state.{arm}: 位置维度不匹配")
        for index, value in enumerate(state["position"]):
            number(value, f"state.{arm}.position[{index}]")
        require(state["position"][6] >= 0, f"state.{arm}: 夹爪开口不能为负")


def validate_goal(goal, obs):
    fields(goal, ("contract_version", "skill_id", "instruction", "arm_mode", "target_object_id",
                  "target_region_id", "timeout_sec", "max_retries", "observation_id", "safety_profile"), "skill_goal")
    version(goal["contract_version"])
    require(goal["skill_id"] in ("pick", "place"), "skill_id: 仅支持 pick/place")
    for key in ("instruction", "safety_profile"):
        label(goal[key], key)
    require(goal["arm_mode"] == obs["arm_mode"] and goal["observation_id"] == obs["observation_id"],
            "skill_goal: 观测关联不匹配")
    target, absent = (("target_object_id", "target_region_id") if goal["skill_id"] == "pick"
                      else ("target_region_id", "target_object_id"))
    label(goal[target], target)
    require(goal[absent] is None, f"{absent}: 必须为 null")
    number(goal["timeout_sec"], "timeout_sec", positive=True)
    integer(goal["max_retries"], "max_retries")


def validate_action(action, obs):
    fields(action, ("contract_version", "observation_id", "arm_mode", "model_version",
                    "normalization_version", "action_space", "dt_ns", "horizon",
                    "observation_timestamp_ns", "inference_timestamp_ns", "targets"), "action_chunk")
    version(action["contract_version"])
    for key in ("model_version", "normalization_version"):
        label(action[key], key)
    require(action["observation_id"] == obs["observation_id"]
            and action["arm_mode"] == obs["arm_mode"], "action_chunk: 观测关联不匹配")
    require(action["normalization_version"] == obs["normalization_version"], "归一化版本不匹配")
    require(action["action_space"] == "absolute_joint_position_7", "action_space: 无效")
    integer(action["dt_ns"], "dt_ns", 1)
    integer(action["horizon"], "horizon", 1)
    integer(action["observation_timestamp_ns"], "action.observation_timestamp_ns", 1)
    integer(action["inference_timestamp_ns"], "inference_timestamp_ns", 1)
    require(action["observation_timestamp_ns"] == obs["observation_timestamp_ns"]
            and action["inference_timestamp_ns"] >= obs["observation_timestamp_ns"],
            "action_chunk: 时间不匹配")
    fields(action["targets"], ARMS[obs["arm_mode"]], "targets")
    for arm, rows in action["targets"].items():
        require(isinstance(rows, list) and len(rows) == action["horizon"], f"targets.{arm}: horizon 不匹配")
        for index, row in enumerate(rows):
            require(isinstance(row, list) and len(row) == 7, f"targets.{arm}[{index}]: 维度不匹配")
            for column, value in enumerate(row):
                number(value, f"targets.{arm}[{index}][{column}]")
            require(row[6] >= 0, f"targets.{arm}[{index}]: 夹爪开口不能为负")


def validate(data):
    fields(data, ("observation", "skill_goal", "action_chunk"), "文件")
    obs = data["observation"]
    validate_observation(obs)
    validate_goal(data["skill_goal"], obs)
    validate_action(data["action_chunk"], obs)


def synthetic(mode="LEFT", skill="pick"):
    arms = ARMS[mode]
    obs = {
        "contract_version": VERSION, "observation_id": "synthetic-001",
        "observation_timestamp_ns": 1_800_000_000_100_000_000, "time_domain": "unix_ns",
        "arm_mode": mode, "calibration_version": "synthetic-calibration",
        "normalization_version": "synthetic-normalization", "images": {}, "state": {},
    }
    for name in ["top_rgb"] + [f"{arm}_wrist_rgb" for arm in arms]:
        obs["images"][name] = {"topic": TOPIC[name], "header_timestamp_ns": 1_800_000_000_090_000_000,
                               "frame_id": f"synthetic_{name}_optical_frame", "encoding": "rgb8",
                               "width": 640, "height": 480, "uri": f"synthetic://{name}"}
    for arm in arms:
        obs["state"][arm] = {"source_topic": f"/joint_{arm}",
                             "header_timestamp_ns": 1_800_000_000_095_000_000,
                             "joint_names": JOINTS.copy(), "position": [0.0] * 6 + [0.04]}
    goal = {"contract_version": VERSION, "skill_id": skill, "instruction": f"synthetic {skill}",
            "arm_mode": mode, "target_object_id": "object-1" if skill == "pick" else None,
            "target_region_id": "region-1" if skill == "place" else None,
            "timeout_sec": 10.0, "max_retries": 0, "observation_id": obs["observation_id"],
            "safety_profile": "offline-only"}
    action = {"contract_version": VERSION, "observation_id": obs["observation_id"],
              "arm_mode": mode, "model_version": "synthetic-model",
              "normalization_version": obs["normalization_version"],
              "action_space": "absolute_joint_position_7", "dt_ns": 100_000_000,
              "horizon": 2, "observation_timestamp_ns": obs["observation_timestamp_ns"],
              "inference_timestamp_ns": obs["observation_timestamp_ns"] + 1_000_000,
              "targets": {arm: [[0.0] * 6 + [0.04]] * 2 for arm in arms}}
    return {"observation": obs, "skill_goal": goal, "action_chunk": action}


def self_test():
    for mode, skill in (("LEFT", "pick"), ("RIGHT", "place"), ("BIMANUAL", "pick")):
        validate(synthetic(mode, skill))
    mutations = (
        lambda d: d["observation"]["state"]["left"]["joint_names"].reverse(),
        lambda d: d["observation"]["state"]["left"]["position"].pop(),
        lambda d: d["observation"]["images"].pop("top_rgb"),
        lambda d: d["action_chunk"]["targets"]["left"][0].__setitem__(0, float("nan")),
        lambda d: d["action_chunk"].__setitem__("horizon", 3),
        lambda d: d["action_chunk"].__setitem__("normalization_version", "other"),
        lambda d: d["action_chunk"].__setitem__("observation_id", "other"),
        lambda d: d["action_chunk"].__setitem__("inference_timestamp_ns", 1),
        lambda d: d["skill_goal"].__setitem__("target_region_id", "unexpected"),
        lambda d: d["observation"].__setitem__("contract_version", "other"),
    )
    for index, mutate in enumerate(mutations, 1):
        sample = copy.deepcopy(synthetic())
        mutate(sample)
        try:
            validate(sample)
        except ValueError:
            continue
        raise AssertionError(f"负例 {index} 未被拒绝")
    print(f"通过：3 个合成正例，{len(mutations)} 个合成负例")


def main():
    parser = argparse.ArgumentParser(description="离线检查 VLA MVP 接口；不连接 ROS/CAN")
    parser.add_argument("file", nargs="?", type=Path, help="含 observation、skill_goal、action_chunk 的 JSON")
    parser.add_argument("--self-test", action="store_true", help="运行内存合成样例")
    args = parser.parse_args()
    if args.self_test:
        self_test()
    elif args.file:
        validate(json.loads(args.file.read_text(encoding="utf-8")))
        print("通过：接口结构检查")
    else:
        parser.error("请提供 JSON 文件或 --self-test")


if __name__ == "__main__":
    main()
