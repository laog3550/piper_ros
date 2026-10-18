#!/usr/bin/env python3
"""Piper π0.5 的纯离线状态、图像和绝对动作适配；运行：python3 tools/vla_contract/piper_pi05_adapter.py --self-test。"""

import argparse
import numpy as np


ACTION_SPACE = "absolute_joint_position_7"
JOINT_NAMES = tuple(f"joint{i}" for i in range(1, 7)) + ("gripper",)
STATE_DIM = ACTION_DIM = 7
MODEL_ACTION_DIM = 32
ACTION_HORIZON = 16


def _vector(value, *, sequence=False):
    array = np.asarray(value, dtype=np.float32)
    if sequence:
        if array.ndim != 2 or array.shape[1] != ACTION_DIM or array.shape[0] != ACTION_HORIZON:
            raise ValueError(f"Piper action 必须为 {ACTION_HORIZON}×7 的绝对目标")
    elif array.shape != (STATE_DIM,):
        raise ValueError("Piper state 必须为 joint1..joint6 rad + gripper m 的 7 维向量")
    if not np.isfinite(array).all() or np.any(array[..., 6] < 0):
        raise ValueError("Piper 位置含非有限数或负夹爪开口")
    return array


def _image(value):
    array = np.asarray(value)
    if array.shape == (3, 480, 640):
        array = np.transpose(array, (1, 2, 0))
    if array.shape != (480, 640, 3):
        raise ValueError("Piper 图像必须为 640×480 RGB")
    if np.issubdtype(array.dtype, np.floating):
        if not np.isfinite(array).all() or array.min() < 0 or array.max() > 1:
            raise ValueError("浮点图像必须在 [0,1] 内")
        array = np.rint(array * 255).astype(np.uint8)
    elif array.dtype != np.uint8:
        raise ValueError("图像 dtype 必须是 uint8 或 [0,1] 浮点数")
    return array


class PiperInputs:
    """LeRobot 观测或同名推理输入到 openpi 模型输入，保留绝对位置语义。"""

    def __init__(self, arm="left"):
        if arm not in ("left", "right"):
            raise ValueError("仅支持单臂 left/right")
        self.arm = arm

    def __call__(self, data):
        if set(data) - {"images", "state", "actions", "prompt"}:
            raise ValueError("Piper 输入含未知字段")
        if not {"images", "state", "prompt"} <= set(data):
            raise ValueError("Piper 输入缺图像、状态或任务文本")
        images = data["images"]
        if set(images) != {"top_rgb", "wrist_rgb"}:
            raise ValueError("单臂 Piper 需要 top_rgb 和 wrist_rgb")
        top, wrist = _image(images["top_rgb"]), _image(images["wrist_rgb"])
        prompt = data["prompt"]
        if isinstance(prompt, bytes):
            prompt = prompt.decode("utf-8")
        if not isinstance(prompt, str) or not prompt.strip():
            raise ValueError("任务文本必须非空")
        left = self.arm == "left"
        result = {
            "state": _vector(data["state"]),
            "image": {"base_0_rgb": top, "left_wrist_0_rgb": wrist if left else np.zeros_like(top),
                      "right_wrist_0_rgb": np.zeros_like(top) if left else wrist},
            "image_mask": {"base_0_rgb": np.True_, "left_wrist_0_rgb": np.bool_(left),
                           "right_wrist_0_rgb": np.bool_(not left)},
            "prompt": prompt,
        }
        if "actions" in data:
            result["actions"] = _vector(data["actions"], sequence=True)
        return result


class PiperOutputs:
    """只从 Piper 已微调模型的 32 维内部动作提取 7 维绝对目标。"""

    def __call__(self, data):
        actions = np.asarray(data["actions"], dtype=np.float32)
        if actions.shape != (ACTION_HORIZON, MODEL_ACTION_DIM) or not np.isfinite(actions).all():
            raise ValueError("模型动作必须为有限的 16×32")
        return {"actions": _vector(actions[:, :ACTION_DIM], sequence=True)}


def self_test():
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    state = np.zeros(7, dtype=np.float32)
    actions = np.zeros((ACTION_HORIZON, ACTION_DIM), dtype=np.float32)
    result = PiperInputs()({"images": {"top_rgb": image, "wrist_rgb": image},
                            "state": state, "actions": actions, "prompt": "抓取香蕉"})
    assert result["state"].shape == (7,) and result["actions"].shape == (16, 7)
    right = PiperInputs("right")({"images": {"top_rgb": image, "wrist_rgb": image},
                                  "state": state, "prompt": "放下香蕉"})
    assert right["image_mask"]["right_wrist_0_rgb"] and not right["image_mask"]["left_wrist_0_rgb"]
    assert PiperOutputs()({"actions": np.zeros((16, 32))})["actions"].shape == (16, 7)
    for invalid in (np.full(7, np.nan), np.zeros(6), np.array([0] * 6 + [-0.1])):
        try:
            _vector(invalid)
        except ValueError:
            continue
        raise AssertionError("无效状态未被拒绝")
    print("通过：Piper 形状、单位契约和拒绝路径")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Piper π0.5 纯离线动作适配检查")
    parser.add_argument("--self-test", action="store_true", required=True)
    parser.parse_args()
    self_test()
