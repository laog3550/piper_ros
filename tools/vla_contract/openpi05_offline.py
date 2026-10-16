#!/usr/bin/env python3
"""检查 Piper RGB 离线输入并运行官方 π0.5-DROID smoke；用法：python3 tools/vla_contract/openpi05_offline.py --help。"""

import argparse
import json
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

from validate_contract import validate_goal, validate_observation


OPENPI_COMMIT = "215abfb217dbac7d5f1273282331b9b1866c0479"
CONFIG = "pi05_droid"
RAW_ACTION_SPACE = "droid_joint_velocity_7_gripper_position_1"


def preflight(data, *, now_ns, max_age_ns, max_skew_ns, image_dir):
    if set(data) != {"observation", "skill_goal"}:
        raise ValueError("输入仅允许 observation 与 skill_goal")
    obs, goal = data["observation"], data["skill_goal"]
    validate_observation(obs)
    validate_goal(goal, obs)
    if obs["arm_mode"] == "BIMANUAL":
        raise ValueError("官方 DROID 基线仅有一个腕部 RGB 槽位；双臂输入拒绝")
    if type(now_ns) is not int or type(max_age_ns) is not int or max_age_ns <= 0:
        raise ValueError("时间与最大年龄必须是有效的整数纳秒")
    age_ns = now_ns - obs["observation_timestamp_ns"]
    if not 0 <= age_ns <= max_age_ns:
        raise ValueError(f"观测过期或位于未来：age_ns={age_ns}")
    stamps = [frame["header_timestamp_ns"] for frame in obs["images"].values()]
    stamps += [state["header_timestamp_ns"] for state in obs["state"].values()]
    if type(max_skew_ns) is not int or max_skew_ns < 0 or max(stamps) - min(stamps) > max_skew_ns:
        raise ValueError("RGB 与关节状态 Header 时间偏差超限")
    from PIL import Image

    resolved = {}
    for name, frame in obs["images"].items():
        uri = frame["uri"]
        if "://" in uri:
            raise ValueError(f"{name}: 仅接受本地离线图像路径")
        path = (image_dir / uri).resolve()
        if not path.is_relative_to(image_dir.resolve()):
            raise ValueError(f"{name}: 图像路径越过指定目录")
        with Image.open(path) as image:
            if image.mode != "RGB" or image.size != (640, 480):
                raise ValueError(f"{name}: 文件必须是 640×480 RGB")
            image.verify()
        resolved[name] = str(path)
    return {
        "status": "input_validated_bridge_required",
        "observation_id": obs["observation_id"],
        "age_ns": age_ns,
        "source_skew_ns": max(stamps) - min(stamps),
        "rgb_files": resolved,
        "official_config": CONFIG,
        "official_commit": OPENPI_COMMIT,
        "official_output_space": RAW_ACTION_SPACE,
        "contract_action_space": "absolute_joint_position_7",
        "missing_bridge": ["Piper 6 关节+夹爪至 DROID 7 关节+夹爪的有标定状态映射",
                           "DROID 关节速度至 Piper 绝对关节位置的安全转换与时间标定",
                           "Piper 示教归一化统计和适配后的 checkpoint"],
    }


def smoke(openpi_root, checkpoint):
    root = openpi_root.resolve()
    revision = subprocess.check_output(["git", "-C", str(root), "rev-parse", "HEAD"], text=True).strip()
    if revision != OPENPI_COMMIT:
        raise ValueError(f"openpi commit 不匹配：{revision}")
    if not checkpoint.is_dir() or not (checkpoint / "assets").is_dir() or not (checkpoint / "params").exists():
        raise ValueError("需要完整本地 pi05_droid checkpoint（assets 与 params）")
    sys.path.insert(0, str(root / "src"))
    import jax
    import numpy as np
    from openpi.policies import droid_policy, policy_config
    from openpi.training import config

    if jax.default_backend() != "gpu":
        raise ValueError(f"JAX 未使用 GPU：{jax.devices()}")

    peak_mib = [0]
    stop = threading.Event()

    def watch_gpu():
        while not stop.is_set():
            result = subprocess.run(
                ["nvidia-smi", "--query-compute-apps=pid,used_gpu_memory", "--format=csv,noheader,nounits"],
                capture_output=True, text=True, check=False,
            )
            for line in result.stdout.splitlines():
                parts = [part.strip() for part in line.split(",")]
                if len(parts) == 2 and parts[0] == str(os.getpid()) and parts[1].isdigit():
                    peak_mib[0] = max(peak_mib[0], int(parts[1]))
            stop.wait(0.2)

    watcher = threading.Thread(target=watch_gpu, daemon=True)
    watcher.start()
    try:
        load_start = time.monotonic()
        policy = policy_config.create_trained_policy(config.get_config(CONFIG), checkpoint)
        load_ms = (time.monotonic() - load_start) * 1000
        example = droid_policy.make_droid_example()
        timings = []
        for _ in range(3):
            started = time.monotonic()
            output = policy.infer(example)
            timings.append((time.monotonic() - started) * 1000)
            actions = np.asarray(output["actions"])
            if actions.shape != (15, 8) or not np.isfinite(actions).all():
                raise ValueError(f"官方输出形状或有限性异常：{actions.shape}")
    finally:
        stop.set()
        watcher.join(timeout=2)
    return {"status": "official_smoke_only", "official_commit": revision, "config": CONFIG,
            "jax_device": str(jax.devices()[0]),
            "output_shape": list(actions.shape), "output_space": RAW_ACTION_SPACE,
            "load_ms": round(load_ms, 1), "first_infer_ms": round(timings[0], 1),
            "warm_infer_ms": [round(value, 1) for value in timings[1:]],
            "gpu_peak_mib_process": peak_mib[0] or None,
            "note": "随机 DROID 输入仅验证官方推理；输出不属于 Piper action_chunk"}


def main():
    parser = argparse.ArgumentParser(description="离线输入预检查或官方 π0.5-DROID smoke；不产生 Piper 目标")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--preflight", type=Path, help="含 observation 与 skill_goal 的 JSON")
    group.add_argument("--smoke", action="store_true", help="运行官方随机 DROID 样例")
    parser.add_argument("--image-dir", type=Path, help="图像文件根目录")
    parser.add_argument("--now-ns", type=int, help="同一 unix_ns 时间域的检查时刻")
    parser.add_argument("--max-age-ns", type=int, help="最大观测年龄，必须显式指定")
    parser.add_argument("--max-skew-ns", type=int, help="最大源间 Header 偏差，必须显式指定")
    parser.add_argument("--openpi-root", type=Path, help="指定 commit 的官方 openpi checkout")
    parser.add_argument("--checkpoint", type=Path, help="完整本地 pi05_droid checkpoint")
    args = parser.parse_args()
    if args.preflight:
        if None in (args.image_dir, args.now_ns, args.max_age_ns, args.max_skew_ns):
            parser.error("--preflight 需要 --image-dir、--now-ns、--max-age-ns、--max-skew-ns")
        result = preflight(json.loads(args.preflight.read_text(encoding="utf-8")),
                           now_ns=args.now_ns, max_age_ns=args.max_age_ns,
                           max_skew_ns=args.max_skew_ns, image_dir=args.image_dir)
    else:
        if args.openpi_root is None or args.checkpoint is None:
            parser.error("--smoke 需要 --openpi-root 与 --checkpoint")
        result = smoke(args.openpi_root, args.checkpoint)
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    try:
        main()
    except (ValueError, FileNotFoundError, ImportError) as error:
        print(f"拒绝：{error}", file=sys.stderr)
        sys.exit(1)
