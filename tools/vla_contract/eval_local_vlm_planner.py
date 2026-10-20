#!/usr/bin/env python3
"""离线评估公开 VLM 的合成规划。运行：python3 tools/vla_contract/eval_local_vlm_planner.py --model /本地模型 --output /tmp/结果.json。"""

import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess
import tempfile
import threading
import time

from PIL import Image, ImageDraw

from local_vlm_planner import LocalVlmBackend
from long_sequence_planner import PlanError, ground_context, parse_plan, validate_context
from validate_contract import synthetic


OBJECTS = [("object-1", "红杯", "#cf3030"), ("object-2", "蓝杯", "#356bd5"),
           ("object-3", "绿杯", "#319458")]
REGIONS = [("region-1", "左盒"), ("region-2", "中盒"), ("region-3", "右盒")]


def cases():
    return [
        ("four_steps", "把红杯放到左盒，再把蓝杯放到右盒", 2, None,
         [("pick", "object-1", None), ("place", "object-1", "region-1"),
          ("pick", "object-2", None), ("place", "object-2", "region-3")]),
        ("three_steps_held", "把红杯放到左盒，再把蓝杯放到右盒", 2, "object-1",
         [("place", "object-1", "region-1"), ("pick", "object-2", None),
          ("place", "object-2", "region-3")]),
        ("five_steps_held", "把红杯放到左盒，再把蓝杯放到中盒，然后把绿杯放到右盒", 3, "object-1",
         [("place", "object-1", "region-1"), ("pick", "object-2", None),
          ("place", "object-2", "region-2"), ("pick", "object-3", None),
          ("place", "object-3", "region-3")]),
    ]


def make_case(root, case_id, instruction, count, held):
    image = Image.new("RGB", (640, 480), "#ece9df")
    draw = ImageDraw.Draw(image)
    draw.rectangle((0, 340, 640, 480), fill="#bca68b")
    for index, (_, _, color) in enumerate(OBJECTS[:count]):
        x = 70 + index * 130
        y = 190 if index == 0 and held else 285
        draw.rectangle((x, y, x + 50, y + 65), fill=color)
        draw.ellipse((x + 38, y + 16, x + 64, y + 45), outline=color, width=6)
    for index in range(3):
        x = 400 + index * 75
        draw.rectangle((x, 280, x + 65, 335), outline="#48565e", width=4)
    image.save(root / f"{case_id}.png")
    obs = synthetic()["observation"]
    now = time.time_ns()
    obs["observation_timestamp_ns"] = now
    for source in (*obs["images"].values(), *obs["state"].values()):
        source["header_timestamp_ns"] = now - 5_000_000
    obs["images"]["top_rgb"]["uri"] = f"{case_id}.png"
    return {
        "request_id": case_id, "instruction": instruction, "observation": obs,
        "scene_objects": [{"id": oid, "aliases": [name]} for oid, name, _ in OBJECTS[:count]],
        "scene_regions": [{"id": rid, "aliases": [name]} for rid, name in REGIONS],
        "available_skills": ["pick", "place"],
        "object_state": {oid: {"status": "held" if oid == held else "available", "region_id": None}
                         for oid, _, _ in OBJECTS[:count]},
        "completed_step_ids": [],
    }


def gpu_memory():
    try:
        output = subprocess.check_output(
            ["nvidia-smi", "--query-gpu=memory.used", "--format=csv,noheader,nounits"],
            stderr=subprocess.DEVNULL, text=True, timeout=3)
        return int(output.strip().splitlines()[0])
    except (OSError, ValueError, subprocess.TimeoutExpired):
        return None


def evaluate_one(backend, context, expected):
    grounded = ground_context(context)
    validate_context(grounded, time.time_ns(), 60_000_000_000)
    backend.last_raw = None
    backend.last_seconds = None
    baseline = gpu_memory()
    readings = []
    stop = threading.Event()

    def sample_gpu():
        while not stop.is_set():
            value = gpu_memory()
            if value is not None:
                readings.append(value)
            stop.wait(0.2)

    monitor = threading.Thread(target=sample_gpu, daemon=True)
    monitor.start()
    error = None
    try:
        raw = backend.generate(grounded)
    except Exception as exc:
        raw = backend.last_raw
        error = f"后端失败：{type(exc).__name__}"
    finally:
        stop.set()
        monitor.join(timeout=5)
    plan = None
    try:
        json_valid = type(json.loads(raw)) is dict
    except (TypeError, ValueError):
        json_valid = False
    if error is None:
        try:
            plan = parse_plan(raw, grounded)
        except (PlanError, TypeError, KeyError) as exc:
            error = str(exc)
    actual = ([(step["skill"], step["object_id"], step["region_id"])
               for step in plan["steps"]] if plan is not None else None)
    age_ms = (time.time_ns() - grounded["observation"]["observation_timestamp_ns"]) / 1e6
    return {
        "case_id": context["request_id"], "instruction": context["instruction"],
        "goal_state": grounded["goal_state"], "expected_steps": expected,
        "raw_model_output": raw, "json_valid": json_valid, "plan_valid": plan is not None,
        "semantic_correct": actual == expected if actual is not None else False,
        "actual_steps": actual, "reason": error,
        "model_seconds": backend.last_seconds, "observation_age_after_ms": round(age_ms, 2),
        "fresh_under_5s": age_ms <= 5000,
        "gpu_base_mib": baseline, "gpu_peak_mib": max(readings, default=baseline),
    }


def main():
    parser = argparse.ArgumentParser(description="只用合成场景评估本地 VLM；不连接机械臂")
    parser.add_argument("--model", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--timeout", type=float, default=120)
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0 or not args.model.is_dir():
        parser.error("模型目录和正数超时必需")
    with tempfile.TemporaryDirectory(prefix="piper-vlm-eval-") as directory:
        root = Path(directory)
        backend = LocalVlmBackend(args.model, root, args.timeout)
        results = []
        for case_id, instruction, count, held, expected in cases():
            context = make_case(root, case_id, instruction, count, held)
            results.append(evaluate_one(backend, context, expected))
    weight = args.model / "model.safetensors"
    weight_hash = None
    if weight.is_file():
        digest = hashlib.sha256()
        with weight.open("rb") as stream:
            for block in iter(lambda: stream.read(16 * 1024 * 1024), b""):
                digest.update(block)
        weight_hash = digest.hexdigest()
    count = len(results)
    valid = sum(x["plan_valid"] for x in results)
    correct = sum(x["semantic_correct"] for x in results)
    report = {
        "schema_version": "piper_vlm_plan_eval/1.0", "data_kind": "synthetic_only",
        "model_dir": str(args.model.resolve()), "model_weight_sha256": weight_hash,
        "results": results,
        "summary": {"cases": count, "json_valid": sum(x["json_valid"] for x in results),
                    "plan_valid": valid, "semantic_correct": correct,
                    "plan_valid_rate": round(valid / count, 3),
                    "semantic_correct_rate": round(correct / count, 3)},
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(report["summary"], ensure_ascii=False))


if __name__ == "__main__":
    main()
