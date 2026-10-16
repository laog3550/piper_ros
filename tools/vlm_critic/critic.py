#!/usr/bin/env python3
"""离线 pick/place 技能边界评估。运行：python3 tools/vlm_critic/critic.py sample.json [--model /本地模型目录]。"""

import argparse
import json
import math
import subprocess
import sys
from pathlib import Path


VERSION = "piper_vlm_critic/1.0"
STATUSES = {"SUCCESS", "FAILURE", "UNKNOWN"}
FAILURES = {"NONE", "EMPTY_GRASP", "SLIP", "PLACE_MISS", "WRONG_OBJECT", "COLLISION_RISK", "OCCLUDED", "DEPTH_INVALID", "OUT_OF_WORKSPACE", "TIMEOUT", "UNKNOWN"}
SOURCES = {"geometry", "pre_rgb", "post_rgb", "system"}


def verdict(sample, status="UNKNOWN", failure="UNKNOWN", confidence=0.0,
            evidence=None, backend="rules"):
    return {
        "schema_version": VERSION,
        "sample_id": sample.get("sample_id", "invalid-input"),
        "skill": f"{sample.get('arm_mode', 'unknown').lower()}_{sample.get('skill_id', 'unknown')}",
        "target_id": sample.get("target_id", "unknown"),
        "result": status,
        "failure_type": failure,
        "confidence": confidence,
        "evidence": evidence or [{"source": "system", "observation": "证据不足，需人工复核"}],
        "backend": backend,
        "recommended_transition": "HUMAN_REVIEW",
        "frame_refs": {"pre_rgb": sample.get("pre_rgb", ""), "post_rgb": sample.get("post_rgb", "")},
        "calibration_id": sample.get("calibration_id", "unknown"),
    }


def validate_output(value, sample):
    if type(value) is not dict or set(value) != {
        "schema_version", "sample_id", "skill", "target_id", "result", "failure_type",
        "confidence", "evidence", "backend", "recommended_transition", "frame_refs", "calibration_id",
    }:
        raise ValueError("输出字段不符合契约")
    if (value["schema_version"] != VERSION or value["sample_id"] != sample["sample_id"]
            or value["skill"] != f"{sample['arm_mode'].lower()}_{sample['skill_id']}"
            or value["target_id"] != sample["target_id"] or value["calibration_id"] != sample["calibration_id"]
            or value["frame_refs"] != {"pre_rgb": sample["pre_rgb"], "post_rgb": sample["post_rgb"]}):
        raise ValueError("输出引用不匹配")
    if value["result"] not in STATUSES or value["failure_type"] not in FAILURES:
        raise ValueError("输出枚举无效")
    if (value["result"] == "SUCCESS" and value["failure_type"] != "NONE") or (value["result"] == "UNKNOWN" and value["failure_type"] != "UNKNOWN") or (value["result"] == "FAILURE" and value["failure_type"] in {"NONE", "UNKNOWN"}):
        raise ValueError("状态与失败类型冲突")
    allowed = ({"EMPTY_GRASP", "SLIP", "WRONG_OBJECT", "COLLISION_RISK", "OCCLUDED", "DEPTH_INVALID", "OUT_OF_WORKSPACE", "TIMEOUT"}
               if sample["skill_id"] == "pick" else {"PLACE_MISS", "WRONG_OBJECT", "COLLISION_RISK", "OCCLUDED", "DEPTH_INVALID", "OUT_OF_WORKSPACE", "TIMEOUT"})
    if value["result"] == "FAILURE" and value["failure_type"] not in allowed:
        raise ValueError("失败类型与技能不匹配")
    c = value["confidence"]
    if type(c) not in (int, float) or not math.isfinite(c) or not 0 <= c <= 1 or (value["result"] == "UNKNOWN" and c != 0):
        raise ValueError("置信度无效")
    if type(value["backend"]) is not str or not value["backend"] or value["recommended_transition"] != "HUMAN_REVIEW":
        raise ValueError("后端或人工接管字段无效")
    ev = value["evidence"]
    if type(ev) is not list or not ev or len(ev) > 8:
        raise ValueError("证据列表无效")
    for item in ev:
        if type(item) is not dict or set(item) != {"source", "observation"} or item["source"] not in SOURCES or type(item["observation"]) is not str or not item["observation"].strip() or len(item["observation"]) > 300:
            raise ValueError("证据字段无效")
    return value


def validate_input(sample, base):
    if type(sample) is not dict or set(sample) != {"sample_id", "skill_id", "arm_mode", "target_id", "calibration_id", "pre_rgb", "post_rgb", "geometry"}:
        raise ValueError("输入字段不符合契约")
    for key in ("sample_id", "target_id", "calibration_id"):
        if type(sample[key]) is not str or not sample[key].strip() or len(sample[key]) > 128:
            raise ValueError(f"{key} 无效")
    if sample["skill_id"] not in {"pick", "place"}:
        raise ValueError("skill_id 无效")
    if sample["arm_mode"] not in {"LEFT", "RIGHT"}:
        raise ValueError("arm_mode 无效")
    from PIL import Image
    for key in ("pre_rgb", "post_rgb"):
        path = (base / sample[key]).resolve()
        if not path.is_file():
            raise ValueError(f"{key} 图像不存在")
        with Image.open(path) as im:
            im.verify()
        with Image.open(path) as im:
            if im.mode != "RGB" or im.width < 1 or im.height < 1:
                raise ValueError(f"{key} 必须为 RGB")
        sample[key] = str(path)
    geometry = sample["geometry"]
    if type(geometry) is not dict or set(geometry) != {"pre_in_gripper", "post_in_gripper", "post_in_region", "post_target_visible"}:
        raise ValueError("geometry 字段无效")
    if any(v is not None and type(v) is not bool for v in geometry.values()):
        raise ValueError("geometry 只允许 bool/null")
    return sample


def rule_result(sample):
    g = sample["geometry"]
    if sample["skill_id"] == "pick":
        if g["pre_in_gripper"] is False and g["post_in_gripper"] is True:
            return verdict(sample, "SUCCESS", "NONE", 1.0, [{"source": "geometry", "observation": "前帧未夹持，后帧几何摘要确认夹持目标"}])
        if g["post_in_gripper"] is False and g["post_target_visible"] is True:
            return verdict(sample, "FAILURE", "EMPTY_GRASP", 1.0, [{"source": "geometry", "observation": "后帧未夹持且目标仍可见"}])
        if g["pre_in_gripper"] is True and g["post_in_gripper"] is False:
            return verdict(sample, "FAILURE", "SLIP", 1.0, [{"source": "geometry", "observation": "夹持状态由真变假"}])
    else:
        if g["post_in_region"] is True and g["post_in_gripper"] is False:
            return verdict(sample, "SUCCESS", "NONE", 1.0, [{"source": "geometry", "observation": "目标位于指定区域且夹爪未持有目标"}])
        if g["post_in_region"] is False and g["post_in_gripper"] is True:
            return verdict(sample, "FAILURE", "PLACE_MISS", 1.0, [{"source": "geometry", "observation": "目标未进入指定区域且仍被夹持"}])
    return verdict(sample)


def parse_model(raw, sample):
    result = json.loads(raw)
    if type(result) is not dict or set(result) != {"result", "failure_type", "confidence", "evidence"}:
        raise ValueError("模型 JSON 字段无效")
    output = verdict(sample, backend="vlm_zero_shot")
    output.update(result)
    validate_output(output, sample)
    if any(item["source"] not in {"pre_rgb", "post_rgb"} for item in result["evidence"]):
        raise ValueError("模型证据来源无效")
    return output


def worker(payload):
    import torch
    from transformers import AutoModelForImageTextToText, AutoProcessor

    model_path = payload["model"]
    sample = payload["sample"]
    processor = AutoProcessor.from_pretrained(model_path, local_files_only=True)
    model = AutoModelForImageTextToText.from_pretrained(
        model_path, local_files_only=True, dtype="auto", device_map="auto"
    ).eval()
    allowed = "EMPTY_GRASP, SLIP" if sample["skill_id"] == "pick" else "PLACE_MISS"
    prompt = (f"Did the robot {sample['skill_id']} target {sample['target_id']}? "
              "Compare Before and After. If unsure, choose UNKNOWN. "
              "Reply with ONLY valid JSON, no markdown or explanation. "
              "Use exactly these keys: result, failure_type, confidence, evidence. "
              f"result is SUCCESS, FAILURE, or UNKNOWN. failure_type is NONE, UNKNOWN, or {allowed}. "
              "For UNKNOWN, confidence must be 0. evidence is an array of objects with exactly "
              "source (pre_rgb or post_rgb) and observation (one visible fact). "
              "Example JSON shape: {\"result\":\"UNKNOWN\",\"failure_type\":\"UNKNOWN\","
              "\"confidence\":0,\"evidence\":[{\"source\":\"post_rgb\",\"observation\":\"target uncertain\"}]}.")
    messages = [{"role": "user", "content": [
        {"type": "text", "text": "Before"}, {"type": "image", "path": sample["pre_rgb"]},
        {"type": "text", "text": "After"}, {"type": "image", "path": sample["post_rgb"]},
        {"type": "text", "text": prompt},
    ]}]
    inputs = processor.apply_chat_template(messages, add_generation_prompt=True, tokenize=True,
                                           return_dict=True, return_tensors="pt").to(model.device)
    with torch.inference_mode():
        ids = model.generate(**inputs, max_new_tokens=256, do_sample=False)
    raw = processor.batch_decode(ids[:, inputs.input_ids.shape[-1]:], skip_special_tokens=True)[0]
    print(raw)


def evaluate(sample, model_path, timeout, min_confidence=0.7):
    rule = rule_result(sample)
    if rule["result"] != "UNKNOWN" or model_path is None:
        return rule
    try:
        if not Path(model_path).is_dir():
            raise ValueError("本地模型目录不存在")
        run = subprocess.run([sys.executable, str(Path(__file__).resolve()), "--worker"],
                             input=json.dumps({"model": str(Path(model_path).resolve()), "sample": sample}),
                             text=True, capture_output=True, timeout=timeout, check=True)
        output = parse_model(run.stdout.strip(), sample)
        if output["result"] != "UNKNOWN" and output["confidence"] < min_confidence:
            return verdict(sample, evidence=[{"source": "system", "observation": "模型自报置信度低于配置阈值"}], backend="vlm_zero_shot")
        return output
    except (ValueError, TypeError, KeyError, subprocess.TimeoutExpired, subprocess.CalledProcessError, OSError) as exc:
        return verdict(sample, evidence=[{"source": "system", "observation": f"模型不可用、超时或输出无效：{type(exc).__name__}"}], backend="vlm_zero_shot")


def main():
    parser = argparse.ArgumentParser(description="离线技能评估；只读图像，不连接机械臂")
    parser.add_argument("file", nargs="?", type=Path)
    parser.add_argument("--model", help="已下载的 VLM 本地目录（如 SmolVLM 或 Qwen2.5-VL）")
    parser.add_argument("--timeout", type=float, default=120)
    parser.add_argument("--min-confidence", type=float, default=0.7)
    parser.add_argument("--worker", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.worker:
        worker(json.load(sys.stdin))
        return
    if args.file is None or not math.isfinite(args.timeout) or args.timeout <= 0 or not math.isfinite(args.min_confidence) or not 0 <= args.min_confidence <= 1:
        parser.error("需要输入 JSON 文件、正数 --timeout 与 [0,1] 的 --min-confidence")
    try:
        sample = json.loads(args.file.read_text())
        sample = validate_input(sample, args.file.resolve().parent)
        output = validate_output(evaluate(sample, args.model, args.timeout, args.min_confidence), sample)
    except (OSError, ValueError, TypeError, KeyError, json.JSONDecodeError) as exc:
        parser.error(str(exc))
    print(json.dumps(output, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
