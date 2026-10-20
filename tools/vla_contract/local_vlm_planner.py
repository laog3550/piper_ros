#!/usr/bin/env python3
"""本地公开 VLM 规划 smoke。运行：python3 tools/vla_contract/local_vlm_planner.py context.json --model /本地模型 --image-root /图像目录。"""

import argparse
import json
import math
from pathlib import Path
import subprocess
import sys
import time

from long_sequence_planner import LongSequencePlanner, PlanError


class LocalVlmBackend:
    def __init__(self, model, image_root, timeout=120):
        self.model = Path(model).resolve()
        self.image_root = Path(image_root).resolve()
        self.timeout = timeout
        self.last_raw = None
        self.last_seconds = None

    def generate(self, context):
        uri = context["observation"]["images"]["top_rgb"]["uri"]
        if "://" in uri:
            raise PlanError("模型输入必须是不可变的本地图像")
        image = (self.image_root / uri).resolve()
        if not image.is_relative_to(self.image_root) or not image.is_file() or not self.model.is_dir():
            raise PlanError("图像或模型目录无效")
        from PIL import Image
        try:
            with Image.open(image) as frame:
                expected = context["observation"]["images"]["top_rgb"]
                if frame.mode != "RGB" or frame.size != (expected["width"], expected["height"]):
                    raise PlanError("顶部图像格式或尺寸与观测不符")
                frame.verify()
        except OSError as exc:
            raise PlanError("顶部图像无法读取") from exc
        payload = {"context": context, "model": str(self.model), "image": str(image)}
        start = time.monotonic()
        try:
            run = subprocess.run([sys.executable, str(Path(__file__).resolve()), "--worker"],
                                 input=json.dumps(payload, ensure_ascii=False), text=True,
                                 capture_output=True, timeout=self.timeout, check=True)
        except (subprocess.TimeoutExpired, subprocess.CalledProcessError, OSError) as exc:
            raise PlanError(f"本地模型调用失败：{type(exc).__name__}") from exc
        self.last_seconds = time.monotonic() - start
        self.last_raw = run.stdout.strip()
        return self.last_raw


def worker(payload):
    import torch
    from transformers import AutoModelForImageTextToText, AutoProcessor

    context = payload["context"]
    processor = AutoProcessor.from_pretrained(payload["model"], local_files_only=True)
    model = AutoModelForImageTextToText.from_pretrained(
        payload["model"], local_files_only=True, dtype="auto", device_map="auto").eval()
    held_ids = [object_id for object_id in context["goal_state"]
                if context["object_state"][object_id]["status"] == "held"]
    required_steps = sum(1 if object_id in held_ids else 2 for object_id in context["goal_state"])
    prompt = (
        "根据图像与已确认的结构化目标生成完整 pick/place 计划。只允许使用提供的 ID，不得发明目标或运动命令。"
        "必须覆盖 goal_state 中每个物体；按 instruction 的顺序执行，最多 5 步。"
        "当前 status=held 的物体直接 place，status=available 的物体先 pick 再 place。"
        "仅输出一个严格 JSON 对象，不要 Markdown、代码或解释。根字段恰为 "
        "schema_version, request_id, observation_id, steps；schema_version 固定 piper_vlm_plan/1.0。"
        "每步字段恰为 step_id, skill, arm, object_id, region_id, depends_on, completion。"
        "step_id 依次用 s1、s2...；首步 depends_on=[]，后续 depends_on=[前一步 step_id]。"
        "pick 的 region_id=null，completion={type:object_held,object_id:同一步物体,region_id:null}；"
        "place 的 region_id=目标区域，completion={type:object_in_region,object_id:同一步物体,region_id:目标区域}。"
        f"规划输入：{json.dumps({key: value for key, value in context.items() if key != 'observation'}, ensure_ascii=False)}。"
        f"观测摘要：{json.dumps({'observation_id': context['observation']['observation_id'], 'arm_mode': context['observation']['arm_mode'], 'observation_timestamp_ns': context['observation']['observation_timestamp_ns'], 'state': context['observation']['state']}, ensure_ascii=False)}。"
        f"最终核对：已经夹持的物体 ID 为 {json.dumps(held_ids, ensure_ascii=False)}，对它们第一步只能 place，绝不能再次 pick；"
        f"本次计划必须恰好 {required_steps} 步，每个 goal_state 目标恰好 place 一次，不得重复动作。"
    )
    messages = [{"role": "system", "content": [{"type": "text", "text": "你是只输出 JSON 的离线技能规划器；无法确定时输出 UNKNOWN，不控制机械臂。"}]},
                {"role": "user", "content": [{"type": "image", "path": payload["image"]},
                                               {"type": "text", "text": prompt}]}]
    inputs = processor.apply_chat_template(messages, add_generation_prompt=True, tokenize=True,
                                           return_dict=True, return_tensors="pt").to(model.device)
    with torch.inference_mode():
        tokens = model.generate(**inputs, max_new_tokens=1024, do_sample=False)
    print(processor.batch_decode(tokens[:, inputs.input_ids.shape[-1]:], skip_special_tokens=True)[0])


def main():
    parser = argparse.ArgumentParser(description="本地公开 VLM 离线规划 smoke；不连接控制设备")
    parser.add_argument("context", nargs="?", type=Path)
    parser.add_argument("--model")
    parser.add_argument("--image-root")
    parser.add_argument("--timeout", type=float, default=120)
    parser.add_argument("--max-age-ms", type=int, default=50, help="仅离线 smoke 的观测最大年龄")
    parser.add_argument("--worker", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.worker:
        worker(json.load(sys.stdin))
        return
    if args.context is None or not args.model or not args.image_root or not math.isfinite(args.timeout) or args.timeout <= 0 or not 0 < args.max_age_ms <= 5000:
        parser.error("需要 context、--model、--image-root 和正数 --timeout")
    context = json.loads(args.context.read_text(encoding="utf-8"))
    backend = LocalVlmBackend(args.model, args.image_root, args.timeout)
    planner = LongSequencePlanner(backend, max_age_ns=args.max_age_ms * 1_000_000,
                                  now_provider=time.time_ns)
    try:
        plan = planner.plan(context, time.time_ns())
        result = {"status": "VALID_PLAN", "plan": plan}
    except PlanError as exc:
        status = "REOBSERVE" if "观测" in str(exc) and "过期" in str(exc) else "HUMAN_REVIEW"
        result = {"status": status, "reason": str(exc)}
    result.update(model_seconds=backend.last_seconds, raw_model_output=backend.last_raw)
    print(json.dumps(result, ensure_ascii=False))


if __name__ == "__main__":
    main()
