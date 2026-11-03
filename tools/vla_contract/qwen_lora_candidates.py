#!/usr/bin/env python3
"""只读核验真实轨迹并导出 Qwen 规划器 LoRA 候选，不授予训练资格。"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import sqlite3

from runtime_trace import TraceError, read_trace


IMAGE_URI = re.compile(r"^rosbag2://(.+\.db3)#message=([1-9][0-9]*)$")
REVIEW_VERSION = "piper_qwen_lora_review/1.0"
CANDIDATE_VERSION = "piper_qwen_lora_candidate/1.0"


def _sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _load(path):
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        raise TraceError(f"无法读取核验文件：{path.name}") from exc
    if type(value) is not dict:
        raise TraceError(f"核验文件必须是 JSON 对象：{path.name}")
    return value


def _check_image(uri, bag_root):
    if type(uri) is not str:
        raise TraceError("图像 URI 无效")
    match = IMAGE_URI.fullmatch(uri)
    if match is None:
        raise TraceError("图像必须引用 rosbag2 SQLite 消息")
    path = Path(match.group(1)).resolve()
    if not path.is_relative_to(bag_root) or not path.is_file():
        raise TraceError("图像文件不属于绑定的 Episode bag")
    try:
        with sqlite3.connect(path.as_uri() + "?mode=ro", uri=True) as connection:
            row = connection.execute(
                "SELECT topics.type FROM messages JOIN topics ON messages.topic_id = topics.id "
                "WHERE messages.id = ?", (int(match.group(2)),)).fetchone()
    except sqlite3.Error as exc:
        raise TraceError("无法核对图像消息") from exc
    if row is None or row[0] != "sensor_msgs/msg/Image":
        raise TraceError("图像消息 ID 或类型不匹配")
    return _sha256(path)


def build_candidates(trace_dir, episode_manifest_path, review_path, recovery_manifest_path=None):
    """返回待人工后续验收的规划候选；全部输入仅读取。"""
    trace_dir = Path(trace_dir)
    episode_manifest_path = Path(episode_manifest_path)
    review_path = Path(review_path)
    manifest, events = read_trace(trace_dir)
    if manifest["source_kind"] != "real" or manifest["terminal_status"] != "DONE":
        raise TraceError("仅接受终态 DONE 的真实轨迹")
    episode = _load(episode_manifest_path)
    if (_sha256(episode_manifest_path) != manifest["episode_manifest_sha256"]
            or episode.get("episode_id") != manifest["links"].get("episode_id")
            or episode.get("state") != "saved"):
        raise TraceError("Episode 清单摘要、身份或保存状态不匹配")
    bag_relative = episode.get("rosbag_relative_path", "bag")
    if type(bag_relative) is not str:
        raise TraceError("Episode bag 路径无效")
    bag_root = (episode_manifest_path.parent / bag_relative).resolve()
    if (not bag_root.is_relative_to(episode_manifest_path.parent.resolve())
            or not bag_root.is_dir()):
        raise TraceError("Episode bag 路径无效")
    recovery_id = manifest["links"].get("recovery_id")
    recovery_sha256 = None
    if recovery_id is not None:
        if recovery_manifest_path is None:
            raise TraceError("外部恢复 ID 必须绑定恢复片段清单")
        recovery_manifest_path = Path(recovery_manifest_path)
        recovery = _load(recovery_manifest_path)
        if (recovery.get("recovery_id") != recovery_id
                or recovery.get("episode_id") != episode["episode_id"]
                or recovery.get("status") != "complete"
                or recovery.get("training_approved") is not False):
            raise TraceError("恢复片段清单身份或状态不匹配")
        recovery_sha256 = _sha256(recovery_manifest_path)
    elif recovery_manifest_path is not None:
        raise TraceError("轨迹没有外部恢复 ID")
    review = _load(review_path)
    if (review.get("schema_version") != REVIEW_VERSION
            or review.get("decision") != "approve_candidate"
            or review.get("run_id") != manifest["run_id"]
            or review.get("trace_manifest_sha256") != _sha256(trace_dir / "manifest.json")
            or review.get("episode_manifest_sha256") != manifest["episode_manifest_sha256"]
            or review.get("recovery_manifest_sha256") != recovery_sha256
            or type(review.get("reviewer_id")) is not str or not review["reviewer_id"].strip()
            or type(review.get("reviewed_at_unix_ns")) is not int
            or review["reviewed_at_unix_ns"] <= 0):
        raise TraceError("人工审核记录缺失或未绑定当前证据")
    selected = review.get("approved_plan_result_seqs")
    if (type(selected) is not list or not selected
            or any(type(seq) is not int for seq in selected) or len(set(selected)) != len(selected)):
        raise TraceError("审核的规划事件序号无效")
    by_seq = {event["seq"]: event for event in events}
    candidates, used_uris, image_digests = [], set(), {}
    for seq in selected:
        result = by_seq.get(seq)
        if result is None or result["event_type"] != "plan_result":
            raise TraceError("审核序号未指向规划结果")
        request = by_seq.get(seq - 1)
        if (request is None or request["event_type"] != "plan_request"
                or request["ids"].get("request_id") != result["ids"].get("request_id")):
            raise TraceError("规划输入与结果不能配对")
        accepted = by_seq.get(seq + 1)
        if (accepted is None or accepted["event_type"] != "plan_preflight"
                or accepted["data"].get("accepted") is not True):
            raise TraceError("规划结果未经通过预检")
        images = request["data"]["observation"]["images"]
        if type(images) is not dict or not images:
            raise TraceError("规划输入缺少图像")
        for frame in images.values():
            uri = frame["uri"]
            image_digests[uri] = _check_image(uri, bag_root)
            used_uris.add(uri)
        candidates.append({"plan_result_seq": seq, "input": request["data"],
                           "target_plan": result["data"]["plan"], "images": images})
    if review.get("verified_image_uris") != sorted(used_uris):
        raise TraceError("人工审核的图像引用与规划输入不一致")
    if review.get("verified_image_sha256") != image_digests:
        raise TraceError("人工审核的图像源文件摘要与当前文件不一致")
    return {"schema_version": CANDIDATE_VERSION, "status": "candidate_only",
            "training_eligible": False, "real_success_verified": False,
            "run_id": manifest["run_id"], "episode_id": episode["episode_id"],
            "recovery_id": recovery_id, "recovery_manifest_sha256": recovery_sha256,
            "trace_manifest_sha256": _sha256(trace_dir / "manifest.json"),
            "episode_manifest_sha256": manifest["episode_manifest_sha256"],
            "review_sha256": _sha256(review_path), "reviewer_id": review["reviewer_id"],
            "samples": candidates}


def export_candidates(trace_dir, episode_manifest_path, review_path, output_path,
                      recovery_manifest_path=None):
    candidates = build_candidates(trace_dir, episode_manifest_path, review_path,
                                  recovery_manifest_path)
    output_path = Path(output_path)
    payload = (json.dumps(candidates, ensure_ascii=False, indent=2, allow_nan=False) + "\n").encode("utf-8")
    descriptor = os.open(output_path, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
    except Exception:
        output_path.unlink(missing_ok=True)
        raise
    return candidates


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="核验真实轨迹并导出 Qwen 规划器 LoRA 候选")
    parser.add_argument("trace_dir")
    parser.add_argument("episode_manifest")
    parser.add_argument("review")
    parser.add_argument("output")
    parser.add_argument("--recovery-manifest")
    args = parser.parse_args()
    export_candidates(args.trace_dir, args.episode_manifest, args.review, args.output,
                      args.recovery_manifest)
