#!/usr/bin/env python3
"""把已保存 Episode 的左右从臂六关节反馈导出为只读 LeRobot v3 数据集。"""

import argparse
from bisect import bisect_right
import json
import math
from importlib.metadata import version
from pathlib import Path
import struct
import sys

import numpy as np
from rosbags.rosbag2 import Reader
from rosbags.typesys import get_types_from_msg

from to_lerobot import CAN_STATES, TYPESTORE, VECTOR_TYPE, source_digest, stamp_value


JOINT_IDS = (0x2A5, 0x2A6, 0x2A7)
DIRECT_TYPE = "piper_msgs/msg/PiperDirectCanFrame"
DIRECT_MSG = """std_msgs/Header header
uint32 can_id
uint8[8] raw_data
uint8 joint_index
float64[2] joint_position_rad
float64 gripper_position_m
float64 gripper_effort_nm
uint8 gripper_status
uint8 gripper_set_zero
"""
TYPESTORE.register(get_types_from_msg(DIRECT_MSG, DIRECT_TYPE))
JOINT_NAMES = tuple(f"joint{index}" for index in range(1, 7))
ARMS = ("left", "right")


def read_states(episode, max_can_skew_ns):
    metadata = json.loads((episode / "episode.json").read_text(encoding="utf-8"))
    if metadata.get("state") != "saved" or metadata.get("schema_version") not in (1, 2):
        raise ValueError("仅接受已保存的 Episode v1/v2")
    start = metadata.get("ready_at_unix_ns", metadata.get("started_at_unix_ns"))
    stop = metadata.get("stopped_at_unix_ns")
    if type(start) is not int or type(stop) is not int or not 0 < start < stop:
        raise ValueError("Episode 缺少有效的录制时间范围")
    rows = {arm: {} for arm in ARMS}
    raw_joints = {arm: {} for arm in ARMS}
    rejected = {arm: {} for arm in ARMS}
    with Reader(episode / "bag") as reader:
        by_topic = {connection.topic: connection for connection in reader.connections}
        for arm in ARMS:
            topic = CAN_STATES[arm]
            connection = by_topic.get(topic)
            if connection is None or connection.msgtype != VECTOR_TYPE:
                raise ValueError(f"缺少从臂向量话题或类型不符：{topic}")
            for _, _, raw in reader.messages(connections=[connection]):
                msg = TYPESTORE.deserialize_cdr(raw, VECTOR_TYPE)
                reason = None
                if (msg.header.frame_id != f"can_{arm}"
                        or msg.source_topic != f"/piper_direct/{arm}/slave_feedback"
                        or tuple(msg.source_can_ids[:3]) != JOINT_IDS):
                    reason = "来源不符"
                elif (tuple(msg.joint_names[:6]) != JOINT_NAMES
                      or len(msg.position) < 6
                      or not all(math.isfinite(value) for value in msg.position[:6])):
                    reason = "关节数值无效"
                elif any(msg.missing[:3]) or any(msg.stale[:3]):
                    reason = "关节来源缺失或过期"
                else:
                    try:
                        stamps = [stamp_value(value) for value in msg.source_stamps[:3]]
                    except ValueError:
                        reason = "关节来源时间无效"
                    else:
                        if (max(stamps) - min(stamps) > max_can_skew_ns
                                or not start <= min(stamps) <= max(stamps) <= stop):
                            reason = "关节来源时间超窗"
                if reason:
                    rejected[arm][reason] = rejected[arm].get(reason, 0) + 1
                    continue
                stamp_ns = max(stamps)
                position = np.asarray(msg.position[:6], dtype=np.float32)
                previous = rows[arm].get(stamp_ns)
                if previous is not None and (not np.array_equal(previous[0], position)
                                             or previous[1] != tuple(stamps)):
                    raise ValueError(f"{topic}: 同一来源 Header 对应冲突关节值")
                rows[arm][stamp_ns] = (position, tuple(stamps))
            direct = by_topic.get(f"/piper_direct/{arm}/slave_feedback")
            if direct is not None and direct.msgtype == DIRECT_TYPE:
                for _, _, raw in reader.messages(connections=[direct]):
                    msg = TYPESTORE.deserialize_cdr(raw, DIRECT_TYPE)
                    if msg.can_id not in JOINT_IDS:
                        continue
                    payload = bytes(msg.raw_data)
                    if len(payload) != 8:
                        continue
                    joint_stamp = stamp_value(msg.header.stamp)
                    values = struct.unpack(">ii", payload)
                    raw_joints[arm][(msg.can_id, joint_stamp)] = tuple(
                        value * math.pi / 180000 for value in values)
    streams = {arm: [(stamp, *value) for stamp, value in sorted(rows[arm].items())]
               for arm in ARMS}
    if any(len(streams[arm]) < 2 for arm in ARMS):
        raise ValueError("左右从臂均须至少有两条完整关节反馈")
    return metadata, streams, rejected, raw_joints


def align_states(streams, fps, max_age_ns, max_cross_arm_skew_ns):
    period = round(1_000_000_000 / fps)
    first = max(streams[arm][0][0] for arm in ARMS)
    last = min(streams[arm][-1][0] for arm in ARMS)
    if first >= last:
        raise ValueError("左右从臂来源时间没有重叠")
    stamps = {arm: [row[0] for row in streams[arm]] for arm in ARMS}
    selected = []
    skipped = {"缺少新鲜反馈": 0, "双臂时间差超窗": 0}
    for index in range((last - first) // period + 1):
        tick = first + index * period
        rows = []
        for arm in ARMS:
            position = bisect_right(stamps[arm], tick) - 1
            if position < 0 or tick - stamps[arm][position] > max_age_ns:
                skipped["缺少新鲜反馈"] += 1
                break
            rows.append(streams[arm][position])
        if len(rows) != 2:
            continue
        if abs(rows[0][0] - rows[1][0]) > max_cross_arm_skew_ns:
            skipped["双臂时间差超窗"] += 1
            continue
        selected.append((tick, rows[0], rows[1]))
    if len(selected) < 2:
        raise ValueError("有效双臂对齐状态少于两帧")
    return selected, skipped


def export(episode, output, repo_id, fps, max_age_ms, max_can_skew_ms,
           max_cross_arm_skew_ms):
    if output.exists():
        raise ValueError("输出目录已存在，拒绝覆盖")
    if version("lerobot") != "0.4.4":
        raise ValueError("此导出入口要求 lerobot==0.4.4（LeRobot v3）")
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    metadata, streams, rejected, raw_joints = read_states(
        episode, round(max_can_skew_ms * 1e6))
    selected, skipped = align_states(streams, fps, round(max_age_ms * 1e6),
                                     round(max_cross_arm_skew_ms * 1e6))
    names = [f"{arm}.{joint}" for arm in ARMS for joint in JOINT_NAMES]
    dataset = LeRobotDataset.create(
        repo_id=repo_id, fps=fps,
        features={"observation.state": {"dtype": "float32", "shape": (12,), "names": names}},
        root=output, robot_type="piper", use_videos=False)
    task = metadata.get("task") or "从臂关节状态只读验收"
    raw_verified = 0
    segment_lengths = []
    segment_frames = 0
    segment_index = 0
    previous_tick = None
    period = round(1_000_000_000 / fps)
    with (output / "source_timestamps.jsonl").open("w", encoding="utf-8") as stream:
        for index, (tick, left, right) in enumerate(selected):
            if previous_tick is not None and tick - previous_tick != period:
                dataset.save_episode()
                segment_lengths.append(segment_frames)
                segment_frames = 0
                segment_index += 1
            verified = True
            for arm, row in zip(ARMS, (left, right)):
                decoded = []
                for can_id, joint_stamp in zip(JOINT_IDS, row[2]):
                    values = raw_joints[arm].get((can_id, joint_stamp))
                    if values is None:
                        verified = False
                        break
                    decoded.extend(values)
                if len(decoded) == 6 and any(abs(a - b) > 1e-9
                                             for a, b in zip(decoded, row[1])):
                    raise ValueError(f"{arm}: 原始 CAN 与从臂向量关节值不一致")
            raw_verified += int(verified)
            state = np.concatenate((left[1], right[1]))
            dataset.add_frame({"task": task, "observation.state": state})
            stream.write(json.dumps({"global_frame_index": index,
                                     "episode_index": segment_index,
                                     "frame_index": segment_frames,
                                     "target_unix_ns": tick,
                                     "left_source_unix_ns": left[0],
                                     "right_source_unix_ns": right[0],
                                     "raw_can_frames_verified": verified},
                                    ensure_ascii=False) + "\n")
            segment_frames += 1
            previous_tick = tick
    dataset.save_episode()
    segment_lengths.append(segment_frames)
    dataset.finalize()
    check = LeRobotDataset(repo_id, root=output)
    if (check.meta.total_episodes != len(segment_lengths) or len(check) != len(selected)
            or tuple(check[0]["observation.state"].shape) != (12,)
            or not np.array_equal(check[0]["observation.state"].numpy(),
                                  np.concatenate((selected[0][1][1], selected[0][2][1])))):
        raise ValueError("LeRobot 回读帧数、维度或首帧状态不匹配")
    manifest = {"format": "LeRobotDataset v3.0", "lerobot_version": "0.4.4",
                "purpose": "follower_joint_state_readonly_validation",
                "training_eligible": False, "contains_action": False,
                "contains_gripper": False, "joint_names": names, "joint_unit": "rad",
                "source": str(episode.resolve()), "source_sha256": source_digest(episode),
                "episode_id": metadata.get("episode_id"), "fps": fps,
                "frames": len(selected), "episodes": len(segment_lengths),
                "segment_lengths": segment_lengths,
                "raw_can_verified_frames": raw_verified,
                "source_vector_count": {arm: len(streams[arm]) for arm in ARMS},
                "rejected_vectors": rejected, "skipped_ticks": skipped,
                "max_age_ms": max_age_ms, "max_can_skew_ms": max_can_skew_ms,
                "max_cross_arm_skew_ms": max_cross_arm_skew_ms}
    (output / "conversion_manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")
    return manifest


def main():
    parser = argparse.ArgumentParser(description="从臂六关节 rosbag2 → 只读 LeRobot v3")
    parser.add_argument("episode_dir", type=Path)
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("--repo-id", default="local/piper_follower_state_validation")
    parser.add_argument("--fps", type=int, default=10)
    parser.add_argument("--max-age-ms", type=float, default=100)
    parser.add_argument("--max-can-skew-ms", type=float, default=30)
    parser.add_argument("--max-cross-arm-skew-ms", type=float, default=50)
    args = parser.parse_args()
    if (args.fps <= 0 or any(not math.isfinite(value) or value <= 0 for value in
                             (args.max_age_ms, args.max_can_skew_ms,
                              args.max_cross_arm_skew_ms))):
        parser.error("fps 与时间阈值必须为正有限数")
    try:
        result = export(args.episode_dir, args.output_dir, args.repo_id, args.fps,
                        args.max_age_ms, args.max_can_skew_ms,
                        args.max_cross_arm_skew_ms)
    except (OSError, ValueError) as error:
        print(f"状态导出失败：{error}", file=sys.stderr)
        return 1
    print(json.dumps({"output": str(args.output_dir), "frames": result["frames"],
                      "training_eligible": False}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
