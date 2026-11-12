#!/usr/bin/env python3
"""只读审计真实 bag 的六关节字节映射、向量来源和主从近时对应；不授予现场映射资格。"""

import argparse
from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg


ROOT = Path(__file__).resolve().parents[2]
KINDS = {"master_target": (0x155, 0x156, 0x157),
         "slave_feedback": (0x2A5, 0x2A6, 0x2A7)}
SCALE = math.pi / 180000


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def stamp(value):
    return int(value.sec) * 1_000_000_000 + int(value.nanosec)


def quantiles(values):
    if not len(values):
        return None
    return dict(zip(("min", "p50", "p95", "max"),
                    map(float, np.quantile(values, (0, .5, .95, 1)))))


def correlation(x, y):
    if len(x) < 3 or np.std(x) < 1e-9 or np.std(y) < 1e-9:
        return None
    return float(np.corrcoef(x, y)[0, 1])


def nearest_indices(source, destination):
    after = np.minimum(np.searchsorted(destination, source), len(destination) - 1)
    before = np.maximum(after - 1, 0)
    return np.where(abs(destination[before] - source) <= abs(destination[after] - source),
                    before, after)


def paired(target, feedback, tolerance_ns):
    if not target or not feedback:
        return {"pairs": 0, "target_frames": len(target)}
    target = sorted(target)
    feedback = sorted(feedback)
    tx = np.array([r[0] for r in target], dtype=np.int64)
    ty = np.array([r[0] for r in feedback], dtype=np.int64)
    vx = np.array([r[1] for r in target])
    vy = np.array([r[1] for r in feedback])
    match = nearest_indices(tx, ty)
    delta = ty[match] - tx
    keep = abs(delta) <= tolerance_ns
    x, y = vx[keep], vy[match[keep]]
    result = {"target_frames": len(target), "pairs": int(sum(keep)),
              "unique_feedback_frames": len(set(match[keep].tolist())),
              "feedback_minus_target_ms": quantiles(delta[keep] / 1e6)}
    result["joints"] = []
    for j in range(2):
        a, b = x[:, j], y[:, j]
        slope = float(np.dot(a - a.mean(), b - b.mean()) / np.sum((a - a.mean()) ** 2)) if len(a) and np.std(a) > 1e-9 else None
        result["joints"].append({"target_range_deg": quantiles(vx[:, j] * 180 / math.pi),
                                 "feedback_range_deg": quantiles(vy[:, j] * 180 / math.pi),
                                 "pearson": correlation(a, b), "ols_feedback_per_target": slope,
                                 "feedback_minus_target_deg": quantiles((b - a) * 180 / math.pi),
                                 "absolute_error_deg": quantiles(abs(b - a) * 180 / math.pi)})
    return result


def audit(bag, tolerance_ns):
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    for name in ("PiperDirectCanFrame", "PiperArmVector"):
        definition = (ROOT / "src/piper_msgs/msg" / (name + ".msg")).read_text()
        typestore.register(get_types_from_msg(definition, "piper_msgs/msg/" + name))
    streams = defaultdict(list)
    vectors = defaultdict(list)
    errors = Counter()
    topic_counts = Counter()
    source_lookup = {}
    duplicate_keys = conflicting_keys = 0
    with Reader(bag) as reader:
        diagnostics = {f"/piper_direct/{side}/diagnostics" for side in ("left", "right")}
        connections = [c for c in reader.connections
                       if c.topic.startswith(("/piper_direct/", "/piper_vector/")) and c.topic not in diagnostics]
        for connection, received, raw in reader.messages(connections=connections):
            message = typestore.deserialize_cdr(raw, connection.msgtype)
            topic = connection.topic
            topic_counts[topic] += 1
            _, prefix, side, kind = topic.split("/")
            if side not in ("left", "right") or kind not in KINDS:
                errors["unexpected_topic"] += 1
                continue
            if prefix == "piper_vector":
                vectors[(side, kind)].append((message, received))
                continue
            can_id = int(message.can_id)
            if can_id not in KINDS[kind]:
                continue  # 夹爪已有独立验收，本工具仅审计六关节。
            index = KINDS[kind].index(can_id)
            payload = bytes(message.raw_data)
            timestamp = stamp(message.header.stamp)
            value = tuple(int.from_bytes(payload[i:i + 4], "big", signed=True) * SCALE for i in (0, 4))
            if len(payload) != 8 or message.joint_index != index * 2 + 1:
                errors["raw_length_or_joint_index"] += 1
            if message.header.frame_id != "can_" + side:
                errors["direct_frame_id"] += 1
            if timestamp <= 0 or not 0 <= message.header.stamp.nanosec < 1_000_000_000:
                errors["direct_timestamp"] += 1
            if not np.allclose(value, message.joint_position_rad, atol=1e-12, rtol=0):
                errors["raw_to_ros_rad"] += 1
            key = (side, kind, can_id, timestamp)
            if key in source_lookup:
                duplicate_keys += 1
                conflicting_keys += source_lookup[key] != value
            source_lookup[key] = value
            streams[(side, kind, can_id)].append((timestamp, value, received, payload.hex()))
    stream_stats = {}
    for (side, kind, can_id), rows in sorted(streams.items()):
        timestamps = np.array([r[0] for r in rows], dtype=np.int64)
        degrees = np.array([r[1] for r in rows]) * 180 / math.pi
        differences = np.diff(timestamps)
        stream_stats[f"{side}/{kind}/{can_id:#x}"] = {
            "frames": len(rows), "first_header_ns": int(timestamps[0]), "last_header_ns": int(timestamps[-1]),
            "first_raw_hex": rows[0][3], "first_position_rad": list(rows[0][1]),
            "joint_range_deg": [quantiles(degrees[:, j]) for j in range(2)],
            "header_backwards": int(sum(differences < 0)), "header_equal": int(sum(differences == 0)),
            "interval_ms": quantiles(differences / 1e6),
            "receive_minus_header_ms": quantiles(np.array([r[2] - r[0] for r in rows]) / 1e6)}
    source_bounds = {key: (min(r[0] for r in rows), max(r[0] for r in rows))
                     for key, rows in streams.items() if rows}
    vector_stats = {}
    for (side, kind), rows in sorted(vectors.items()):
        stats = Counter(messages=len(rows))
        skews = []
        previous = None
        for message, _ in rows:
            ids = list(message.source_can_ids)
            sources = [stamp(s) for s in message.source_stamps]
            header = stamp(message.header.stamp)
            stats["header_backwards"] += previous is not None and header < previous
            stats["header_equal"] += previous == header
            previous = header
            if (ids != [*KINDS[kind], 0x159 if kind == "master_target" else 0x2A8]
                    or message.source_topic != f"/piper_direct/{side}/{kind}"
                    or message.header.frame_id != "can_" + side
                    or list(message.joint_names) != [f"joint{j}" for j in range(1, 7)] + ["gripper"]):
                stats["identity_or_order_error"] += 1
            stats["header_max_source_error"] += header != max(sources)
            stats["joint_missing"] += any(message.missing[:3])
            stats["joint_stale"] += any(message.stale[:3])
            if min(sources[:3]) > 0:
                skews.append((max(sources[:3]) - min(sources[:3])) / 1e6)
            for index, can_id in enumerate(KINDS[kind]):
                key = (side, kind, can_id, sources[index])
                if key not in source_lookup:
                    stats["joint_source_not_in_bag"] += 1
                    bounds = source_bounds.get((side, kind, can_id))
                    if sources[index] <= 0:
                        reason = "zero_stamp"
                    elif bounds is None:
                        reason = "no_raw_stream"
                    elif sources[index] < bounds[0]:
                        reason = "before_raw_start"
                    elif sources[index] > bounds[1]:
                        reason = "after_raw_end"
                    else:
                        reason = "inside_raw_span"
                    stats["joint_source_unmatched_" + reason] += 1
                else:
                    stats["matched_joint_sources"] += 1
                    if not np.allclose(source_lookup[key], message.position[2 * index:2 * index + 2], atol=1e-12, rtol=0):
                        stats["joint_value_error"] += 1
        vector_stats[f"{side}/{kind}"] = dict(stats, joint_source_skew_ms=quantiles(skews))
    comparisons = {}
    for side in ("left", "right"):
        for index in range(3):
            target = streams[(side, "master_target", KINDS["master_target"][index])]
            feedback = streams[(side, "slave_feedback", KINDS["slave_feedback"][index])]
            comparisons[f"{side}/joint{index * 2 + 1}_{index * 2 + 2}"] = paired(target, feedback, tolerance_ns)
    files = [bag / "metadata.yaml", *sorted(bag.glob("*.db3"))]
    return {"bag": str(bag), "source_sha256": {p.name: sha256(p) for p in files},
            "topic_counts": dict(topic_counts), "direct_errors": dict(errors),
            "duplicate_source_keys": duplicate_keys, "conflicting_source_keys": conflicting_keys,
            "direct_streams": stream_stats, "vectors": vector_stats, "near_time_pairs": comparisons}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bags", type=Path, nargs="+")
    parser.add_argument("--max-pair-ms", type=float, default=30)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not math.isfinite(args.max_pair_ms) or args.max_pair_ms <= 0:
        parser.error("配对阈值必须为有限正数")
    if args.output.exists():
        parser.error("输出文件已存在，请使用新路径，避免覆盖既有证据")
    result = {"schema_version": "piper_joint_mapping_audit_v1", "max_pair_ms": args.max_pair_ms,
              "method": "独立有符号大端 int32 × pi/180000；同组最近 Header 配对，可使用未来反馈；不估计因果时延，不授予现场映射资格",
              "tool_sha256": sha256(Path(__file__)),
              "message_definition_sha256": {name: sha256(ROOT / "src/piper_msgs/msg" / name)
                                            for name in ("PiperDirectCanFrame.msg", "PiperArmVector.msg")},
              "bags": [audit(path.resolve(), round(args.max_pair_ms * 1e6)) for path in args.bags]}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x", encoding="utf-8") as output:
        json.dump(result, output, ensure_ascii=False, indent=2, allow_nan=False)
        output.write("\n")
    print(f"已写入只读六关节审计：{args.output}")


if __name__ == "__main__":
    main()
