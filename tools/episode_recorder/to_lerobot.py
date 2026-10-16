#!/usr/bin/env python3
"""把已保存的 rosbag2 Episode 转为本地 LeRobot v3。运行：python to_lerobot.py INPUT OUTPUT --repo-id local/piper --arm-mode LEFT --input-format can-vector --fps 10 --max-age-ms 100。"""

import argparse
from bisect import bisect_right
import hashlib
import json
import math
from pathlib import Path
import sys
import tempfile

import numpy as np
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

JOINTS = [f"joint{i}" for i in range(1, 7)] + ["gripper"]
CAMERAS = {
    "top_rgb": "/camera_top/D455/color/image_raw",
    "left_wrist_rgb": "/camera_wrist_left/color/image_raw",
    "right_wrist_rgb": "/camera_wrist_right/color/image_raw",
}
STATES = {arm: f"/joint_{arm}" for arm in ("left", "right")}
ACTIONS = {arm: f"/demonstration/action_{arm}" for arm in ("left", "right")}
CAN_STATES = {arm: f"/piper_vector/{arm}/slave_feedback" for arm in ("left", "right")}
CAN_ACTIONS = {arm: f"/piper_vector/{arm}/master_target" for arm in ("left", "right")}
CAN_IDS = {"master_target": [0x155, 0x156, 0x157, 0x159],
           "slave_feedback": [0x2A5, 0x2A6, 0x2A7, 0x2A8]}
VECTOR_TYPE = "piper_msgs/msg/PiperArmVector"
VECTOR_MSG = """std_msgs/Header header
string source_topic
string[7] joint_names
float64[7] position
uint32[4] source_can_ids
builtin_interfaces/Time[4] source_stamps
bool[4] missing
bool[4] stale
bool time_inconsistent
bool valid
bool gripper_mapping_verified
"""
TYPES = {**{topic: "sensor_msgs/msg/Image" for topic in CAMERAS.values()},
         **{topic: "sensor_msgs/msg/JointState" for topic in (*STATES.values(), *ACTIONS.values())},
         **{topic: VECTOR_TYPE for topic in (*CAN_STATES.values(), *CAN_ACTIONS.values())}}
TYPESTORE = get_typestore(Stores.ROS2_HUMBLE)
TYPESTORE.register(get_types_from_msg(VECTOR_MSG, VECTOR_TYPE))


def fail(message):
    raise ValueError(message)


def stamp(message):
    value = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
    if value <= 0 or message.header.stamp.nanosec >= 1_000_000_000:
        fail("Header 时间戳无效或非 Unix 纳秒")
    return value


def stamp_value(value):
    timestamp = value.sec * 1_000_000_000 + value.nanosec
    if timestamp <= 0 or not 0 <= value.nanosec < 1_000_000_000:
        fail("CAN 来源时间戳无效")
    return timestamp


def vector(message, topic):
    names = list(message.joint_names if topic in CAN_STATES.values() or topic in CAN_ACTIONS.values()
                 else message.name)
    if len(names) != len(set(names)) or set(names) != set(JOINTS) or len(message.position) != len(names):
        fail(f"{topic}: 必须恰有 joint1..joint6、gripper 和对应的 7 个 position")
    by_name = dict(zip(names, message.position))
    values = np.array([by_name[name] for name in JOINTS], dtype=np.float32)
    if not np.isfinite(values).all() or values[6] < 0:
        fail(f"{topic}: 位置非有限数或夹爪开口为负")
    return values


def can_vector(message, topic, max_can_skew_ns):
    arm, kind = topic.split("/")[2:]
    if (not message.valid or not message.gripper_mapping_verified
            or any(message.missing) or any(message.stale) or message.time_inconsistent):
        fail(f"{topic}: valid=false、夹爪映射未核实或帧完整性标志异常")
    if (message.source_topic != f"/piper_direct/{arm}/{kind}"
            or message.header.frame_id != f"can_{arm}"
            or list(message.source_can_ids) != CAN_IDS[kind]):
        fail(f"{topic}: CAN 来源话题、接口或 ID 不符")
    timestamps = [stamp_value(value) for value in message.source_stamps]
    if max(timestamps) != stamp(message) or max(timestamps) - min(timestamps) > max_can_skew_ns:
        fail(f"{topic}: Header 与四帧来源时间不一致或超窗")
    return vector(message, topic), timestamps


def image(message, topic, materialize, spool_dir=None):
    if (message.encoding != "rgb8" or message.width != 640 or message.height != 480
            or message.is_bigendian or message.step != 640 * 3
            or len(message.data) != 480 * 640 * 3 or not message.header.frame_id):
        fail(f"{topic}: 需要带 frame_id 的 640×480 连续 rgb8 图像")
    if spool_dir is not None:
        path = spool_dir / f"{hashlib.sha256(topic.encode()).hexdigest()[:12]}_{stamp(message)}.rgb"
        path.write_bytes(message.data.tobytes())
        return path
    return np.frombuffer(message.data, dtype=np.uint8).reshape(480, 640, 3).copy() if materialize else None


def read_episode(path, arms, fps, max_age_ns, jitter_ns, materialize=False, spool_dir=None,
                 input_format="joint-state", max_can_skew_ns=30_000_000):
    metadata_path = path / "episode.json"
    if not metadata_path.is_file() or not (path / "bag" / "metadata.yaml").is_file():
        fail(f"{path}: 缺少 episode.json 或 bag/metadata.yaml")
    meta = json.loads(metadata_path.read_text(encoding="utf-8"))
    expected_mode = "BIMANUAL" if len(arms) == 2 else arms[0].upper()
    fields = meta.get("task_metadata", {})
    if meta.get("schema_version") != 1 or meta.get("state") != "saved":
        fail(f"{path}: 仅接受 schema_version=1 且 state=saved 的 Episode")
    if not isinstance(fields, dict):
        fail(f"{path}: task_metadata 必须为对象")
    if (fields.get("arm_mode") != expected_mode or fields.get("time_domain") != "unix_ns"
            or fields.get("action_space") != "absolute_joint_position_7"
            or fields.get("action_provenance") != "human_demonstration"):
        fail(f"{path}: 缺少显式 arm_mode/time_domain/action_space/action_provenance 元数据")
    for key in ("calibration_version", "normalization_version"):
        if not isinstance(fields.get(key), str) or not fields[key].strip():
            fail(f"{path}: 缺少 {key} 标识")
    task = meta.get("task")
    if not isinstance(task, str) or not task.strip():
        fail(f"{path}: task 为空")
    if input_format not in ("joint-state", "can-vector"):
        fail("input_format 无效")
    state_topics = CAN_STATES if input_format == "can-vector" else STATES
    action_topics = CAN_ACTIONS if input_format == "can-vector" else ACTIONS
    needed = {CAMERAS["top_rgb"], *(CAMERAS[f"{arm}_wrist_rgb"] for arm in arms),
              *(state_topics[arm] for arm in arms), *(action_topics[arm] for arm in arms)}
    streams = {topic: [] for topic in needed}
    with Reader(path / "bag") as reader:
        connections = [conn for conn in reader.connections if conn.topic in needed]
        found = {conn.topic for conn in connections}
        if found != needed:
            fail(f"{path}: 缺少话题 {sorted(needed - found)}")
        for conn in connections:
            if conn.msgtype != TYPES[conn.topic] or conn.ext.serialization_format != "cdr":
                fail(f"{conn.topic}: 消息类型或序列化格式不符")
        for conn, _, raw in reader.messages(connections=connections):
            message = TYPESTORE.deserialize_cdr(raw, conn.msgtype)
            timestamp = stamp(message)
            topic = conn.topic
            if topic in CAMERAS.values():
                value, can_stamps = image(message, topic, materialize, spool_dir), None
            elif input_format == "can-vector":
                value, can_stamps = can_vector(message, topic, max_can_skew_ns)
            else:
                value, can_stamps = vector(message, topic), None
            streams[topic].append((timestamp, value, can_stamps))
    for topic, values in streams.items():
        if not values:
            fail(f"{path}: {topic} 没有消息")
        values.sort(key=lambda item: item[0])
        if input_format == "can-vector" and topic in (*CAN_STATES.values(), *CAN_ACTIONS.values()):
            deduplicated = [values[0]]
            for item in values[1:]:
                previous = deduplicated[-1]
                if item[0] == previous[0]:
                    if not np.array_equal(item[1], previous[1]) or item[2] != previous[2]:
                        fail(f"{path}: {topic} 同一 Header 时间对应冲突向量")
                else:
                    deduplicated.append(item)
            streams[topic] = deduplicated
            values = deduplicated
        if any(a[0] >= b[0] for a, b in zip(values, values[1:])):
            fail(f"{path}: {topic} 的 Header 时间戳重复或倒退")
    if input_format == "can-vector":
        first = max(streams[action_topics[arm]][0][0] for arm in arms)
        last = min(streams[action_topics[arm]][-1][0] for arm in arms)
        period = 1_000_000_000 / fps
        action_times = [round(first + index * period) for index in range(max(0, int((last - first) / period) + 1))]
    else:
        action_times = [item[0] for item in streams[action_topics[arms[0]]]]
    if len(action_times) < 2:
        fail(f"{path}: 至少需要两帧示教动作")
    start, stop = meta.get("started_at_unix_ns"), meta.get("stopped_at_unix_ns")
    if (type(start) is not int or type(stop) is not int or start <= 0 or stop <= start
            or action_times[0] < start or action_times[-1] > stop):
        fail(f"{path}: 动作 Header 时间不在 Episode 录制区间内")
    if input_format == "can-vector":
        for topic in (*[state_topics[arm] for arm in arms], *[action_topics[arm] for arm in arms]):
            if any(min(item[2]) < start or max(item[2]) > stop for item in streams[topic]):
                fail(f"{path}: {topic} 的 CAN 来源时间不在 Episode 录制区间内")
    if input_format == "joint-state":
        period = 1_000_000_000 / fps
        for index, timestamp in enumerate(action_times):
            if abs((timestamp - action_times[0]) - index * period) > jitter_ns:
                fail(f"{path}: 示教动作时间戳不符合 {fps} FPS，帧 {index}")
        for arm in arms[1:]:
            other = streams[action_topics[arm]]
            if len(other) != len(action_times) or any(abs(item[0] - ref) > jitter_ns for item, ref in zip(other, action_times)):
                fail(f"{path}: 左右示教动作未同步")
    indexes = {topic: [item[0] for item in values] for topic, values in streams.items()}
    def preceding(topic, timestamp):
        index = bisect_right(indexes[topic], timestamp) - 1
        if index < 0 or timestamp - indexes[topic][index] > max_age_ns:
            fail(f"{path}: {topic} 在动作时间 {timestamp} 前缺帧或超窗")
        if not start <= indexes[topic][index] <= stop:
            fail(f"{path}: {topic} 的 Header 时间不在 Episode 录制区间内")
        return streams[topic][index]
    frames = []
    for index, timestamp in enumerate(action_times):
        source_stamps, can_sources, actions = {}, {}, []
        for arm in arms:
            topic = action_topics[arm]
            actual, value, can_stamps = (preceding(topic, timestamp) if input_format == "can-vector"
                                          else streams[topic][index])
            source_stamps[topic] = actual
            if can_stamps is not None:
                can_sources[topic] = can_stamps
            actions.append(value)
        states = []
        for arm in arms:
            topic = state_topics[arm]
            observed_at, value, can_stamps = preceding(topic, timestamp)
            source_stamps[topic] = observed_at
            if can_stamps is not None:
                can_sources[topic] = can_stamps
            states.append(value)
        frame = {"task": task,
                 "observation.state": np.concatenate(states),
                 "action": np.concatenate(actions),
                 "__source_header_ns__": source_stamps,
                 "__source_can_stamps_ns__": can_sources}
        for camera in ("top_rgb", *(f"{arm}_wrist_rgb" for arm in arms)):
            observed_at, value, _ = preceding(CAMERAS[camera], timestamp)
            source_stamps[CAMERAS[camera]] = observed_at
            frame[f"observation.images.{camera}"] = value
        frames.append(frame)
    return meta, frames


def features(arms):
    names = [f"{arm}.{joint}" for arm in arms for joint in JOINTS]
    result = {key: {"dtype": "float32", "shape": (len(names),), "names": names}
              for key in ("observation.state", "action")}
    for camera in ("top_rgb", *(f"{arm}_wrist_rgb" for arm in arms)):
        result[f"observation.images.{camera}"] = {"dtype": "image", "shape": (480, 640, 3),
                                                    "names": ["height", "width", "channel"]}
    return result


def source_digest(path):
    digest = hashlib.sha256()
    databases = sorted((path / "bag").glob("*.db3"))
    if not databases:
        fail(f"{path}: 缺少 rosbag2 SQLite 文件")
    for file in [path / "episode.json", path / "bag" / "metadata.yaml", *databases]:
        digest.update(file.name.encode())
        with file.open("rb") as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(block)
    return digest.hexdigest()


def convert(args):
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    from importlib.metadata import version
    if version("lerobot") != "0.4.4" or version("rosbags") != "0.11.7":
        fail("依赖版本必须为 lerobot==0.4.4、rosbags==0.11.7")
    arms = ["left", "right"] if args.arm_mode == "BIMANUAL" else [args.arm_mode.lower()]
    paths = sorted(path for path in args.input_dir.glob("episode_*") if path.is_dir())
    if not paths:
        fail("输入目录没有 episode_* 目录")
    if args.output_dir.exists():
        fail("输出目录已存在，拒绝覆盖")
    prepared = []
    ids = set()
    for path in paths:
        meta, frames = read_episode(path, arms, args.fps, int(args.max_age_ms * 1e6),
                                    int(args.jitter_ms * 1e6), input_format=args.input_format,
                                    max_can_skew_ns=int(args.max_can_skew_ms * 1e6))
        episode_id = meta.get("episode_id")
        if not isinstance(episode_id, str) or not episode_id or episode_id in ids:
            fail("episode_id 缺失或重复")
        ids.add(episode_id)
        prepared.append((path, meta, len(frames), episode_id, source_digest(path)))
    if len(prepared) < 2:
        fail("至少需要两个完整 Episode 才能按 Episode 划分 train/test")
    test_count = min(len(prepared) - 1, max(1, round(len(prepared) * args.test_percent / 100)))
    ranked = sorted(prepared, key=lambda item: hashlib.sha256(item[3].encode()).digest())
    test_ids = {item[3] for item in ranked[:test_count]}
    args.output_dir.mkdir(parents=True)
    manifest = {"format": "LeRobotDataset v3.0", "lerobot_version": "0.4.4", "fps": args.fps,
                "arm_mode": args.arm_mode, "input_format": args.input_format,
                "joint_names": [f"{arm}.{joint}" for arm in arms for joint in JOINTS],
                "state_unit": "rad (joint1..6), m (gripper)", "action_space": "absolute_joint_position_7",
                "split_rule": f"sha256(episode_id) 排序前 {test_count}/{len(prepared)} 个为 test",
                "episodes": []}
    timestamps_file = (args.output_dir / "source_timestamps.jsonl").open("w", encoding="utf-8")
    for split in ("train", "test"):
        repo_id = f"{args.repo_id}_{split}"
        dataset = LeRobotDataset.create(repo_id=repo_id, fps=args.fps,
                                        features=features(arms), root=args.output_dir / split,
                                        robot_type="piper", use_videos=False)
        expected_episodes = expected_frames = 0
        for path, meta, count, episode_id, digest in prepared:
            if ("test" if episode_id in test_ids else "train") != split:
                continue
            with tempfile.TemporaryDirectory(prefix="piper-rgb-") as temporary:
                _, frames = read_episode(path, arms, args.fps, int(args.max_age_ms * 1e6),
                                         int(args.jitter_ms * 1e6), spool_dir=Path(temporary),
                                         input_format=args.input_format,
                                         max_can_skew_ns=int(args.max_can_skew_ms * 1e6))
                for index, frame in enumerate(frames):
                    timestamps_file.write(json.dumps({"split": split, "episode_id": episode_id,
                                                      "frame_index": index,
                                                      "header_timestamp_ns": frame.pop("__source_header_ns__"),
                                                      "source_can_stamps_ns": frame.pop("__source_can_stamps_ns__")},
                                                     ensure_ascii=False) + "\n")
                    for key, value in frame.items():
                        if key.startswith("observation.images."):
                            frame[key] = np.fromfile(value, dtype=np.uint8).reshape(480, 640, 3)
                    dataset.add_frame(frame)
            dataset.save_episode()
            expected_episodes += 1
            expected_frames += count
            manifest["episodes"].append({"episode_id": episode_id, "source": str(path.resolve()),
                                         "split": split, "frames": count,
                                         "source_sha256": digest,
                                         "calibration_version": meta["task_metadata"]["calibration_version"],
                                         "normalization_version": meta["task_metadata"]["normalization_version"],
                                         "bag_validation": meta.get("bag_validation")})
        dataset.finalize()
        check = LeRobotDataset(repo_id, root=args.output_dir / split)
        if check.meta.total_episodes != expected_episodes or len(check) != expected_frames:
            fail(f"{split}: LeRobot 回读 Episode 或帧数不匹配")
        sample = check[0]
        if tuple(sample["action"].shape) != (7 * len(arms),) or tuple(sample["observation.images.top_rgb"].shape) != (3, 480, 640):
            fail(f"{split}: LeRobot 回读特征维度不匹配")
    timestamps_file.close()
    (args.output_dir / "conversion_manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({"output": str(args.output_dir), "episodes": len(prepared),
                      "frames": sum(item[2] for item in prepared)}, ensure_ascii=False))


def main():
    parser = argparse.ArgumentParser(description="Episode rosbag2 → LeRobot v3 离线转换")
    parser.add_argument("input_dir", type=Path)
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("--repo-id", required=True)
    parser.add_argument("--arm-mode", choices=("LEFT", "RIGHT", "BIMANUAL"), required=True)
    parser.add_argument("--input-format", choices=("joint-state", "can-vector"), required=True)
    parser.add_argument("--fps", type=int, required=True)
    parser.add_argument("--max-age-ms", type=float, required=True)
    parser.add_argument("--jitter-ms", type=float, default=2)
    parser.add_argument("--max-can-skew-ms", type=float, default=30)
    parser.add_argument("--test-percent", type=int, default=20)
    args = parser.parse_args()
    if (args.fps <= 0 or not math.isfinite(args.max_age_ms) or args.max_age_ms <= 0
            or not math.isfinite(args.jitter_ms) or args.jitter_ms < 0
            or not math.isfinite(args.max_can_skew_ms) or args.max_can_skew_ms < 0
            or not 0 < args.test_percent < 100):
        parser.error("fps、max-age-ms、jitter-ms、max-can-skew-ms 或 test-percent 无效")
    try:
        convert(args)
    except (ValueError, OSError) as error:
        print(f"转换失败: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
