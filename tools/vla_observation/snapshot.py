#!/usr/bin/env python3
"""只读 RGB-D/关节快照与光学坐标系 ROI 摘要。运行：python3 tools/vla_observation/snapshot.py bag EPISODE/bag --config config/vla_observation.example.json --output snapshots.jsonl；或 live --help。"""

import argparse
from collections import deque
import json
import math
from pathlib import Path
import sqlite3
import struct
import sys
import time
import uuid

import numpy as np

try:
    from sensor_msgs.msg import CameraInfo, Image, JointState
except ImportError:  # 纯合成测试不依赖 ROS 安装
    CameraInfo = Image = JointState = None

CONTRACT_VERSION = "piper_vla_mvp/1.0"
JOINTS = [f"joint{i}" for i in range(1, 7)] + ["gripper"]
CAMERAS = {
    "top": ("/camera_top/D455/color/image_raw", "/camera_top/D455/aligned_depth_to_color/image_raw",
            "/camera_top/D455/aligned_depth_to_color/camera_info"),
    "left_wrist": ("/camera_wrist_left/color/image_raw", "/camera_wrist_left/depth/image_raw",
                   "/camera_wrist_left/depth/camera_info"),
    "right_wrist": ("/camera_wrist_right/color/image_raw", "/camera_wrist_right/depth/image_raw",
                    "/camera_wrist_right/depth/camera_info"),
}
STATES = {"left": "/joint_left", "right": "/joint_right"}
TOPICS = {topic: (role, kind) for role, triple in CAMERAS.items()
          for topic, kind in zip(triple, ("rgb", "depth", "info"))}
TOPICS.update({topic: (arm, "state") for arm, topic in STATES.items()})


def stamp_ns(msg):
    stamp = msg.header.stamp
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
        raise ValueError("header.stamp 无效")
    result = stamp.sec * 1_000_000_000 + stamp.nanosec
    if result <= 0:
        raise ValueError("header.stamp 为空或时间域不符")
    return result


def nearest(items, target, now, tolerance):
    eligible = (item for item in items if item[0] <= now and abs(item[0] - target) <= tolerance)
    return min(eligible, key=lambda item: (abs(item[0] - target), -item[0]), default=None)


def status(item, now, max_age):
    if item is None:
        return {"valid": False, "reason": "missing", "source_timestamp_ns": None, "age_ns": None, "stale": False}
    age = now - item[0]
    stale = age < 0 or age > max_age
    return {"valid": not stale, "reason": "stale" if stale else None,
            "source_timestamp_ns": item[0], "age_ns": age, "stale": stale}


def validate_info(msg, width, height):
    k = list(msg.k)
    return (msg.width == width and msg.height == height and len(k) == 9
            and all(math.isfinite(x) for x in k) and k[0] > 0 and k[4] > 0
            and k[8] == 1 and 0 <= k[2] < width and 0 <= k[5] < height
            and all(abs(k[i]) < 1e-9 for i in (1, 3, 6, 7)))


def depth_array(msg, scale):
    if msg.encoding not in ("16UC1", "32FC1") or msg.is_bigendian not in (0, 1):
        raise ValueError("不支持的深度编码或字节序")
    dtype = np.dtype((">" if msg.is_bigendian else "<") + ("u2" if msg.encoding == "16UC1" else "f4"))
    row_bytes = msg.width * dtype.itemsize
    if msg.step < row_bytes or len(msg.data) < msg.step * msg.height:
        raise ValueError("深度数据长度不足")
    data = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=bytes(msg.data), strides=(msg.step, dtype.itemsize))
    return data.astype(np.float64) * (scale if msg.encoding == "16UC1" else 1.0)


def roi_summary(depth, info, roi, scale):
    x, y, width, height = roi
    if x < 0 or y < 0 or width <= 0 or height <= 0 or x + width > depth.width or y + height > depth.height:
        raise ValueError("ROI 超出深度图像")
    if not validate_info(info, depth.width, depth.height):
        raise ValueError("CameraInfo 内参或尺寸无效")
    z = depth_array(depth, scale)[y:y + height, x:x + width]
    good = np.isfinite(z) & (z > 0)
    count = int(good.sum())
    result = {"roi_xywh": list(roi), "valid_pixels": count,
              "valid_fraction": count / (width * height), "depth_unit": "m",
              "optical_frame": depth.header.frame_id, "distance_median_m": None,
              "distance_min_m": None, "point_median_xyz_m": None}
    if count:
        values = z[good]
        yy, xx = np.nonzero(good)
        u = xx + x
        v = yy + y
        k = info.k
        points = np.column_stack(((u - k[2]) * values / k[0], (v - k[5]) * values / k[4], values))
        result.update(distance_median_m=float(np.median(values)), distance_min_m=float(np.min(values)),
                      point_median_xyz_m=[float(value) for value in np.median(points, axis=0)])
    return result


class SnapshotBuilder:
    def __init__(self, config):
        self.config = config
        self._validate_config()
        self.buffers = {topic: deque(maxlen=90) for topic in TOPICS}
        self.session = uuid.uuid4().hex
        self.sequence = 0

    def _validate_config(self):
        for key in ("calibration_version", "normalization_version"):
            if not isinstance(self.config.get(key), str) or not self.config[key].strip():
                raise ValueError(f"{key} 必须显式指定")
        for key in ("max_age_ns", "max_cross_camera_skew_ns", "max_rgb_depth_skew_ns", "max_info_age_ns"):
            if type(self.config.get(key)) is not int or self.config[key] <= 0:
                raise ValueError(f"{key} 必须为正整数")
        scales = self.config.get("depth_scale_m_per_unit", {})
        if set(scales) != set(CAMERAS) or any(type(v) not in (int, float) or not math.isfinite(v)
                                              or v <= 0 for v in scales.values()):
            raise ValueError("三路 16UC1 深度单位必须显式配置为正有限数")
        if self.config.get("time_domain") != "unix_ns":
            raise ValueError("只支持 unix_ns 时间域")

    def add(self, topic, msg, uri):
        if topic not in TOPICS:
            return
        self.buffers[topic].append((stamp_ns(msg), msg, uri))

    def _camera(self, role, anchor, now):
        rgb_topic, depth_topic, info_topic = CAMERAS[role]
        cfg = self.config
        candidates = [item for item in self.buffers[rgb_topic]
                      if item[0] <= now and abs(item[0] - anchor) <= cfg["max_cross_camera_skew_ns"]]
        # 同一相机先保持 RGB-D 配对，再从可用对中选择最接近跨相机锚点的一对。
        pairs = [(item, nearest(self.buffers[depth_topic], item[0], now,
                                cfg["max_rgb_depth_skew_ns"])) for item in candidates]
        paired = [(item, depth_item) for item, depth_item in pairs if depth_item is not None]
        if paired:
            rgb, depth = min(paired, key=lambda pair: (abs(pair[0][0] - anchor),
                                                       abs(pair[1][0] - pair[0][0])))
        else:
            rgb = nearest(self.buffers[rgb_topic], anchor, now, cfg["max_cross_camera_skew_ns"])
            depth = None
        rgb_status = status(rgb, now, cfg["max_age_ns"])
        result = {"role": role, "rgb": rgb_status, "depth": status(None, now, cfg["max_age_ns"]),
                  "camera_info": status(None, now, cfg["max_info_age_ns"]),
                  "rgb_depth_delta_ns": None, "roi_depth": {"valid": False, "reason": "missing_rgb"},
                  "world_geometry": {"valid": False, "reason": "extrinsics_and_tf_not_verified"}}
        if rgb is None:
            result["world_geometry"]["reason"] = "missing_rgb"
            return result, None
        rgb_msg = rgb[1]
        if rgb_msg.encoding != "rgb8" or rgb_msg.width != 640 or rgb_msg.height != 480 or not rgb_msg.header.frame_id:
            rgb_status.update(valid=False, reason="invalid_rgb")
        rgb_status.update(topic=rgb_topic, frame_id=rgb_msg.header.frame_id, uri=rgb[2],
                          encoding=rgb_msg.encoding, width=rgb_msg.width, height=rgb_msg.height)
        result["depth"] = status(depth, now, cfg["max_age_ns"])
        if depth is None:
            result["roi_depth"]["reason"] = "missing_depth"
            result["world_geometry"]["reason"] = "missing_depth"
            return result, rgb
        result["rgb_depth_delta_ns"] = depth[0] - rgb[0]
        result["depth"].update(topic=depth_topic, frame_id=depth[1].header.frame_id, encoding=depth[1].encoding)
        info = nearest(self.buffers[info_topic], depth[0], now, cfg["max_info_age_ns"])
        result["camera_info"] = status(info, now, cfg["max_info_age_ns"])
        if info:
            result["camera_info"].update(topic=info_topic, frame_id=info[1].header.frame_id)
        if not rgb_status["valid"] or not result["depth"]["valid"] or not result["camera_info"]["valid"]:
            result["roi_depth"]["reason"] = "source_invalid"
            result["world_geometry"]["reason"] = "source_invalid"
            return result, rgb
        if not (rgb_msg.header.frame_id == depth[1].header.frame_id == info[1].header.frame_id):
            result["roi_depth"]["reason"] = "optical_frame_mismatch"
            result["world_geometry"]["reason"] = "optical_frame_mismatch"
            return result, rgb
        if (depth[1].width, depth[1].height) != (rgb_msg.width, rgb_msg.height):
            result["roi_depth"]["reason"] = "rgb_depth_size_mismatch"
            result["world_geometry"]["reason"] = "rgb_depth_size_mismatch"
            return result, rgb
        try:
            roi = cfg.get("roi_xywh", [rgb_msg.width // 2 - 20, rgb_msg.height // 2 - 20, 40, 40])
            summary = roi_summary(depth[1], info[1], roi, cfg["depth_scale_m_per_unit"][role])
            result["roi_depth"] = {"valid": summary["valid_pixels"] > 0,
                                   "reason": None if summary["valid_pixels"] else "no_valid_depth", **summary}
        except (ValueError, TypeError) as error:
            result["roi_depth"] = {"valid": False, "reason": str(error)}
        if not result["roi_depth"]["valid"]:
            result["world_geometry"]["reason"] = "invalid_roi_depth"
        return result, rgb

    def build(self, mode, anchor_ns, now_ns=None):
        if mode not in ("LEFT", "RIGHT", "BIMANUAL") or anchor_ns <= 0:
            raise ValueError("arm_mode 或 anchor 无效")
        now = now_ns if now_ns is not None else time.time_ns()
        roles = ["top"] + (["left_wrist", "right_wrist"] if mode == "BIMANUAL" else
                           ["left_wrist" if mode == "LEFT" else "right_wrist"])
        cameras, images, reasons = {}, {}, []
        for role in roles:
            camera, rgb = self._camera(role, anchor_ns, now)
            cameras[role] = camera
            if not camera["rgb"]["valid"]:
                reasons.append(f"{role}.rgb:{camera['rgb']['reason']}")
            else:
                images["top_rgb" if role == "top" else f"{role}_rgb"] = {
                    key: camera["rgb"][key] for key in ("topic", "source_timestamp_ns", "frame_id", "encoding", "width", "height", "uri")}
        states = {}
        for arm in (["left", "right"] if mode == "BIMANUAL" else [mode.lower()]):
            topic = STATES[arm]
            item = nearest(self.buffers[topic], anchor_ns, now, self.config["max_cross_camera_skew_ns"])
            item_status = status(item, now, self.config["max_age_ns"])
            if item_status["valid"]:
                names = list(item[1].name)
                values = list(item[1].position)
                if len(names) != len(set(names)) or not set(JOINTS).issubset(names) or len(values) != len(names):
                    item_status.update(valid=False, reason="joint_names_invalid")
                else:
                    positions = [values[names.index(name)] for name in JOINTS]
                    if any(not math.isfinite(value) for value in positions) or positions[-1] < 0:
                        item_status.update(valid=False, reason="joint_values_invalid")
                    else:
                        item_status["position"] = positions
            if item_status["valid"]:
                states[arm] = {"source_topic": topic, "header_timestamp_ns": item[0],
                               "joint_names": JOINTS, "position": item_status["position"]}
            else:
                reasons.append(f"{arm}.state:{item_status['reason']}")
            item_status["source_topic"] = topic
            cameras[f"{arm}_state"] = item_status
        self.sequence += 1
        ready = not reasons
        record = {"snapshot_id": f"{self.session}-{self.sequence}", "snapshot_timestamp_ns": now,
                  "anchor_timestamp_ns": anchor_ns, "time_domain": "unix_ns", "arm_mode": mode,
                  "calibration_version": self.config["calibration_version"], "sources": cameras,
                  "world_geometry_valid": False,
                  "contract_observation_valid": ready, "rejection_reasons": reasons,
                  "observation": None}
        if ready:
            record["observation"] = {"contract_version": CONTRACT_VERSION,
                "observation_id": record["snapshot_id"], "observation_timestamp_ns": now,
                "time_domain": "unix_ns", "arm_mode": mode,
                "calibration_version": self.config["calibration_version"],
                "normalization_version": self.config["normalization_version"],
                "images": {name: {**{key: value for key, value in image.items() if key != "source_timestamp_ns"},
                                  "header_timestamp_ns": image["source_timestamp_ns"]}
                           for name, image in images.items()}, "state": states}
        return record


def bag_records(directory):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    files = sorted(Path(directory).glob("*.db3"))
    if not files:
        raise ValueError("未找到 rosbag2 SQLite 文件")
    connections = []
    try:
        index = []
        topic_maps = []
        for file_index, path in enumerate(files):
            connection = sqlite3.connect(f"file:{path}?mode=ro", uri=True)
            connections.append(connection)
            known = {row[0]: (row[1], get_message(row[2])) for row in connection.execute(
                "SELECT id,name,type FROM topics") if row[1] in TOPICS}
            topic_maps.append(known)
            for msg_id, topic_id, prefix in connection.execute("SELECT id,topic_id,substr(data,1,12) FROM messages"):
                if topic_id not in known:
                    continue
                if len(prefix) < 12 or prefix[0] != 0 or prefix[1] not in (0, 1):
                    raise ValueError(f"{path.name} 中有无效 CDR Header")
                sec, nanosec = struct.unpack_from("<iI" if prefix[1] == 1 else ">iI", prefix, 4)
                if sec <= 0 or nanosec >= 1_000_000_000:
                    continue
                index.append((sec * 1_000_000_000 + nanosec, file_index, msg_id, topic_id))
        for _, file_index, msg_id, topic_id in sorted(index):
            payload = connections[file_index].execute("SELECT data FROM messages WHERE id=?", (msg_id,)).fetchone()[0]
            topic, msg_type = topic_maps[file_index][topic_id]
            yield topic, deserialize_message(payload, msg_type), \
                f"rosbag2://{files[file_index].resolve()}#message={msg_id}"
    finally:
        for connection in connections:
            connection.close()


def run_bag(args, config):
    builder = SnapshotBuilder(config)
    pending = deque()
    count = 0
    with open(args.output, "w", encoding="utf-8") as stream:
        for topic, msg, uri in bag_records(args.bag):
            try:
                timestamp = stamp_ns(msg)
            except ValueError:
                continue
            builder.add(topic, msg, uri)
            if topic == CAMERAS["top"][0]:
                pending.append(timestamp)
            while pending and timestamp > pending[0] + config["max_cross_camera_skew_ns"]:
                anchor = pending.popleft()
                result = builder.build(args.arm_mode, anchor, anchor + config["max_cross_camera_skew_ns"])
                stream.write(json.dumps(result, ensure_ascii=False) + "\n")
                count += 1
                if args.limit and count >= args.limit:
                    return
        while pending and (not args.limit or count < args.limit):
            anchor = pending.popleft()
            result = builder.build(args.arm_mode, anchor, anchor + config["max_cross_camera_skew_ns"])
            stream.write(json.dumps(result, ensure_ascii=False) + "\n")
            count += 1
    if count == 0:
        raise ValueError("bag 中没有可用的顶部 RGB 帧")


def run_live(args, config):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    rclpy.init()
    node = rclpy.create_node("vla_observation_read_only")
    builder = SnapshotBuilder(config)
    top_topic = CAMERAS["top"][0]
    anchors = deque()
    subscriptions = []
    def callback(topic):
        def receive(msg):
            try:
                ts = stamp_ns(msg)
            except ValueError:
                return
            builder.add(topic, msg, f"ros://{topic}?header_timestamp_ns={ts}")
            if topic == top_topic:
                anchors.append(ts)
        return receive
    for topic, (_, kind) in TOPICS.items():
        msg_type = {"rgb": Image, "depth": Image, "info": CameraInfo, "state": JointState}[kind]
        subscriptions.append(node.create_subscription(msg_type, topic, callback(topic), qos_profile_sensor_data))
    try:
        with open(args.output, "w", encoding="utf-8") as stream:
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.05)
                now = time.time_ns()
                while anchors and now >= anchors[0] + config["max_cross_camera_skew_ns"]:
                    result = builder.build(args.arm_mode, anchors.popleft(), now)
                    # 实时 URI 不指向不可变帧，故不导出 v1 离线契约对象。
                    result["observation"] = None
                    result["contract_observation_valid"] = False
                    result["rejection_reasons"].append("live_rgb_uri_not_immutable")
                    stream.write(json.dumps(result, ensure_ascii=False) + "\n")
                    stream.flush()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description="只读 RGB-D/关节快照；不发布 ROS 话题或控制命令")
    sub = parser.add_subparsers(dest="command", required=True)
    for command in ("bag", "live"):
        part = sub.add_parser(command)
        if command == "bag":
            part.add_argument("bag", help="Episode 的 bag 目录")
            part.add_argument("--limit", type=int, default=0)
        part.add_argument("--config", required=True)
        part.add_argument("--output", required=True)
        part.add_argument("--arm-mode", choices=("LEFT", "RIGHT", "BIMANUAL"), default="LEFT")
    args = parser.parse_args()
    config = json.loads(Path(args.config).read_text(encoding="utf-8"))
    (run_bag if args.command == "bag" else run_live)(args, config)


if __name__ == "__main__":
    main()
