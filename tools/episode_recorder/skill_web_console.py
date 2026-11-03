#!/usr/bin/env python3
"""只读三路相机预览与 Episode 控制台。运行：python3 tools/episode_recorder/skill_web_console.py --config config.json。"""

import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import socket
import signal
import threading
import time
from urllib.parse import urlsplit

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from piper_msgs.msg import PiperArmVector


PAGE = Path(__file__).with_name("skill_web_console.html").read_bytes()
CAMERA_NAMES = ("左腕", "右腕", "顶部")
SKILLS = ("pick_banana", "place_banana_in_box", "close_hinged_lid_bimanual")
ARM_TOPICS = {arm: {"master_target": f"/piper_vector/{arm}/master_target",
                    "slave_feedback": f"/piper_vector/{arm}/slave_feedback"}
              for arm in ("left", "right")}


def teleoperation_ready(config, arm_mode):
    mapping = config.get("teleoperation")
    if not isinstance(mapping, dict) or mapping.get("source") != "master_slave_teleoperation":
        return False
    if mapping.get("mapping_verified") is not True or mapping.get("gripper_mapping_verified") is not True:
        return False
    required = ("left", "right") if arm_mode == "BIMANUAL" else (arm_mode.lower(),)
    configured = set(config.get("topics", ()))
    arms = mapping.get("arms")
    return isinstance(arms, dict) and all(
        arms.get(arm) == ARM_TOPICS[arm] and set(ARM_TOPICS[arm].values()).issubset(configured)
        for arm in required)


def camera_topics(config):
    topics = [topic for topic in config["topics"] if topic.endswith("/color/image_raw")]
    for marker in ("camera_wrist_left", "camera_wrist_right", "camera_top"):
        matches = [topic for topic in topics if f"/{marker}/" in topic]
        if len(matches) != 1:
            raise ValueError(f"{marker} 必须恰有一路 color/image_raw 话题")
        yield matches[0]


def encode_image(message):
    formats = {"rgb8": (3, cv2.COLOR_RGB2BGR), "bgr8": (3, None),
               "rgba8": (4, cv2.COLOR_RGBA2BGR), "bgra8": (4, cv2.COLOR_BGRA2BGR),
               "mono8": (1, cv2.COLOR_GRAY2BGR)}
    if message.encoding not in formats:
        raise ValueError(f"不支持的图像编码: {message.encoding}")
    channels, conversion = formats[message.encoding]
    width, height, step = message.width, message.height, message.step
    if not width or not height or step < width * channels or len(message.data) < step * height:
        raise ValueError("图像尺寸或数据长度无效")
    raw = np.frombuffer(message.data, dtype=np.uint8, count=step * height)
    frame = raw.reshape(height, step)[:, :width * channels].reshape(height, width, channels)
    if message.is_bigendian:
        raise ValueError("预览仅支持 8 位图像")
    if conversion is not None:
        frame = cv2.cvtColor(frame, conversion)
    if max(width, height) > 640:
        scale = 640 / max(width, height)
        frame = cv2.resize(frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
    if not ok:
        raise ValueError("JPEG 编码失败")
    return encoded.tobytes()


class Preview:
    def __init__(self, config):
        self.node = rclpy.create_node("piper_collection_web_preview")
        self.lock = threading.Lock()
        self.frames = [None] * 3
        self.times = [0.0] * 3
        self.errors = [None] * 3
        self.header_stamps = [None] * 3
        self.source_times = [0.0] * 3
        self.event = None
        self.joints = {arm: None for arm in ARM_TOPICS}
        self.topics = list(camera_topics(config))
        self.frame_topics = set(config.get("frame_topics", ()))
        for index, topic in enumerate(self.topics):
            self.node.create_subscription(Image, topic, lambda msg, i=index: self.on_image(i, msg), qos_profile_sensor_data)
        for arm, topics in ARM_TOPICS.items():
            self.node.create_subscription(PiperArmVector, topics["slave_feedback"],
                                          lambda msg, side=arm: self.on_joint(side, msg), 10)
        self.node.create_subscription(String, "/collection/events", self.on_event, 10)

    def on_joint(self, arm, message):
        source_stamps = [stamp.sec * 1_000_000_000 + stamp.nanosec
                         for stamp in message.source_stamps[:3]]
        stamp_ns = max(source_stamps)
        positions = list(message.position[:6])
        usable = (all(stamp > 0 for stamp in source_stamps)
                  and max(source_stamps) - min(source_stamps) <= 30_000_000
                  and len(positions) == 6
                  and all(math.isfinite(value) for value in positions)
                  and not any(message.missing[:3]) and not any(message.stale[:3])
                  and message.source_topic == f"/piper_direct/{arm}/slave_feedback")
        with self.lock:
            self.joints[arm] = {"topic": ARM_TOPICS[arm]["slave_feedback"],
                                "header_stamp_ns": stamp_ns, "received_at_unix_ns": time.time_ns(),
                                "received_at_monotonic": time.monotonic(),
                                "position_rad": positions if usable else None,
                                "usable": usable}

    def joint_snapshot(self):
        with self.lock:
            latest = {arm: dict(value) if value else None for arm, value in self.joints.items()}
        now = time.monotonic()
        result = {}
        for arm, value in latest.items():
            topic = ARM_TOPICS[arm]["slave_feedback"]
            publishers = self.node.count_publishers(topic)
            age = now - value["received_at_monotonic"] if value else None
            online = bool(value and value["usable"] and age <= 1.5)
            result[arm] = {"topic": topic, "publishers": publishers, "online": online,
                           "age_sec": round(age, 2) if age is not None else None,
                           "header_stamp_ns": value["header_stamp_ns"] if online else None,
                           "received_at_unix_ns": value["received_at_unix_ns"] if online else None,
                           "position_rad": value["position_rad"] if online else None,
                           "error": None if online else ("话题无发布者；请检查只读 piper_direct_can 与 piper_arm_vector" if not publishers else
                                   "未收到关节反馈" if value is None else
                                   "关节反馈过期" if age > 1.5 else "关节反馈缺帧或时间不一致")}
        return result

    def initial_joint_positions(self, required_arms):
        snapshot = self.joint_snapshot()
        for arm in required_arms:
            if not snapshot[arm]["online"]:
                raise ValueError(f"{arm} 臂没有可用的实时关节反馈：{snapshot[arm]['error']}")
        return {arm: {"topic": item["topic"], "header_stamp_ns": item["header_stamp_ns"],
                      "received_at_unix_ns": item["received_at_unix_ns"],
                      "joint_names": [f"joint{index}" for index in range(1, 7)],
                      "position_rad": item["position_rad"]}
                for arm, item in snapshot.items() if item["online"]}

    def on_image(self, index, message):
        now = time.monotonic()
        stamp = message.header.stamp
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        with self.lock:
            if now - self.times[index] < 0.4:
                return
        try:
            frame = encode_image(message)
            error = None
        except (ValueError, cv2.error) as exc:
            frame, error = None, str(exc)
        with self.lock:
            self.frames[index], self.errors[index], self.times[index] = frame, error, now
            self.header_stamps[index] = stamp_ns if frame and stamp_ns > 0 else None
            self.source_times[index] = now if frame and stamp_ns > 0 else 0.0

    def latest_reference(self, index):
        with self.lock:
            stamp = self.header_stamps[index]
            if (self.topics[index] in self.frame_topics and self.frames[index]
                    and stamp and time.monotonic() - self.source_times[index] <= 0.5):
                return self.topics[index], stamp
        return None

    def on_event(self, message):
        try:
            event = json.loads(message.data)
        except json.JSONDecodeError:
            return
        with self.lock:
            self.event = event

    def snapshot(self):
        with self.lock:
            now = time.monotonic()
            frames = list(self.frames)
            times = list(self.times)
            errors = list(self.errors)
        result = []
        for i, topic in enumerate(self.topics):
            publishers = self.node.count_publishers(topic)
            online = bool(frames[i]) and now - times[i] < 3
            error = None if online else ("话题无发布者；请启动相机 ROS 节点" if not publishers else
                    errors[i] or ("图像已过期" if times[i] else "已发现发布者，尚未收到图像"))
            result.append({"name": CAMERA_NAMES[i], "topic": topic, "publishers": publishers,
                           "online": online, "age_sec": round(now - times[i], 1) if times[i] else None,
                           "error": error})
        return result


def recorder_request(socket_path, request):
    payload = json.dumps(request, ensure_ascii=False).encode()
    with socket.socket(socket.AF_UNIX) as client:
        client.settimeout(30)
        client.connect(str(socket_path))
        client.sendall(payload)
        chunks = []
        while chunk := client.recv(1_000_000):
            chunks.append(chunk)
        data = b"".join(chunks)
    if not data:
        raise OSError("录制服务未返回数据")
    return json.loads(data)


def start_request(body, config, preview):
    task, instruction = body.get("task"), body.get("language_instruction")
    if (not isinstance(task, str) or not isinstance(instruction, str)
            or not task.strip() or not instruction.strip()
            or len(task.strip()) > 120 or len(instruction.strip()) > 2000):
        raise ValueError("请填写任务名和语言指令（任务名最多 120 字，指令最多 2000 字）")
    mode = body.get("mode", "task")
    if mode not in ("task", "skill"):
        raise ValueError("采集类型必须为 task 或 skill")
    missing_cameras = [camera["name"] for camera in preview.snapshot() if not camera["online"]]
    if missing_cameras:
        raise ValueError("相机预览未就绪：" + "、".join(missing_cameras))
    metadata = {"language_instruction": instruction.strip()}
    perturbation_id = body.get("perturbation_id", "")
    if not isinstance(perturbation_id, str) or len(perturbation_id.strip()) > 120:
        raise ValueError("扰动编号须为最多 120 字的文本")
    if mode == "skill" and not perturbation_id.strip():
        raise ValueError("单技能采集须填写扰动编号，以便按场景划分训练与测试")
    skill = body.get("skill") if mode == "skill" else None
    if mode == "skill" and skill not in SKILLS:
        raise ValueError("请选择固定技能")
    if mode == "skill":
        arm_mode = body.get("arm_mode")
        if skill == "close_hinged_lid_bimanual":
            if arm_mode != "BIMANUAL":
                raise ValueError("双臂翻盖技能须选择 BIMANUAL")
        elif arm_mode not in ("LEFT", "RIGHT"):
            raise ValueError("单臂技能须选择 LEFT 或 RIGHT")
        metadata["arm_mode"] = arm_mode
        for field in ("calibration_version", "normalization_version"):
            value = body.get(field)
            if (not isinstance(value, str) or not value.strip() or len(value.strip()) > 120
                    or value.strip().upper().startswith("UNVERIFIED")):
                raise ValueError(f"{field} 须填写已验收的非空版本号，不能使用 UNVERIFIED 占位值")
            metadata[field] = value.strip()
        if teleoperation_ready(config, arm_mode):
            metadata.update(time_domain="unix_ns", action_space="absolute_joint_position_7",
                            action_provenance="human_demonstration")
            metadata["collection_contract_status"] = "metadata_ready_pending_data_audit"
        else:
            metadata["collection_contract_status"] = "training_not_ready_unverified_teleoperation_mapping"
    required_arms = ("left", "right") if mode == "task" or arm_mode == "BIMANUAL" else (arm_mode.lower(),)
    metadata["initial_conditions"] = {"source": "ros_slave_feedback",
                                      "joint_positions": preview.initial_joint_positions(required_arms)}
    if perturbation_id.strip():
        metadata["initial_conditions"]["perturbation_id"] = perturbation_id.strip()
    return {"action": "start", "mode": mode, "task": task.strip(), "skill": skill,
            "metadata": metadata}


def annotation_request(body, preview):
    kind, value = body.get("kind"), body.get("value")
    if kind in ("skill_start", "skill_complete"):
        if value not in SKILLS:
            raise ValueError("请选择固定技能")
    elif kind == "outcome":
        if value not in ("success", "failure"):
            raise ValueError("结果须为 success 或 failure")
    elif kind == "recovery_trigger":
        if not isinstance(value, str) or not value.strip() or len(value.strip()) > 200:
            raise ValueError("请填写 1 至 200 字的异常或恢复原因")
        value = value.strip()
    else:
        raise ValueError("未知标注类型")
    request = {"action": "annotate", "kind": kind, "value": value}
    index = body.get("camera_index")
    if index is not None:
        if isinstance(index, bool) or not isinstance(index, int) or index not in range(3):
            raise ValueError("关联画面必须为左腕、右腕或顶部")
        reference = preview.latest_reference(index)
        if reference:
            request.update(source_topic=reference[0], header_stamp_ns=reference[1])
    return request


def handler_factory(socket_path, preview, config):
    class Handler(BaseHTTPRequestHandler):
        def respond(self, status, payload, content_type="application/json; charset=utf-8"):
            if not isinstance(payload, bytes):
                payload = json.dumps(payload, ensure_ascii=False).encode()
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(payload)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.end_headers()
            self.wfile.write(payload)

        def do_GET(self):
            path = urlsplit(self.path).path
            if path == "/":
                return self.respond(200, PAGE, "text/html; charset=utf-8")
            if path == "/api/status":
                try:
                    result = recorder_request(socket_path, {"action": "status"})
                    with preview.lock:
                        event = preview.event
                    result["cameras"] = preview.snapshot()
                    result["joints"] = preview.joint_snapshot()
                    result["event"] = event
                    return self.respond(200, result)
                except (OSError, ValueError, json.JSONDecodeError) as exc:
                    return self.respond(503, {"ok": False, "error": str(exc),
                                              "cameras": preview.snapshot(), "joints": preview.joint_snapshot()})
            if path == "/api/episodes":
                try:
                    return self.respond(200, recorder_request(socket_path, {"action": "list"}))
                except (OSError, ValueError, json.JSONDecodeError) as exc:
                    return self.respond(503, {"ok": False, "error": str(exc)})
            if path == "/api/contract":
                return self.respond(200, {"ok": True, "teleoperation_ready": {
                    mode: teleoperation_ready(config, mode) for mode in ("LEFT", "RIGHT", "BIMANUAL")}})
            if path.startswith("/camera/") and path.endswith(".jpg"):
                try:
                    index = int(path[8:-4])
                    with preview.lock:
                        frame = preview.frames[index] if index in range(3) else None
                    if frame:
                        return self.respond(200, frame, "image/jpeg")
                except ValueError:
                    pass
                return self.respond(404, {"error": "暂无图像"})
            return self.respond(404, {"error": "页面不存在"})

        def do_POST(self):
            path = urlsplit(self.path).path
            action = path.removeprefix("/api/")
            if path != f"/api/{action}" or action not in ("start", "stop", "save", "discard", "annotate"):
                return self.respond(404, {"ok": False, "error": "未知操作"})
            origin = self.headers.get("Origin")
            if origin and origin != f"http://{self.headers.get('Host')}":
                return self.respond(403, {"ok": False, "error": "来源不匹配"})
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if length < 0 or length > 65536:
                    return self.respond(413, {"ok": False, "error": "请求过大"})
                body = json.loads(self.rfile.read(length) or b"{}")
                if not isinstance(body, dict):
                    raise ValueError("请求必须为 JSON 对象")
                if action == "start":
                    request = start_request(body, config, preview)
                elif action == "annotate":
                    request = annotation_request(body, preview)
                elif action == "save":
                    request = {"action": action, "outcome": body.get("outcome")}
                    if request["outcome"] not in ("success", "failure"):
                        raise ValueError("请选择成功或失败标签")
                else:
                    request = {"action": action}
                response = recorder_request(socket_path, request)
                return self.respond(200 if response["ok"] else 409, response)
            except (OSError, ValueError, KeyError, json.JSONDecodeError) as exc:
                return self.respond(400, {"ok": False, "error": str(exc)})

    return Handler


def main():
    parser = argparse.ArgumentParser(description="Episode 数据采集 Web 控制台")
    parser.add_argument("--config", required=True, help="录制器 JSON 配置文件")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()
    with open(args.config, encoding="utf-8") as stream:
        config = json.load(stream)
    root = Path(config["output_dir"]).expanduser().resolve()
    socket_path = Path(config.get("socket_path", str(root / "recorder.sock"))).expanduser().resolve()
    rclpy.init()
    preview = Preview(config)
    def spin_preview():
        try:
            rclpy.spin(preview.node)
        except ExternalShutdownException:
            pass

    spin = threading.Thread(target=spin_preview, daemon=True)
    spin.start()
    server = ThreadingHTTPServer((args.host, args.port), handler_factory(socket_path, preview, config))
    def shutdown(_signum, _frame):
        threading.Thread(target=server.shutdown, daemon=True).start()
    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)
    print(f"控制台：http://{args.host}:{args.port}/", flush=True)
    try:
        server.serve_forever()
    finally:
        server.server_close()
        rclpy.shutdown()
        spin.join(timeout=2)
        preview.node.destroy_node()


if __name__ == "__main__":
    main()
