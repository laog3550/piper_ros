#!/usr/bin/env python3
"""只读三路相机预览与 Episode 控制台。运行：python3 tools/episode_recorder/web_console.py --config config.json。"""

import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
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


PAGE = Path(__file__).with_name("web_console.html").read_bytes()
CAMERA_NAMES = ("左腕", "右腕", "顶部")


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
        self.event = None
        self.topics = list(camera_topics(config))
        for index, topic in enumerate(self.topics):
            self.node.create_subscription(Image, topic, lambda msg, i=index: self.on_image(i, msg), qos_profile_sensor_data)
        self.node.create_subscription(String, "/collection/events", self.on_event, 10)

    def on_image(self, index, message):
        now = time.monotonic()
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
            return [{"name": CAMERA_NAMES[i], "topic": topic, "online": bool(self.frames[i]) and now - self.times[i] < 3,
                     "age_sec": round(now - self.times[i], 1) if self.times[i] else None,
                     "error": self.errors[i]} for i, topic in enumerate(self.topics)]


def recorder_request(socket_path, request):
    payload = json.dumps(request, ensure_ascii=False).encode()
    with socket.socket(socket.AF_UNIX) as client:
        client.settimeout(30)
        client.connect(str(socket_path))
        client.sendall(payload)
        data = client.recv(1_000_000)
    if not data:
        raise OSError("录制服务未返回数据")
    return json.loads(data)


def handler_factory(socket_path, preview):
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
                    result["event"] = event
                    return self.respond(200, result)
                except (OSError, ValueError, json.JSONDecodeError) as exc:
                    return self.respond(503, {"ok": False, "error": str(exc), "cameras": preview.snapshot()})
            if path == "/api/episodes":
                try:
                    return self.respond(200, recorder_request(socket_path, {"action": "list"}))
                except (OSError, ValueError, json.JSONDecodeError) as exc:
                    return self.respond(503, {"ok": False, "error": str(exc)})
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
            if path != f"/api/{action}" or action not in ("start", "stop", "save", "discard"):
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
                    task = body.get("task", "").strip()
                    instruction = body.get("language_instruction", "").strip()
                    if not task or not instruction or len(task) > 120 or len(instruction) > 2000:
                        raise ValueError("请填写任务名和语言指令（任务名最多 120 字，指令最多 2000 字）")
                    request = {"action": action, "mode": "task", "task": task, "skill": None,
                               "metadata": {"language_instruction": instruction}}
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
    server = ThreadingHTTPServer((args.host, args.port), handler_factory(socket_path, preview))
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
