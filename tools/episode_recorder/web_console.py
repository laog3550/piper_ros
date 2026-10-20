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
from preview_depth import DEPTH_MAX_METERS, camera_topics, depth_to_bgr, overlay_color_edges


PAGE = Path(__file__).with_name("web_console.html").read_bytes()
CAMERA_NAMES = ("左腕", "右腕", "顶部")


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


def encode_depth(message, index, color_message=None):
    frame = depth_to_bgr(message, max_dimension=640, max_meters=DEPTH_MAX_METERS[index])
    overlay_color_edges(frame, color_message, (message.width, message.height))
    ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
    if not ok:
        raise ValueError("深度图 JPEG 编码失败")
    return encoded.tobytes()


class Preview:
    def __init__(self, config):
        self.node = rclpy.create_node("piper_collection_web_preview")
        self.lock = threading.Lock()
        self.frames = [[None] * 3 for _ in range(2)]
        self.times = [[0.0] * 3 for _ in range(2)]
        self.errors = [[None] * 3 for _ in range(2)]
        self.event = None
        self.color_messages = [None] * 3
        self.color_times = [0.0] * 3
        pairs = camera_topics(config)
        self.topics = [color for color, _ in pairs]
        self.depth_topics = [depth for _, depth in pairs]
        for kind, topics in enumerate((self.topics, self.depth_topics)):
            for index, topic in enumerate(topics):
                self.node.create_subscription(Image, topic,
                                              lambda msg, k=kind, i=index: self.on_image(k, i, msg),
                                              qos_profile_sensor_data)
        self.node.create_subscription(String, "/collection/events", self.on_event, 10)

    def on_image(self, kind, index, message):
        now = time.monotonic()
        with self.lock:
            if kind == 0:
                self.color_messages[index], self.color_times[index] = message, now
            if now - self.times[kind][index] < 0.4:
                return
            color_message = (self.color_messages[index] if kind == 1
                             and now - self.color_times[index] < 0.25 else None)
        try:
            frame = encode_image(message) if kind == 0 else encode_depth(message, index, color_message)
            error = None
        except (ValueError, cv2.error) as exc:
            frame, error = None, str(exc)
        with self.lock:
            self.frames[kind][index], self.errors[kind][index], self.times[kind][index] = frame, error, now

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
            return [{"name": CAMERA_NAMES[i],
                     "color": self._stream_status(0, i, self.topics[i], now),
                     "depth": self._stream_status(1, i, self.depth_topics[i], now)}
                    for i in range(3)]

    def _stream_status(self, kind, index, topic, now):
        return {"topic": topic, "online": bool(self.frames[kind][index]) and now - self.times[kind][index] < 3,
                "age_sec": round(now - self.times[kind][index], 1) if self.times[kind][index] else None,
                "error": self.errors[kind][index],
                "max_meters": DEPTH_MAX_METERS[index] if kind == 1 else None}


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


def handler_factory(socket_path, preview, default_save_dir):
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
                    result["default_save_dir"] = str(default_save_dir)
                    return self.respond(200, result)
                except (OSError, ValueError, json.JSONDecodeError) as exc:
                    return self.respond(503, {"ok": False, "error": str(exc), "cameras": preview.snapshot()})
            if path == "/api/episodes":
                try:
                    return self.respond(200, recorder_request(socket_path, {"action": "list"}))
                except (OSError, ValueError, json.JSONDecodeError) as exc:
                    return self.respond(503, {"ok": False, "error": str(exc)})
            if path in ("/api/plans", "/api/vlm/export"):
                try:
                    action = "plans" if path == "/api/plans" else "vlm_export"
                    response = recorder_request(socket_path, {"action": action})
                    return self.respond(200 if response["ok"] else 409, response)
                except (OSError, ValueError, json.JSONDecodeError) as exc:
                    return self.respond(503, {"ok": False, "error": str(exc)})
            if path.startswith("/camera/") and path.endswith(".jpg"):
                try:
                    parts = path.split("/")
                    kind = {"color": 0, "depth": 1}[parts[2]]
                    index = int(parts[3][:-4])
                    with preview.lock:
                        frame = preview.frames[kind][index] if index in range(3) else None
                    if frame:
                        return self.respond(200, frame, "image/jpeg")
                except (ValueError, KeyError, IndexError):
                    pass
                return self.respond(404, {"error": "暂无图像"})
            return self.respond(404, {"error": "页面不存在"})

        def do_POST(self):
            path = urlsplit(self.path).path
            action = path.removeprefix("/api/")
            if path != f"/api/{action}" or action not in ("start", "stop", "save", "discard", "plan_create"):
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
                    task = body.get("task")
                    if not isinstance(task, str) or not task.strip() or len(task.strip()) > 120:
                        raise ValueError("请填写 1 至 120 字的任务名")
                    duration = body.get("duration_sec")
                    if isinstance(duration, bool) or not isinstance(duration, (int, float)) or not 1 <= duration <= 86400:
                        raise ValueError("时长上限须为 1 至 86400 秒")
                    if "vlm" in body:
                        context = body["vlm"]
                        if not isinstance(context, dict):
                            raise ValueError("技能上下文须为 JSON 对象")
                        request = {"action": action, "mode": "skill", "task": task.strip(),
                                   "skill": body.get("skill"),
                                   "metadata": {"language_instruction": context.get("skill_instruction"),
                                                "vlm": context}, "duration_sec": duration}
                    else:
                        instruction = body.get("language_instruction")
                        if not isinstance(instruction, str) or not instruction.strip() or len(instruction.strip()) > 2000:
                            raise ValueError("请填写 1 至 2000 字的语言指令")
                        request = {"action": action, "mode": "task", "task": task.strip(), "skill": None,
                                   "metadata": {"language_instruction": instruction.strip()},
                                   "duration_sec": duration}
                elif action == "save":
                    request = {"action": action, "outcome": body.get("outcome"),
                               "save_dir": body.get("save_dir"), "annotation": body.get("annotation")}
                    if request["outcome"] not in ("success", "failure", "unknown"):
                        raise ValueError("请选择成功、失败或无法判断标签")
                elif action == "plan_create":
                    request = {"action": action, "plan": body}
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
    server = ThreadingHTTPServer((args.host, args.port), handler_factory(socket_path, preview, root))
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
