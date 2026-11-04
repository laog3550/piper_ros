#!/usr/bin/env python3
"""只读测三路 RGB Header 到订阅回调的时延，并可抽样本机网页 JPEG 缓存。"""

import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import threading
import time
from urllib.error import HTTPError, URLError
from urllib.parse import urlparse
from urllib.request import Request, urlopen

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from preview_depth import camera_topics


ROLES = ("left_wrist", "right_wrist", "top")
DEFAULT_CONFIG = Path(__file__).resolve().parents[2] / "config/episode_recorder.example.json"


def percentile(values, proportion):
    if not values:
        return None
    ordered = sorted(values)
    return round(ordered[max(0, math.ceil(proportion * len(ordered)) - 1)], 3)


def distribution(values):
    return {"p50": percentile(values, 0.50), "p95": percentile(values, 0.95),
            "p99": percentile(values, 0.99), "max": round(max(values), 3) if values else None}


class RosStream:
    def __init__(self, topic):
        self.topic = topic
        self.headers_ns = []
        self.received_mono_ns = []
        self.age_ms = []

    def on_image(self, message):
        # 在回调入口取本机时钟；年龄包含相机、传输、DDS 和订阅调度。
        received_wall_ns = time.time_ns()
        received_mono_ns = time.monotonic_ns()
        header_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        self.headers_ns.append(header_ns)
        self.received_mono_ns.append(received_mono_ns)
        self.age_ms.append((received_wall_ns - header_ns) / 1_000_000)

    def summary(self):
        header_gaps = [(right - left) / 1_000_000
                       for left, right in zip(self.headers_ns, self.headers_ns[1:])]
        receive_gaps = [(right - left) / 1_000_000
                        for left, right in zip(self.received_mono_ns, self.received_mono_ns[1:])]
        header_span = ((self.headers_ns[-1] - self.headers_ns[0]) / 1e9
                       if len(self.headers_ns) > 1 else 0)
        receive_span = ((self.received_mono_ns[-1] - self.received_mono_ns[0]) / 1e9
                        if len(self.received_mono_ns) > 1 else 0)
        return {
            "topic": self.topic, "frames": len(self.headers_ns),
            "header_fps": round((len(self.headers_ns) - 1) / header_span, 3) if header_span > 0 else None,
            "receive_fps": round((len(self.received_mono_ns) - 1) / receive_span, 3) if receive_span > 0 else None,
            "header_gap_ms": distribution(header_gaps), "receive_gap_ms": distribution(receive_gaps),
            "header_gap_gt_50ms": sum(gap > 50 for gap in header_gaps),
            "header_gap_gt_100ms": sum(gap > 100 for gap in header_gaps),
            "header_nonincreasing": sum(gap <= 0 for gap in header_gaps),
            "header_zero_count": sum(stamp <= 0 for stamp in self.headers_ns),
            "header_to_callback_age_ms": distribution(self.age_ms),
            "negative_age_count": sum(age < 0 for age in self.age_ms),
            "age_gt_1000ms": sum(age > 1000 for age in self.age_ms),
        }


class WebProbe:
    def __init__(self, url, poll_seconds):
        parsed = urlparse(url)
        if parsed.scheme != "http" or parsed.hostname not in ("127.0.0.1", "localhost", "::1"):
            raise ValueError("网页探针仅允许本机 HTTP 地址")
        self.base = url.rstrip("/")
        self.poll_seconds = poll_seconds
        self.stop = threading.Event()
        self.stats = [{"responses": 0, "changes": [], "last_hash": None,
                       "rtt_ms": [], "errors": Counter()} for _ in ROLES]

    def run(self):
        while not self.stop.is_set():
            cycle_start = time.monotonic()
            for index, item in enumerate(self.stats):
                started = time.monotonic_ns()
                request = Request(f"{self.base}/camera/color/{index}.jpg?t={started}",
                                  headers={"Cache-Control": "no-cache"})
                try:
                    with urlopen(request, timeout=0.7) as response:
                        payload = response.read()
                    finished = time.monotonic_ns()
                    item["responses"] += 1
                    item["rtt_ms"].append((finished - started) / 1e6)
                    digest = hashlib.sha256(payload).digest()
                    if item["last_hash"] is not None and digest != item["last_hash"]:
                        item["changes"].append(finished)
                    item["last_hash"] = digest
                except HTTPError as exc:
                    item["errors"][f"HTTP_{exc.code}"] += 1
                except (URLError, TimeoutError, OSError) as exc:
                    item["errors"][type(exc).__name__] += 1
            self.stop.wait(max(0, self.poll_seconds - (time.monotonic() - cycle_start)))

    def summary(self, duration_seconds):
        result = {}
        for role, item in zip(ROLES, self.stats):
            intervals = [(right - left) / 1e9
                         for left, right in zip(item["changes"], item["changes"][1:])]
            result[role] = {
                "http_200": item["responses"], "http_errors": dict(item["errors"]),
                "jpeg_hash_changes": len(item["changes"]),
                "observed_change_rate_hz_lower_bound": round(len(item["changes"]) / duration_seconds, 3),
                "change_interval_sec": distribution(intervals), "http_rtt_ms": distribution(item["rtt_ms"]),
            }
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--seconds", type=float, default=30)
    parser.add_argument("--web-url", help="可选：现有网页地址，例如 http://127.0.0.1:8765")
    parser.add_argument("--web-poll-seconds", type=float, default=0.2)
    parser.add_argument("--output", type=Path, help="可选 JSON 路径；默认只打印到标准输出")
    args = parser.parse_args()
    if args.seconds <= 0 or args.web_poll_seconds <= 0:
        parser.error("测量时长和网页轮询间隔必须大于 0")
    config = json.loads(args.config.read_text(encoding="utf-8"))
    streams = {role: RosStream(color) for role, (color, _depth) in zip(ROLES, camera_topics(config))}
    web = WebProbe(args.web_url, args.web_poll_seconds) if args.web_url else None
    web_thread = threading.Thread(target=web.run, daemon=True) if web else None
    started_wall_ns = time.time_ns()
    started_mono = time.monotonic()
    rclpy.init()
    node = rclpy.create_node("piper_preview_latency_probe")
    for stream in streams.values():
        node.create_subscription(Image, stream.topic, stream.on_image, qos_profile_sensor_data)
    try:
        if web_thread:
            web_thread.start()
        deadline = started_mono + args.seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(0.1, max(0, deadline - time.monotonic())))
    except KeyboardInterrupt:
        pass
    finally:
        if web_thread:
            web.stop.set()
            web_thread.join(timeout=3)
        node.destroy_node()
        rclpy.shutdown()
    duration = time.monotonic() - started_mono
    report = {
        "schema_version": "piper_preview_latency_probe_v1", "started_wall_ns": started_wall_ns,
        "duration_sec": round(duration, 3), "ros_rgb": {role: stream.summary() for role, stream in streams.items()},
        "web_jpeg_cache": web.summary(duration) if web else None,
        "measurement_note": ("Header→回调年龄包含相机采集、传输、DDS 与订阅调度，不等于 GUI 延迟。"
                             "JPEG 哈希变化率只是服务端缓存更新的可观测下界，不等于浏览器绘制率。"),
    }
    rendered = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if args.output:
        args.output.write_text(rendered, encoding="utf-8")
    else:
        print(rendered, end="")


if __name__ == "__main__":
    main()
