#!/usr/bin/env python3
"""合成 ROS 流验收本地控制台。运行：source /opt/ros/humble/setup.bash && source install/setup.bash && python3 tools/episode_recorder/test_desktop_console_integration.py。"""

import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time

os.environ["QT_QPA_PLATFORM"] = "offscreen"
os.environ["ROS_DOMAIN_ID"] = "198"

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from desktop_console import Cameras, Console, recorder_request

HERE = Path(__file__).parent
TOPICS = [f"/camera_{name}/color/image_raw" for name in ("wrist_left", "wrist_right", "top")]


def publish(node, stop):
    image = Image()
    image.width, image.height, image.step, image.encoding = 64, 48, 192, "rgb8"
    image.data = bytes((40, 100, 210)) * (image.width * image.height)
    images = [node.create_publisher(Image, topic, qos_profile_sensor_data) for topic in TOPICS]
    feedback = node.create_publisher(String, "/synthetic/feedback", 10)
    next_frame = time.monotonic()
    while not stop.is_set():
        image.header.stamp = node.get_clock().now().to_msg()
        for publisher in images:
            publisher.publish(image)
        feedback.publish(String(data="synthetic; no CAN device"))
        next_frame += 1 / 30
        stop.wait(max(0, next_frame - time.monotonic()))


def main():
    with tempfile.TemporaryDirectory(prefix="piper-desktop-test-") as temp:
        config_path = Path(temp) / "config.json"
        config = {"output_dir": str(Path(temp) / "episodes"),
                  "topics": [*TOPICS, "/synthetic/feedback"],
                  "frame_topics": TOPICS, "min_free_gb": 0, "startup_timeout_sec": 15}
        config_path.write_text(json.dumps(config))
        socket_path = Path(config["output_dir"]) / "recorder.sock"
        rclpy.init()
        app = QApplication([])
        source = rclpy.create_node("synthetic_desktop_source")
        cameras = Cameras(config)
        stop = threading.Event()
        feeder = threading.Thread(target=publish, args=(source, stop), daemon=True)
        spinner = threading.Thread(target=rclpy.spin, args=(cameras.node,), daemon=True)
        feeder.start(); spinner.start()
        recorder = subprocess.Popen([sys.executable, str(HERE / "episode_recorder.py"),
                                     "serve", "--config", str(config_path)])
        window = Console(config, cameras)
        window.show()
        stage = ["ready"]
        failure = []
        deadline = time.monotonic() + 35

        def step():
            nonlocal window
            try:
                if time.monotonic() > deadline:
                    raise AssertionError(f"超时：{stage[0]}，状态 {window.state}，提示 {window.message.text()}")
                if stage[0] == "ready" and window.state == "idle" and all(fps >= 25 for fps in window.display_fps):
                    window.task.setText("合成拾取")
                    window.instruction.setPlainText("拿起合成物体")
                    window.start.click(); stage[0] = "started"
                elif stage[0] == "started" and window.state == "recording":
                    window.close()
                    assert recorder_request(socket_path, {"action": "status"})["state"] == "recording"
                    window = Console(config, cameras); window.show(); stage[0] = "reopened"
                elif stage[0] == "reopened" and window.state == "recording":
                    window.stop.click(); stage[0] = "stopping"
                elif stage[0] == "stopping" and window.state == "stopped":
                    window.outcome.setCurrentIndex(1)
                    window.save.click(); stage[0] = "saving"
                elif stage[0] == "saving" and window.state == "idle":
                    episodes = recorder_request(socket_path, {"action": "list"})["episodes"]
                    assert len(episodes) == 1 and episodes[0]["outcome"] == "success"
                    assert episodes[0]["language_instruction"] == "拿起合成物体"
                    window.task.setText("丢弃测试")
                    window.instruction.setPlainText("测试")
                    window.start.click(); stage[0] = "discard_start"
                elif stage[0] == "discard_start" and window.state == "recording":
                    window.action("discard"); stage[0] = "discarding"
                elif stage[0] == "discarding" and window.state == "idle":
                    print(f"桌面预览帧率：{window.display_fps}；关闭窗口继续录制、保存与丢弃：通过")
                    app.quit()
            except Exception as exc:
                failure.append(exc)
                app.quit()

        timer = QTimer()
        timer.timeout.connect(step)
        timer.start(100)
        try:
            app.exec_()
            if failure:
                raise failure[0]
        finally:
            timer.stop(); window.close()
            stop.set(); feeder.join(timeout=2)
            if recorder.poll() is None:
                recorder.terminate()
                try: recorder.wait(timeout=8)
                except subprocess.TimeoutExpired: recorder.kill(); recorder.wait()
            rclpy.shutdown(); spinner.join(timeout=2)
            cameras.node.destroy_node(); source.destroy_node()


if __name__ == "__main__":
    main()
