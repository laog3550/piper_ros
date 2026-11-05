#!/usr/bin/env python3
"""合成 ROS 流验收本地控制台。运行：source /opt/ros/humble/setup.bash && source install/setup.bash && python3 tools/episode_recorder/test_desktop_console_integration.py。"""

import json
import os
from pathlib import Path
import socket
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
from piper_msgs.msg import PiperArmVector

from desktop_console import Cameras, Console, recorder_request
from skill_web_console import provisional_collection_versions

HERE = Path(__file__).parent
TOPICS = [f"/camera_{name}/color/image_raw" for name in ("wrist_left", "wrist_right", "top")]
DEPTH_TOPICS = [f"/camera_{name}/depth/image_raw" for name in ("wrist_left", "wrist_right")]
DEPTH_TOPICS.append("/camera_top/D455/aligned_depth_to_color/image_raw")


def publish(node, stop):
    image = Image()
    image.width, image.height, image.step, image.encoding = 64, 48, 192, "rgb8"
    image.data = bytes((40, 100, 210)) * (image.width * image.height)
    images = [node.create_publisher(Image, topic, qos_profile_sensor_data) for topic in TOPICS]
    depth = Image()
    depth.width, depth.height, depth.step, depth.encoding = 64, 48, 128, "16UC1"
    depth.data = (500).to_bytes(2, "little") * (depth.width * depth.height)
    depths = [node.create_publisher(Image, topic, qos_profile_sensor_data) for topic in DEPTH_TOPICS]
    feedback = node.create_publisher(String, "/synthetic/feedback", 10)
    joint_publishers = {arm: node.create_publisher(PiperArmVector, f"/piper_vector/{arm}/slave_feedback", 10)
                        for arm in ("left", "right")}
    next_frame = time.monotonic()
    while not stop.is_set():
        image.header.stamp = node.get_clock().now().to_msg()
        for publisher in images:
            publisher.publish(image)
        depth.header.stamp = image.header.stamp
        for publisher in depths:
            publisher.publish(depth)
        feedback.publish(String(data="synthetic; no CAN device"))
        for arm, publisher in joint_publishers.items():
            vector = PiperArmVector()
            vector.source_topic = f"/piper_direct/{arm}/slave_feedback"
            vector.position = [0.1] * 6 + [0.02]
            vector.source_stamps = [image.header.stamp] * 4
            publisher.publish(vector)
        next_frame += 1 / 30
        stop.wait(max(0, next_frame - time.monotonic()))


def main():
    with tempfile.TemporaryDirectory(prefix="piper-desktop-socket-test-") as temp:
        path = Path(temp) / "recorder.sock"
        with socket.socket(socket.AF_UNIX) as server:
            server.bind(str(path))
            server.listen(1)
            def reply():
                connection, _ = server.accept()
                with connection:
                    connection.recv(1024)
                    connection.sendall(json.dumps({"ok": True, "result": {"text": "x" * 1_050_000}}).encode())
            responder = threading.Thread(target=reply)
            responder.start()
            assert len(recorder_request(path, {"action": "status"})["text"]) == 1_050_000
            responder.join(timeout=5)
    with (tempfile.TemporaryDirectory(prefix="piper-desktop-test-") as temp,
          tempfile.TemporaryDirectory(prefix="piper-save-test-", dir="/dev/shm") as target):
        target_dir = Path(target)
        config_path = Path(temp) / "config.json"
        config = {"output_dir": str(Path(temp) / "episodes"),
                  "topics": [*TOPICS, *DEPTH_TOPICS, "/synthetic/feedback",
                             "/piper_vector/left/slave_feedback", "/piper_vector/right/slave_feedback"],
                  "frame_topics": [*TOPICS, *DEPTH_TOPICS], "min_free_gb": 0, "startup_timeout_sec": 15}
        config_path.write_text(json.dumps(config))
        socket_path = Path(config["output_dir"]) / "recorder.sock"
        rclpy.init()
        app = QApplication([])
        app.setQuitOnLastWindowClosed(False)
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
        raw_versions = provisional_collection_versions()
        assert window.calibration.text() == raw_versions["calibration_version"]
        assert window.normalization.text() == raw_versions["normalization_version"]
        stage = ["ready"]
        closed_at = [None]
        failure = []
        preview_p95_ms = [None]
        deadline = time.monotonic() + 35

        def step():
            nonlocal window
            try:
                if time.monotonic() > deadline:
                    raise AssertionError(f"超时：{stage[0]}，状态 {window.state}，提示 {window.message.text()}")
                if stage[0] == "ready" and window.state == "idle" and all(fps >= 25 for fps in window.display_fps):
                    assert all(image.pixmap() for image in window.depth_images)
                    preview_p95_ms[0] = [round(sorted(samples)[int(0.95 * (len(samples) - 1))], 1)
                                          for samples in window.display_latency_samples]
                    assert max(preview_p95_ms[0]) < 150, preview_p95_ms[0]
                    window.task.setText("合成拾取")
                    window.instruction.setPlainText("拿起合成物体")
                    window.duration.setValue(5)
                    window.start.click(); stage[0] = "started"
                elif stage[0] == "started" and window.state == "recording":
                    window.mark_start.click(); stage[0] = "task_mark_started"
                elif stage[0] == "task_mark_started" and window.active_skill == "pick_and_place_banana_in_box":
                    window.mark_complete.click(); stage[0] = "task_mark_completed"
                elif stage[0] == "task_mark_completed" and len(window.annotations) >= 2:
                    window.mark_exception.click(); stage[0] = "task_exception"
                elif stage[0] == "task_exception" and len(window.annotations) >= 3:
                    window.mark_recovery.click(); stage[0] = "task_recovery"
                elif stage[0] == "task_recovery" and len(window.annotations) >= 4:
                    window.close()
                    assert recorder_request(socket_path, {"action": "status"})["state"] == "recording"
                    closed_at[0] = time.monotonic()
                    stage[0] = "window_closed"
                elif stage[0] == "window_closed" and time.monotonic() - closed_at[0] >= 6:
                    assert recorder_request(socket_path, {"action": "status"})["state"] == "stopped"
                    window = Console(config, cameras); window.show(); stage[0] = "reopened"
                elif stage[0] == "reopened" and window.state == "stopped":
                    metadata = recorder_request(socket_path, {"action": "status"})["metadata"]
                    assert metadata["stop_reason"] == "duration_limit"
                    assert metadata["recorded_duration_sec"] >= 5
                    assert set(metadata["task_metadata"]["initial_conditions"]["joint_positions"]) == {"left", "right"}
                    assert [a["kind"] for a in metadata["annotations"]] == [
                        "skill_start", "skill_complete", "recovery_trigger", "recovery_trigger"]
                    collision = target_dir / "episode_000001"
                    collision.mkdir()
                    try:
                        recorder_request(socket_path, {"action": "save", "outcome": "success",
                                                       "save_dir": str(target_dir)})
                        raise AssertionError("现有 Episode 目录未被拒绝")
                    except ValueError as exc:
                        assert "已存在" in str(exc)
                    assert recorder_request(socket_path, {"action": "status"})["state"] == "stopped"
                    collision.rmdir()
                    window.save_dir.setText(str(target_dir))
                    window.outcome.setCurrentIndex(1)
                    window.save.click(); stage[0] = "saving"
                elif stage[0] == "saving" and window.state == "idle":
                    episodes = recorder_request(socket_path, {"action": "list"})["episodes"]
                    assert len(episodes) == 1 and episodes[0]["outcome"] == "success"
                    assert episodes[0]["language_instruction"] == "拿起合成物体"
                    assert episodes[0]["stop_reason"] == "duration_limit"
                    assert Path(episodes[0]["path"]).parent == target_dir
                    assert (Path(episodes[0]["path"]) / "bag" / "metadata.yaml").is_file()
                    window.mode.setCurrentIndex(1)
                    window.skill.setCurrentIndex(1)
                    assert window.arm_mode.currentData() == "BIMANUAL" and not window.arm_mode.isEnabled()
                    window.skill.setCurrentIndex(0)
                    assert window.arm_mode.currentData() == "LEFT" and window.arm_mode.isEnabled()
                    window.arm_mode.setCurrentIndex(0)
                    window.calibration.setText("synthetic-camera-v1")
                    window.normalization.setText("synthetic-norm-v1")
                    window.perturbation.setText("synthetic-scene-1")
                    window.task.setText("合成抓取")
                    window.instruction.setPlainText("抓取合成物体")
                    window.duration.setValue(3)
                    window.camera_ref.setCurrentIndex(3)
                    window.start.click(); stage[0] = "skill_started"
                elif stage[0] == "skill_started" and window.state == "recording":
                    window.mark_complete.click(); stage[0] = "skill_completed"
                elif stage[0] == "skill_completed" and len(window.annotations) >= 1:
                    window.mark_success.click(); stage[0] = "skill_labeled"
                elif stage[0] == "skill_labeled" and window.state == "stopped":
                    meta = recorder_request(socket_path, {"action": "status"})["metadata"]
                    assert meta["mode"] == "skill" and meta["skill"] == "pick_and_place_banana_in_box"
                    assert meta["task_metadata"]["initial_conditions"]["perturbation_id"] == "synthetic-scene-1"
                    assert meta["task_metadata"]["initial_conditions"]["joint_positions"]["left"]["position_rad"]
                    assert meta["task_metadata"]["collection_contract_status"] == "training_not_ready_unverified_teleoperation_mapping"
                    assert any(a["kind"] == "skill_complete" and a.get("source_topic") == TOPICS[2]
                               for a in meta["annotations"])
                    window.outcome.setCurrentIndex(1)
                    window.save.click(); stage[0] = "skill_saving"
                elif stage[0] == "skill_saving" and window.state == "idle":
                    window.task.setText("丢弃测试")
                    window.instruction.setPlainText("测试")
                    window.mode.setCurrentIndex(0)
                    window.start.click(); stage[0] = "discard_start"
                elif stage[0] == "discard_start" and window.state == "recording":
                    window.action("discard"); stage[0] = "discarding"
                elif stage[0] == "discarding" and window.state == "idle":
                    print(f"桌面预览帧率：{window.display_fps}；接收至显示延迟 p95：{preview_p95_ms[0]} ms；"
                          "技能标注、关闭窗口继续录制、保存与丢弃：通过")
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
            legacy = recorder_request(socket_path, {"action": "start", "mode": "task",
                                                    "task": "旧接口", "skill": None, "metadata": {}})
            assert legacy["state"] == "recording" and legacy["remaining_sec"] is None
            assert recorder_request(socket_path, {"action": "stop"})["metadata"]["stop_reason"] == "manual"
            recorder_request(socket_path, {"action": "discard"})
            recorder.terminate(); recorder.wait(timeout=8)
            recorder = subprocess.Popen([sys.executable, str(HERE / "episode_recorder.py"),
                                         "serve", "--config", str(config_path)])
            deadline = time.monotonic() + 8
            while time.monotonic() < deadline:
                try:
                    episodes = recorder_request(socket_path, {"action": "list"})["episodes"]
                    break
                except OSError:
                    time.sleep(0.1)
            else:
                raise AssertionError("重启录制器后无法查询 Episode")
            assert len(episodes) == 2 and all(Path(item["path"]).parent == target_dir for item in episodes)
            print("重启录制器后仍能列出跨磁盘保存的 Episode：通过")
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
