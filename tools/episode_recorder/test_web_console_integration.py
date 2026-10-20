#!/usr/bin/env python3
"""用合成 ROS 流验收采集控制闭环。运行：source /opt/ros/humble/setup.bash && source <install>/setup.bash && python3 tools/episode_recorder/test_web_console_integration.py。"""

from contextlib import closing
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import threading
import time
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

import rclpy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data

HERE = Path(__file__).parent
TOPICS = [f"/camera_{name}/color/image_raw" for name in ("wrist_left", "wrist_right", "top")]
DEPTH_TOPICS = [f"/camera_{name}/depth/image_raw" for name in ("wrist_left", "wrist_right")]
DEPTH_TOPICS.append("/camera_top/D455/aligned_depth_to_color/image_raw")


def wait_for(check, timeout=25):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            result = check()
            if result:
                return result
        except (OSError, URLError):
            pass
        time.sleep(0.2)
    raise AssertionError("等待服务或数据超时")


def http(port, path, data=None):
    payload = json.dumps(data).encode() if data is not None else None
    request = Request(f"http://127.0.0.1:{port}{path}", data=payload,
                      headers={"Content-Type": "application/json"} if payload else {})
    with urlopen(request, timeout=30) as response:
        return json.load(response)


def rejected(port, path, data):
    try:
        http(port, path, data)
    except HTTPError as error:
        assert error.code in (400, 409), error.code
        assert json.load(error)["error"]
    else:
        raise AssertionError(f"无效输入被接受: {path}")


def publisher(node, stop):
    image = Image()
    image.height, image.width, image.encoding, image.step = 24, 32, "rgb8", 96
    image.data = bytes([50, 100, 200]) * (image.height * image.width)
    images = [node.create_publisher(Image, topic, qos_profile_sensor_data) for topic in TOPICS]
    depth = Image()
    depth.height, depth.width, depth.encoding, depth.step = 24, 32, "16UC1", 64
    depth.data = (700).to_bytes(2, "little") * (depth.height * depth.width)
    depths = [node.create_publisher(Image, topic, qos_profile_sensor_data) for topic in DEPTH_TOPICS]
    can = node.create_publisher(String, "/synthetic/can_feedback", 10)
    while not stop.is_set():
        image.header.stamp = node.get_clock().now().to_msg()
        for item in images:
            item.publish(image)
        depth.header.stamp = image.header.stamp
        for item in depths:
            item.publish(depth)
        can.publish(String(data="synthetic; no CAN device"))
        stop.wait(0.1)


def main():
    with tempfile.TemporaryDirectory(prefix="piper-web-test-") as temp:
        root = Path(temp)
        config = root / "config.json"
        config.write_text(json.dumps({"output_dir": str(root / "episodes"),
                                      "topics": [*TOPICS, *DEPTH_TOPICS, "/synthetic/can_feedback"],
                                      "frame_topics": [*TOPICS, *DEPTH_TOPICS], "min_free_gb": 0,
                                      "startup_timeout_sec": 15}))
        with closing(socket.socket()) as probe:
            probe.bind(("127.0.0.1", 0))
            port = probe.getsockname()[1]
        os.environ["ROS_DOMAIN_ID"] = "197"
        env = dict(os.environ)
        rclpy.init()
        node = rclpy.create_node("synthetic_collection_source")
        stop = threading.Event()
        feeder = threading.Thread(target=publisher, args=(node, stop), daemon=True)
        feeder.start()
        recorder = subprocess.Popen([sys.executable, str(HERE / "episode_recorder.py"), "serve", "--config", str(config)], env=env)
        web_command = [sys.executable, str(HERE / "web_console.py"), "--config", str(config), "--port", str(port)]
        web = subprocess.Popen(web_command, env=env)
        try:
            wait_for(lambda: http(port, "/api/status")["ok"])
            def online_cameras():
                items = http(port, "/api/status")["cameras"]
                return items if all(item["color"]["online"] and item["depth"]["online"]
                                    for item in items) else None
            cameras = wait_for(online_cameras)
            assert [item["depth"]["max_meters"] for item in cameras] == [0.5, 0.5, 1.0]
            with urlopen(f"http://127.0.0.1:{port}/") as response:
                assert b"<!doctype html>" in response.read()
            for index in range(3):
                for kind in ("color", "depth"):
                    with urlopen(f"http://127.0.0.1:{port}/camera/{kind}/{index}.jpg") as response:
                        assert response.read(3) == b"\xff\xd8\xff"
            start = http(port, "/api/start", {"task": "合成拾取", "language_instruction": "拿起测试物体",
                                                "duration_sec": 3})
            assert start["result"]["state"] == "recording"
            web.terminate(); web.wait(timeout=5)
            time.sleep(1)
            assert recorder.poll() is None
            web = subprocess.Popen(web_command, env=env)
            wait_for(lambda: http(port, "/api/status")["result"]["state"] == "stopped")
            assert http(port, "/api/status")["result"]["metadata"]["stop_reason"] == "duration_limit"
            selected_dir = root / "selected_episodes"
            saved = http(port, "/api/save", {"outcome": "success", "save_dir": str(selected_dir)})["result"]
            assert saved["state"] == "saved"
            assert Path(saved["path"]).parent == selected_dir
            entries = http(port, "/api/episodes")["result"]["episodes"]
            assert len(entries) == 1 and entries[0]["outcome"] == "success"
            assert entries[0]["language_instruction"] == "拿起测试物体"
            assert http(port, "/api/start", {"task": "丢弃测试", "language_instruction": "测试",
                                             "duration_sec": 30})["result"]["state"] == "recording"
            assert http(port, "/api/stop", {})["result"]["state"] == "stopped"
            assert http(port, "/api/discard", {})["result"]["state"] == "discarded"
            assert http(port, "/api/status")["result"]["state"] == "idle"
            rejected(port, "/api/plan_create", {"long_task_description": "测试", "steps": []})
            plan = http(port, "/api/plan_create", {
                "long_task_description": "把杯子放入盒子", "object_ids": ["cup_1", "box_1"],
                "location_ids": ["table_1", "box_slot"],
                "steps": [{"skill": "grasp", "target_object_id": "cup_1",
                           "completion_criteria": "杯子离开桌面"},
                          {"skill": "place", "target_object_id": "cup_1",
                           "target_location_id": "box_slot", "completion_criteria": "杯子位于盒内"}],
            })["result"]
            assert plan["plan_id"] and plan["task_group_id"]
            assert len(http(port, "/api/plans")["result"]["plans"]) == 1
            context = {"plan_id": plan["plan_id"], "long_task_description": "把杯子放入盒子",
                       "skill_instruction": "抓住杯子", "target_object_id": "cup_1",
                       "target_location_id": "", "expected_state_change": "杯子由桌面变为夹爪持有",
                       "completion_criteria": "杯子离开桌面"}
            rejected(port, "/api/start", {"task": "错配规划", "skill": "grasp",
                                           "duration_sec": 30,
                                           "vlm": {**context, "target_object_id": "other_cup"}})
            rejected(port, "/api/start", {"task": "无效技能", "skill": "grasp",
                                           "duration_sec": 30, "vlm": {**context, "completion_criteria": ""}})
            started = http(port, "/api/start", {"task": "杯子任务", "skill": "grasp",
                                                  "duration_sec": 30, "vlm": context})["result"]
            assert started["metadata"]["task_metadata"]["vlm"]["task_group_id"] == plan["task_group_id"]
            assert http(port, "/api/stop", {})["result"]["state"] == "stopped"
            rejected(port, "/api/save", {"outcome": "success", "save_dir": str(selected_dir)})
            annotation = {"outcome": "success", "observed_state_change": "杯子已抬起",
                          "next_decision": "continue", "next_skill": "place"}
            success = http(port, "/api/save", {"outcome": "success", "save_dir": str(selected_dir),
                                                 "annotation": annotation})["result"]
            success_meta = json.loads((Path(success["path"]) / "episode.json").read_text())
            assert success_meta["vlm_annotation"]["source"] == "human"
            assert success_meta["vlm_annotation"]["failure_reason"] is None
            assert success_meta["vlm_annotation"]["annotated_at_unix_ns"] > 0
            assert success_meta["task_metadata"]["vlm"]["plan_id"] == plan["plan_id"]
            assert success_meta["episode_id"] and success_meta["bag_validation"]["messages_per_topic"]
            assert (Path(success["path"]) / "bag" / "metadata.yaml").is_file()
            unknown_context = {**context, "plan_id": "", "target_object_id": "",
                               "target_location_id": ""}
            assert http(port, "/api/start", {"task": "未知结果", "skill": "observe",
                                             "duration_sec": 30, "vlm": unknown_context})["result"]["state"] == "recording"
            assert http(port, "/api/stop", {})["result"]["state"] == "stopped"
            unknown = http(port, "/api/save", {"outcome": "unknown", "save_dir": str(selected_dir),
                                                 "annotation": {"outcome": "unknown", "next_decision": "reobserve"}})["result"]
            unknown_meta = json.loads((Path(unknown["path"]) / "episode.json").read_text())
            assert unknown_meta["vlm_annotation"]["observed_state_change"] is None
            assert unknown_meta["task_metadata"]["vlm"]["target_object_id"] is None
            assert http(port, "/api/start", {"task": "失败结果", "skill": "grasp",
                                             "duration_sec": 30, "vlm": context})["result"]["state"] == "recording"
            assert http(port, "/api/stop", {})["result"]["state"] == "stopped"
            rejected(port, "/api/save", {"outcome": "failure", "save_dir": str(selected_dir),
                                           "annotation": {"outcome": "failure", "next_decision": "ask_human"}})
            failed = http(port, "/api/save", {"outcome": "failure", "save_dir": str(selected_dir),
                                                "annotation": {"outcome": "failure",
                                                               "failure_reason": "unable_to_determine",
                                                               "next_decision": "ask_human"}})["result"]
            assert Path(failed["path"]).is_dir()
            web.terminate(); web.wait(timeout=5)
            recorder.terminate(); recorder.wait(timeout=8)
            recorder = subprocess.Popen([sys.executable, str(HERE / "episode_recorder.py"), "serve", "--config", str(config)], env=env)
            web = subprocess.Popen(web_command, env=env)
            wait_for(lambda: http(port, "/api/status")["ok"])
            entries = http(port, "/api/episodes")["result"]["episodes"]
            assert len(entries) == 4 and any(item["vlm"] is None for item in entries)
            assert len(http(port, "/api/plans")["result"]["plans"]) == 1
            tampered = json.loads(json.dumps(success_meta))
            tampered["task_metadata"]["vlm"]["target_object_id"] = "other_cup"
            Path(success["path"]).joinpath("episode.json").write_text(
                json.dumps(tampered, ensure_ascii=False), encoding="utf-8")
            rejected(port, "/api/vlm/export", None)
            Path(success["path"]).joinpath("episode.json").write_text(
                json.dumps(success_meta, ensure_ascii=False), encoding="utf-8")
            exported = http(port, "/api/vlm/export")["result"]
            if output := os.environ.get("VLM_EXPORT_TEST_OUTPUT"):
                Path(output).write_text(json.dumps(exported, ensure_ascii=False, indent=2), encoding="utf-8")
            assert len(exported["planning_samples"]) == 1
            assert len(exported["transition_samples"]) == 3
            assert {item["human_annotation"]["outcome"] for item in exported["transition_samples"]} == {"success", "failure", "unknown"}
            for item in exported["transition_samples"]:
                assert item["episode_id"] and item["started_at_unix_ns"]
                assert item["rosbag_path"] and item["topics"] and item["bag_validation"]
            print("合成 ROS 流、旧 Episode、VLM 单技能三种结果、规划、重启读取及导出：通过")
        finally:
            stop.set(); feeder.join(timeout=2)
            for process in (web, recorder):
                if process.poll() is None:
                    process.terminate()
                    try: process.wait(timeout=8)
                    except subprocess.TimeoutExpired: process.kill(); process.wait()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
