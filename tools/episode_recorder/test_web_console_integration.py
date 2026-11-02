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
from web_console import ARM_TOPICS, start_request, teleoperation_ready

HERE = Path(__file__).parent
TOPICS = [f"/camera_{name}/color/image_raw" for name in ("wrist_left", "wrist_right", "top")]


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
        raise AssertionError(f"无效请求被接受：{path}")


def publisher(node, stop):
    image = Image()
    image.height, image.width, image.encoding, image.step = 24, 32, "rgb8", 96
    image.data = bytes([50, 100, 200]) * (image.height * image.width)
    images = [node.create_publisher(Image, topic, qos_profile_sensor_data) for topic in TOPICS]
    can = node.create_publisher(String, "/synthetic/can_feedback", 10)
    while not stop.is_set():
        image.header.stamp = node.get_clock().now().to_msg()
        for item in images:
            item.publish(image)
        can.publish(String(data="synthetic; no CAN device"))
        stop.wait(0.1)


def main():
    declared = {"topics": list(ARM_TOPICS["left"].values()),
                "teleoperation": {"source": "master_slave_teleoperation",
                                  "mapping_verified": True, "gripper_mapping_verified": True,
                                  "arms": {"left": ARM_TOPICS["left"]}}}
    assert teleoperation_ready(declared, "LEFT")
    assert not teleoperation_ready(declared, "RIGHT")
    verified_request = start_request({
        "mode": "skill", "skill": "pick_banana", "task": "合成抓取", "language_instruction": "抓取香蕉",
        "initial_conditions": {name: "人工设置" for name in
                               ("object", "box", "lid_angle", "left_arm", "right_arm", "perturbation_id")},
        "arm_mode": "LEFT", "calibration_version": "cal-v1", "normalization_version": "norm-v1"}, declared)
    assert {key: verified_request["metadata"][key] for key in
            ("arm_mode", "time_domain", "action_space", "action_provenance",
             "calibration_version", "normalization_version")} == {
                 "arm_mode": "LEFT", "time_domain": "unix_ns",
                 "action_space": "absolute_joint_position_7",
                 "action_provenance": "human_demonstration",
                 "calibration_version": "cal-v1", "normalization_version": "norm-v1"}
    with tempfile.TemporaryDirectory(prefix="piper-web-test-") as temp:
        root = Path(temp)
        config = root / "config.json"
        config.write_text(json.dumps({"output_dir": str(root / "episodes"),
                                      "topics": [*TOPICS, "/synthetic/can_feedback"],
                                      "frame_topics": TOPICS, "min_free_gb": 0,
                                      "startup_timeout_sec": 15}))
        with closing(socket.socket()) as probe:
            probe.bind(("127.0.0.1", 0))
            port = probe.getsockname()[1]
        os.environ.setdefault("ROS_DOMAIN_ID", "197")
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
            wait_for(lambda: all(item["online"] for item in http(port, "/api/status")["cameras"]))
            with urlopen(f"http://127.0.0.1:{port}/") as response:
                assert b"<!doctype html>" in response.read()
            for index in range(3):
                with urlopen(f"http://127.0.0.1:{port}/camera/{index}.jpg") as response:
                    assert response.read(3) == b"\xff\xd8\xff"
            start = http(port, "/api/start", {"task": "合成拾取", "language_instruction": "拿起测试物体"})
            assert start["result"]["state"] == "recording"
            assert start["result"]["metadata"]["mode"] == "task"
            web.terminate(); web.wait(timeout=5)
            time.sleep(1)
            assert recorder.poll() is None
            web = subprocess.Popen(web_command, env=env)
            wait_for(lambda: http(port, "/api/status")["result"]["state"] == "recording")
            wait_for(lambda: all(item["online"] for item in http(port, "/api/status")["cameras"]))
            rejected(port, "/api/annotate", {"kind": "skill_start", "value": "pick_banana",
                                             "camera_index": 9})
            started = http(port, "/api/annotate", {"kind": "skill_start", "value": "pick_banana",
                                                  "camera_index": 2})["result"]
            assert started["active_skill"] == "pick_banana"
            assert started["annotation"]["source_topic"] == TOPICS[2]
            assert started["annotation"]["header_stamp_ns"] > 0
            completed = http(port, "/api/annotate", {"kind": "skill_complete", "value": "pick_banana",
                                                    "camera_index": 2})["result"]
            assert completed["active_skill"] is None
            assert completed["annotation"]["source_topic"] == TOPICS[2]
            http(port, "/api/annotate", {"kind": "recovery_trigger", "value": "异常：夹爪打滑"})
            http(port, "/api/annotate", {"kind": "skill_start", "value": "place_banana_in_box"})
            unresolved = http(port, "/api/annotate", {"kind": "skill_complete",
                                                     "value": "place_banana_in_box",
                                                     "source_topic": TOPICS[2],
                                                     "header_stamp_ns": 123})["result"]
            assert unresolved["annotation"]["frame_resolution_required"] is True
            assert "source_topic" not in unresolved["annotation"]
            http(port, "/api/annotate", {"kind": "outcome", "value": "success"})
            assert len(http(port, "/api/status")["result"]["metadata"]["annotations"]) == 6
            assert http(port, "/api/stop", {})["result"]["state"] == "stopped"
            saved = http(port, "/api/save", {"outcome": "success"})["result"]
            assert saved["state"] == "saved"
            saved_meta = json.loads((Path(saved["path"]) / "episode.json").read_text())
            assert len(saved_meta["annotations"]) == 6
            assert saved_meta["annotations"][1]["kind"] == "skill_complete"
            entries = http(port, "/api/episodes")["result"]["episodes"]
            assert len(entries) == 1 and entries[0]["outcome"] == "success"
            assert entries[0]["language_instruction"] == "拿起测试物体"
            initial = {"object": "香蕉左偏 3 cm", "box": "盒子右移 2 cm", "lid_angle": "80°",
                       "left_arm": "左臂在起始位", "right_arm": "右臂在起始位", "perturbation_id": "scene_003"}
            rejected(port, "/api/start", {"mode": "skill", "task": "翻盖片段",
                                          "language_instruction": "双臂合上翻盖",
                                          "skill": "close_hinged_lid_bimanual"})
            assert http(port, "/api/contract")["teleoperation_ready"] == {
                "LEFT": False, "RIGHT": False, "BIMANUAL": False}
            rejected(port, "/api/start", {"mode": "skill", "task": "翻盖片段",
                                          "language_instruction": "双臂合上翻盖",
                                          "skill": "close_hinged_lid_bimanual",
                                          "initial_conditions": initial, "arm_mode": "LEFT",
                                          "calibration_version": "cal-v1", "normalization_version": "norm-v1"})
            rejected(port, "/api/start", {"mode": "skill", "task": "翻盖片段",
                                          "language_instruction": "双臂合上翻盖",
                                          "skill": "close_hinged_lid_bimanual",
                                          "initial_conditions": initial, "arm_mode": "BIMANUAL",
                                          "calibration_version": "UNVERIFIED_PLACEHOLDER",
                                          "normalization_version": "norm-v1"})
            skill = http(port, "/api/start", {"mode": "skill", "task": "翻盖片段",
                                               "language_instruction": "双臂合上翻盖",
                                               "skill": "close_hinged_lid_bimanual",
                                               "initial_conditions": initial, "arm_mode": "BIMANUAL",
                                               "calibration_version": "cal-v1",
                                               "normalization_version": "norm-v1"})["result"]
            assert skill["metadata"]["task_metadata"]["initial_conditions"] == {"source": "human", **initial}
            skill_contract = skill["metadata"]["task_metadata"]
            assert skill_contract["arm_mode"] == "BIMANUAL"
            assert skill_contract["calibration_version"] == "cal-v1"
            assert skill_contract["normalization_version"] == "norm-v1"
            assert skill_contract["collection_contract_status"] == "training_not_ready_unverified_teleoperation_mapping"
            assert all(field not in skill_contract for field in
                       ("time_domain", "action_space", "action_provenance"))
            assert skill["metadata"]["active_skill"] == "close_hinged_lid_bimanual"
            done = http(port, "/api/annotate", {"kind": "skill_complete",
                                               "value": "close_hinged_lid_bimanual",
                                               "camera_index": 0})["result"]
            assert done["annotation"]["source_topic"] == TOPICS[0]
            rejected(port, "/api/annotate", {"kind": "skill_complete",
                                             "value": "close_hinged_lid_bimanual"})
            http(port, "/api/annotate", {"kind": "recovery_trigger", "value": "恢复：重新扶正盒体"})
            http(port, "/api/annotate", {"kind": "outcome", "value": "failure"})
            assert http(port, "/api/stop", {})["result"]["state"] == "stopped"
            skill_saved = http(port, "/api/save", {"outcome": "failure"})["result"]
            assert skill_saved["state"] == "saved"
            saved_contract = json.loads((Path(skill_saved["path"]) / "episode.json").read_text())["task_metadata"]
            assert saved_contract == skill_contract
            assert len(http(port, "/api/episodes")["result"]["episodes"]) == 2
            assert http(port, "/api/start", {"task": "丢弃测试", "language_instruction": "测试"})["result"]["state"] == "recording"
            assert http(port, "/api/discard", {})["result"]["state"] == "discarded"
            assert http(port, "/api/status")["result"]["state"] == "idle"
            print("合成 ROS 流、旧任务兼容、技能采集契约、未验证映射训练未就绪、真实 Header 标注、重启与丢弃：通过")
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
