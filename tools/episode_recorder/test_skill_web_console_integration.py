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
from piper_msgs.msg import PiperArmVector
from rclpy.qos import qos_profile_sensor_data
from skill_web_console import (ARM_TOPICS, Preview, recorder_request, start_request,
                               teleoperation_ready, provisional_collection_versions)

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
    joints = {arm: node.create_publisher(PiperArmVector, ARM_TOPICS[arm]["slave_feedback"], 10)
              for arm in ("left", "right")}
    while not stop.is_set():
        image.header.stamp = node.get_clock().now().to_msg()
        for item in images:
            item.publish(image)
        can.publish(String(data="synthetic; no CAN device"))
        for arm, item in joints.items():
            joint = PiperArmVector()
            joint.header.stamp = image.header.stamp
            joint.source_topic = f"/piper_direct/{arm}/slave_feedback"
            joint.source_stamps = [image.header.stamp] * 4
            joint.position = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.01]
            item.publish(joint)
        stop.wait(0.1)


def main():
    preview = object.__new__(Preview)
    preview.lock = threading.Lock()
    preview.frames, preview.errors, preview.times = [None] * 3, [None] * 3, [0.0] * 3
    preview.header_stamps, preview.source_times = [None] * 3, [0.0] * 3
    preview.topics, preview.frame_topics = TOPICS, set(TOPICS)
    frame = Image()
    frame.width, frame.height, frame.step, frame.encoding = 2, 2, 6, "rgb8"
    frame.data = bytes([10, 20, 30] * 4)
    frame.header.stamp.sec = 1_700_000_000
    Preview.on_image(preview, 0, frame)
    first_stamp = preview.header_stamps[0]
    frame.header.stamp.nanosec = 100_000_000
    Preview.on_image(preview, 0, frame)
    assert preview.header_stamps[0] == first_stamp
    assert Preview.latest_reference(preview, 0) == (TOPICS[0], first_stamp)
    with tempfile.TemporaryDirectory(prefix="piper-socket-test-") as temp:
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
            assert len(recorder_request(path, {"action": "status"})["result"]["text"]) == 1_050_000
            responder.join(timeout=5)
    class FakePreview:
        def snapshot(self):
            return [{"name": name, "online": True} for name in ("左腕", "右腕", "顶部")]

        def initial_joint_positions(self, required_arms):
            return {arm: {"position_rad": [0.1] * 6} for arm in required_arms}

    class MissingCamera(FakePreview):
        def snapshot(self):
            return [{"name": "左腕", "online": False}]

    try:
        start_request({"task": "合成任务", "language_instruction": "测试"}, {}, MissingCamera())
    except ValueError as error:
        assert "相机预览未就绪" in str(error)
    else:
        raise AssertionError("无相机时仍允许开始录制")

    declared = {"topics": list(ARM_TOPICS["left"].values()),
                "teleoperation": {"source": "master_slave_teleoperation",
                                  "mapping_verified": True, "gripper_mapping_verified": True,
                                  "arms": {"left": ARM_TOPICS["left"]}}}
    assert teleoperation_ready(declared, "LEFT")
    assert not teleoperation_ready(declared, "RIGHT")
    verified_request = start_request({
        "mode": "skill", "skill": "pick_and_place_banana_in_box", "task": "合成抓取", "language_instruction": "抓取香蕉",
        "perturbation_id": "scene_001",
        "arm_mode": "LEFT", "calibration_version": "cal-v1", "normalization_version": "norm-v1"}, declared, FakePreview())
    assert {key: verified_request["metadata"][key] for key in
            ("arm_mode", "time_domain", "action_space", "action_provenance",
             "calibration_version", "normalization_version")} == {
                 "arm_mode": "LEFT", "time_domain": "unix_ns",
                 "action_space": "absolute_joint_position_7",
                 "action_provenance": "human_demonstration",
                 "calibration_version": "cal-v1", "normalization_version": "norm-v1"}
    raw_versions = provisional_collection_versions()
    assert set(raw_versions) == {"calibration_version", "normalization_version"}
    raw_request = start_request({
        "mode": "skill", "skill": "pick_and_place_banana_in_box", "task": "原始香蕉入盒",
        "language_instruction": "抓起香蕉并放入盒中", "perturbation_id": "scene_001",
        "arm_mode": "LEFT", **raw_versions}, declared, FakePreview())
    assert raw_request["metadata"]["collection_contract_status"] == "training_not_ready_provisional_collection_versions"
    assert "action_provenance" not in raw_request["metadata"]
    assert set(raw_request["metadata"]["collection_version_evidence"]) == set(raw_versions)
    with tempfile.TemporaryDirectory(prefix="piper-web-test-") as temp:
        root = Path(temp)
        config = root / "config.json"
        config.write_text(json.dumps({"output_dir": str(root / "episodes"),
                                      "topics": [*TOPICS, "/synthetic/can_feedback",
                                                 ARM_TOPICS["left"]["slave_feedback"],
                                                 ARM_TOPICS["right"]["slave_feedback"]],
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
        web_command = [sys.executable, str(HERE / "skill_web_console.py"), "--config", str(config), "--port", str(port)]
        web = subprocess.Popen(web_command, env=env)
        try:
            wait_for(lambda: http(port, "/api/status")["ok"])
            wait_for(lambda: all(item["online"] for item in http(port, "/api/status")["cameras"]))
            wait_for(lambda: all(item["online"] for item in http(port, "/api/status")["joints"].values()))
            with urlopen(f"http://127.0.0.1:{port}/") as response:
                page = response.read().decode()
                assert "采集数据的训练用途" in page
                assert "π0.5（Pi）动作策略" in page and "Qwen 规划与结果判断" in page
                assert 'id="duration_sec"' in page and 'id="save_dir"' in page
                assert 'id="quality_warning"' in page
            for index in range(3):
                with urlopen(f"http://127.0.0.1:{port}/camera/{index}.jpg") as response:
                    assert response.read(3) == b"\xff\xd8\xff"
            start = http(port, "/api/start", {"task": "合成拾取", "language_instruction": "拿起测试物体",
                                               "duration_sec": 60})
            assert start["result"]["state"] == "recording"
            assert start["result"]["metadata"]["mode"] == "task"
            assert start["result"]["metadata"]["duration_limit_sec"] == 60
            assert start["result"]["metadata"]["rosbag_cache_bytes"] == 268_435_456
            captured = start["result"]["metadata"]["task_metadata"]["initial_conditions"]
            assert captured["source"] == "ros_slave_feedback"
            assert set(captured["joint_positions"]) == {"left", "right"}
            assert captured["joint_positions"]["left"]["position_rad"] == [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
            web.terminate(); web.wait(timeout=5)
            time.sleep(1)
            assert recorder.poll() is None
            web = subprocess.Popen(web_command, env=env)
            wait_for(lambda: http(port, "/api/status")["result"]["state"] == "recording")
            wait_for(lambda: all(item["online"] for item in http(port, "/api/status")["cameras"]))
            rejected(port, "/api/annotate", {"kind": "skill_start", "value": "pick_and_place_banana_in_box",
                                             "camera_index": 9})
            started = http(port, "/api/annotate", {"kind": "skill_start", "value": "pick_and_place_banana_in_box",
                                                  "camera_index": 2})["result"]
            assert started["active_skill"] == "pick_and_place_banana_in_box"
            assert started["annotation"]["source_topic"] == TOPICS[2]
            assert started["annotation"]["header_stamp_ns"] > 0
            completed = http(port, "/api/annotate", {"kind": "skill_complete", "value": "pick_and_place_banana_in_box",
                                                    "camera_index": 2, "skill_result": "success"})["result"]
            assert completed["active_skill"] is None
            assert completed["annotation"]["source_topic"] == TOPICS[2]
            assert completed["annotation"]["skill_result"] == "success"
            http(port, "/api/annotate", {"kind": "recovery_trigger", "value": "异常：夹爪打滑"})
            http(port, "/api/annotate", {"kind": "skill_start", "value": "flip_box_lid_bimanual"})
            rejected(port, "/api/annotate", {"kind": "skill_complete",
                                             "value": "flip_box_lid_bimanual", "camera_index": 2})
            http(port, "/api/annotate", {"kind": "skill_complete",
                                           "value": "flip_box_lid_bimanual", "camera_index": 2,
                                           "skill_result": "success"})
            http(port, "/api/annotate", {"kind": "skill_start", "value": "latch_box_lid_bimanual"})
            http(port, "/api/annotate", {"kind": "skill_complete", "value": "latch_box_lid_bimanual",
                                          "camera_index": 2, "skill_result": "success"})
            assert len(http(port, "/api/status")["result"]["metadata"]["annotations"]) == 7
            assert http(port, "/api/stop", {})["result"]["state"] == "stopped"
            custom_save = root / "manual_destination"
            saved = http(port, "/api/save", {"outcome": "success",
                                              "save_dir": str(custom_save)})["result"]
            assert saved["state"] == "saved"
            assert Path(saved["path"]).parent == custom_save
            saved_meta = json.loads((Path(saved["path"]) / "episode.json").read_text())
            assert len(saved_meta["annotations"]) == 7
            assert saved_meta["annotations"][1]["kind"] == "skill_complete"
            assert all(item["verified_in_bag"] for item in saved_meta["annotations"]
                       if item["kind"] == "skill_complete")
            entries = http(port, "/api/episodes")["result"]["episodes"]
            assert len(entries) == 1 and entries[0]["outcome"] == "success"
            assert entries[0]["language_instruction"] == "拿起测试物体"
            rejected(port, "/api/start", {"mode": "skill", "task": "翻盖片段",
                                          "language_instruction": "双臂合上翻盖",
                                          "skill": "latch_box_lid_bimanual"})
            assert http(port, "/api/contract")["teleoperation_ready"] == {
                "LEFT": False, "RIGHT": False, "BIMANUAL": False}
            rejected(port, "/api/start", {"mode": "skill", "task": "翻盖片段",
                                          "language_instruction": "双臂合上翻盖",
                                          "skill": "latch_box_lid_bimanual",
                                          "perturbation_id": "scene_003", "arm_mode": "LEFT",
                                          "calibration_version": "cal-v1", "normalization_version": "norm-v1"})
            rejected(port, "/api/start", {"mode": "skill", "task": "翻盖片段",
                                          "language_instruction": "双臂翻过盒盖",
                                          "skill": "flip_box_lid_bimanual",
                                          "perturbation_id": "scene_003", "arm_mode": "LEFT",
                                          "calibration_version": "cal-v1", "normalization_version": "norm-v1"})
            rejected(port, "/api/start", {"mode": "skill", "task": "翻盖片段",
                                          "language_instruction": "双臂合上翻盖",
                                          "skill": "latch_box_lid_bimanual",
                                          "perturbation_id": "scene_003", "arm_mode": "BIMANUAL",
                                          "calibration_version": "UNVERIFIED_PLACEHOLDER",
                                          "normalization_version": "norm-v1"})
            skill = http(port, "/api/start", {"mode": "skill", "task": "翻盖片段",
                                               "language_instruction": "双臂合上翻盖",
                                               "skill": "latch_box_lid_bimanual",
                                               "perturbation_id": "scene_003", "arm_mode": "BIMANUAL",
                                               "calibration_version": "cal-v1",
                                               "normalization_version": "norm-v1"})["result"]
            initial = skill["metadata"]["task_metadata"]["initial_conditions"]
            assert initial["source"] == "ros_slave_feedback"
            assert initial["perturbation_id"] == "scene_003"
            assert set(initial["joint_positions"]) == {"left", "right"}
            skill_contract = skill["metadata"]["task_metadata"]
            assert skill_contract["arm_mode"] == "BIMANUAL"
            assert skill_contract["calibration_version"] == "cal-v1"
            assert skill_contract["normalization_version"] == "norm-v1"
            assert skill_contract["collection_contract_status"] == "training_not_ready_unverified_teleoperation_mapping"
            assert all(field not in skill_contract for field in
                       ("time_domain", "action_space", "action_provenance"))
            assert skill["metadata"]["active_skill"] == "latch_box_lid_bimanual"
            rejected(port, "/api/annotate", {"kind": "skill_complete",
                                             "value": "latch_box_lid_bimanual", "camera_index": 0})
            http(port, "/api/annotate", {"kind": "recovery_trigger", "value": "恢复：重新扶正盒体"})
            time.sleep(0.7)
            assert http(port, "/api/stop", {})["result"]["state"] == "stopped"
            done = http(port, "/api/annotate", {"kind": "skill_complete",
                                               "value": "latch_box_lid_bimanual",
                                               "camera_index": 0, "review_offset_sec": 0.5})["result"]
            assert done["annotation"]["source_topic"] == TOPICS[0]
            assert done["annotation"]["annotation_origin"] == "post_recording_review"
            rejected(port, "/api/annotate", {"kind": "skill_complete",
                                             "value": "latch_box_lid_bimanual"})
            skill_saved = http(port, "/api/save", {"outcome": "failure"})["result"]
            assert skill_saved["state"] == "saved"
            saved_meta = json.loads((Path(skill_saved["path"]) / "episode.json").read_text())
            saved_contract = saved_meta["task_metadata"]
            assert saved_contract == skill_contract
            outcomes = [item for item in saved_meta["annotations"] if item["kind"] == "outcome"]
            assert len(outcomes) == 1 and outcomes[0]["value"] == "failure"
            assert outcomes[0]["annotation_origin"] == "save_selection"
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
