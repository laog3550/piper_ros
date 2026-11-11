"""Test recorder annotation contract without a live ROS graph."""

import importlib.util
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
from types import ModuleType, SimpleNamespace

import pytest


def load_recorder_module():
    modules = {}
    for name in ("rclpy", "piper_msgs", "piper_msgs.srv", "std_msgs", "std_msgs.msg",
                 "std_srvs", "std_srvs.srv"):
        modules[name] = ModuleType(name)
    modules["piper_msgs.srv"].StartEpisode = object
    modules["std_msgs.msg"].String = lambda **kwargs: SimpleNamespace(**kwargs)
    modules["std_srvs.srv"].Trigger = object
    previous = {name: sys.modules.get(name) for name in modules}
    sys.modules.update(modules)
    try:
        spec = importlib.util.spec_from_file_location(
            "recorder_annotations_under_test", Path(__file__).with_name("episode_recorder.py"))
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    finally:
        for name, original in previous.items():
            if original is None:
                del sys.modules[name]
            else:
                sys.modules[name] = original


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message.data)


@pytest.fixture
def recorder():
    module = load_recorder_module()
    with TemporaryDirectory() as directory:
        instance = module.Recorder.__new__(module.Recorder)
        instance.current = Path(directory)
        instance.frame_topics = ["/camera_top/color/image_raw"]
        instance.event_publisher = Publisher()
        instance.metadata = {"schema_version": 2, "state": "recording", "mode": "task",
                             "episode_id": "test", "active_skill": None, "annotations": []}
        yield instance


def test_task_order_and_completion_frame(recorder):
    recorder.annotate({"kind": "skill_start", "value": "pick_and_place_banana_in_box"})
    result = recorder.annotate({"kind": "skill_complete", "value": "pick_and_place_banana_in_box",
                                "skill_result": "success", "source_topic": "/camera_top/color/image_raw",
                                "header_stamp_ns": 123})
    assert result["active_skill"] == "flip_box_lid_bimanual"
    assert result["annotation"]["header_stamp_ns"] == 123
    assert not result["annotation"].get("frame_resolution_required", False)
    with pytest.raises(ValueError):
        recorder.annotate({"kind": "skill_start", "value": "latch_box_lid_bimanual"})
    assert len(recorder.metadata["annotations"]) == 3
    recorder.annotate({"kind": "skill_complete", "value": "flip_box_lid_bimanual",
                       "skill_result": "success", "source_topic": "/camera_top/color/image_raw",
                       "header_stamp_ns": 124})
    assert recorder.metadata["active_skill"] == "latch_box_lid_bimanual"
    recorder.annotate({"kind": "skill_complete", "value": "latch_box_lid_bimanual",
                       "skill_result": "success", "source_topic": "/camera_top/color/image_raw",
                       "header_stamp_ns": 125})
    assert [item["value"] for item in recorder.metadata["annotations"] if item["kind"] == "skill_start"] == [
        "pick_and_place_banana_in_box", "flip_box_lid_bimanual", "latch_box_lid_bimanual"]


def test_invalid_frame_does_not_change_state(recorder):
    with pytest.raises(ValueError):
        recorder.annotate({"kind": "skill_start", "value": "pick_and_place_banana_in_box",
                           "source_topic": "/unknown", "header_stamp_ns": 123})
    assert recorder.metadata["active_skill"] is None
    assert recorder.metadata["annotations"] == []


def test_task_completion_requires_current_camera_frame(recorder):
    recorder.annotate({"kind": "skill_start", "value": "pick_and_place_banana_in_box"})
    with pytest.raises(ValueError):
        recorder.annotate({"kind": "skill_complete", "value": "pick_and_place_banana_in_box"})
    assert recorder.metadata["active_skill"] == "pick_and_place_banana_in_box"


def test_skill_success_needs_recorded_frame_and_verified_save(recorder):
    recorder.metadata.update(mode="skill", skill="pick_and_place_banana_in_box",
                             active_skill="pick_and_place_banana_in_box")
    with pytest.raises(ValueError, match="当前彩色相机帧"):
        recorder.annotate({"kind": "skill_complete", "value": "pick_and_place_banana_in_box",
                           "skill_result": "success"})
    result = recorder.annotate({"kind": "skill_complete", "value": "pick_and_place_banana_in_box",
                                "skill_result": "success", "source_topic": "/camera_top/color/image_raw",
                                "header_stamp_ns": 123})
    assert result["annotation"]["skill_result"] == "success"
    module = load_recorder_module()
    with pytest.raises(ValueError, match="已在 rosbag 核验"):
        module.require_verified_skill_completion(recorder.metadata, "success")
    result["annotation"]["verified_in_bag"] = True
    module.require_verified_skill_completion(recorder.metadata, "success")
    module.require_verified_skill_completion(recorder.metadata, "failure")


def test_recovery_trigger_has_stable_link(recorder):
    recorder.annotate({"kind": "skill_start", "value": "pick_and_place_banana_in_box"})
    result = recorder.annotate({"kind": "recovery_trigger", "value": "grasp_slip"})
    assert len(result["annotation"]["recovery_id"]) == 32
    assert result["annotation"]["skill"] == "pick_and_place_banana_in_box"


def test_clock_step_is_recorded_for_quality_audit(recorder, monkeypatch):
    recorder._last_clock_offset_ns = 1_000_000_000
    clock = recorder._sample_clock.__globals__["time"]
    monkeypatch.setattr(clock, "time_ns", lambda: 2_000_000_000)
    monkeypatch.setattr(clock, "monotonic_ns", lambda: 900_000_000)

    recorder._sample_clock()

    assert recorder.metadata["clock_adjustments"] == [{
        "observed_at_unix_ns": 2_000_000_000,
        "offset_delta_ns": 100_000_000,
    }]


def test_single_arm_does_not_require_inactive_arm_messages():
    module = load_recorder_module()
    topics = ["/piper_direct/left/master_target", "/piper_direct/right/master_target",
              "/piper_vector/left/slave_feedback", "/piper_vector/right/slave_feedback",
              "/camera_top/color/image_raw"]
    required = module.required_topics_for_episode(topics, "skill", {"arm_mode": "LEFT"})
    assert "/piper_direct/right/master_target" not in required
    assert "/piper_vector/right/slave_feedback" not in required
    assert "/piper_direct/left/master_target" in required
    assert "/camera_top/color/image_raw" in required
    assert module.required_topics_for_episode(topics, "task", {"arm_mode": "LEFT"}) == topics
    assert module.required_topics_for_episode(topics, "skill", {"arm_mode": "BIMANUAL"}) == topics


def publisher_info(name="source", gid=1):
    return SimpleNamespace(node_name=name, node_namespace="/", endpoint_gid=[gid] * 24)


def test_duplicate_same_name_publishers_rejected_before_staging(tmp_path):
    module = load_recorder_module()
    instance = module.Recorder.__new__(module.Recorder)
    instance.current = None
    instance.staging = tmp_path
    instance.topics = ["/piper_vector/left/master_target"]
    instance.node = SimpleNamespace(get_publishers_info_by_topic=lambda topic: [
        publisher_info("piper_arm_vector_left", 1), publisher_info("piper_arm_vector_left", 2)])
    instance._available = lambda: 1
    instance.min_free_bytes = 0
    with pytest.raises(ValueError, match="有 2 个发布者"):
        instance.start({"mode": "skill", "task": "香蕉入盒", "skill": module.SKILLS[0]})
    assert instance.current is None and list(tmp_path.iterdir()) == []


def test_startup_graph_rejects_duplicate_optional_source_and_keeps_endpoint_evidence(monkeypatch):
    module = load_recorder_module()
    monkeypatch.setattr(module.time, "sleep", lambda _: None)
    instance = module.Recorder.__new__(module.Recorder)
    required, optional = "/piper_vector/left/master_target", "/piper_vector/right/master_target"
    instance.topics = [required, optional]
    instance.metadata = {"required_topics": [required]}
    publishers = {required: [publisher_info()], optional: []}
    instance.node = SimpleNamespace(
        get_publishers_info_by_topic=lambda topic: publishers[topic],
        get_subscriptions_info_by_topic=lambda topic: [SimpleNamespace(node_name="bag")])
    assert instance._graph_ready("bag")
    assert instance.metadata["source_publishers_at_ready"][required][0]["gid"] == "01" * 24
    publishers[optional] = [publisher_info(gid=1), publisher_info(gid=2)]
    with pytest.raises(ValueError, match="right/master_target 有 2 个发布者"):
        instance._graph_ready("bag")


def test_duplicate_discovered_after_bag_start_stops_recorder(tmp_path, monkeypatch):
    module = load_recorder_module()
    instance = module.Recorder.__new__(module.Recorder)
    instance.current = None
    instance.staging = tmp_path
    instance.topics = ["/piper_vector/left/master_target"]
    instance.frame_topics, instance.expected_fps = [], {}
    instance.rosbag_cache_bytes, instance.raw_can_qos_depth = 0, 0
    instance.hold_max_sec, instance.hold_anchor_max_gap_sec = 10., 2.
    instance.min_free_bytes, instance.timeout = 0, 1.
    instance._available = lambda: 1
    instance.event_publisher = Publisher()
    calls = []
    def publishers(topic):
        calls.append(topic)
        return [publisher_info(gid=1)] if len(calls) == 1 else [
            publisher_info(gid=1), publisher_info(gid=2)]
    instance.node = SimpleNamespace(get_publishers_info_by_topic=publishers)
    monkeypatch.setattr(module.time, "sleep", lambda _: None)
    monkeypatch.setattr(module.subprocess, "Popen", lambda *a, **kw:
                        SimpleNamespace(pid=1234, poll=lambda: None))
    terminated = []
    def terminate():
        terminated.append(instance.process.pid)
        instance.process = None
    monkeypatch.setattr(instance, "_terminate", terminate)
    with pytest.raises(ValueError, match="有 2 个发布者"):
        instance.start({"mode": "skill", "task": "香蕉入盒", "skill": module.SKILLS[0]})
    assert terminated == [1234] and instance.process is None
    assert instance.metadata["state"] == "failed"


def test_save_selection_adds_result_with_honest_timestamp():
    module = load_recorder_module()
    metadata = {"schema_version": 2, "mode": "skill", "annotations": []}
    module.add_save_outcome_annotation(metadata, "success", 123)
    assert metadata["annotations"] == [{"kind": "outcome", "value": "success",
                                        "annotation_origin": "save_selection",
                                        "reviewed_at_unix_ns": 123}]
    module.add_save_outcome_annotation(metadata, "success", 124)
    assert len(metadata["annotations"]) == 1
    task = {"schema_version": 2, "mode": "task", "annotations": []}
    module.add_save_outcome_annotation(task, "success", 123)
    assert task["annotations"] == []


def install_vector_deserializer(monkeypatch, deserialize):
    for name in ("rclpy", "piper_msgs"):
        package = ModuleType(name)
        package.__path__ = []
        monkeypatch.setitem(sys.modules, name, package)
    serialization = ModuleType("rclpy.serialization")
    serialization.deserialize_message = deserialize
    messages = ModuleType("piper_msgs.msg")
    messages.PiperArmVector = object
    monkeypatch.setitem(sys.modules, "rclpy.serialization", serialization)
    monkeypatch.setitem(sys.modules, "piper_msgs.msg", messages)


def stale_bag(tmp_path, monkeypatch, records, camera_frames=()):
    import json
    import sqlite3
    import struct
    module = load_recorder_module()
    base = 1_800_000_000_000_000_000
    bag = tmp_path / "bag"
    bag.mkdir()
    with sqlite3.connect(bag / "bag_0.db3") as db:
        db.executescript("CREATE TABLE topics(id INTEGER, name TEXT);"
                         "CREATE TABLE messages(id INTEGER PRIMARY KEY, topic_id INTEGER, timestamp INTEGER, data BLOB);")
        db.executemany("INSERT INTO topics VALUES (?,?)", [(1, "/piper_vector/left/master_target"),
                                                           (2, "/camera_top/color/image_raw")])
        for received_ms, header_ms, state in records:
            payload = {"header_ns": base + header_ms * 1_000_000,
                       "stale": [state == "stale"] * 4, "missing": [state == "missing"] * 4,
                       "time_inconsistent": state == "inconsistent", "valid": state == "valid",
                       "gripper_mapping_verified": state != "unverified"}
            db.execute("INSERT INTO messages(topic_id,timestamp,data) VALUES (1,?,?)",
                       (base + received_ms * 1_000_000, json.dumps(payload).encode()))
        for received_ms, header_ms in camera_frames:
            sec, nsec = divmod(base + header_ms * 1_000_000, 1_000_000_000)
            prefix = b"\0\1\0\0" + struct.pack("<iI", sec, nsec)
            db.execute("INSERT INTO messages(topic_id,timestamp,data) VALUES (2,?,?)",
                       (base + received_ms * 1_000_000, prefix))
    def deserialize(raw, _):
        data = json.loads(raw)
        sec, nsec = divmod(data.pop("header_ns"), 1_000_000_000)
        return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=nsec),
                                                      frame_id="can_left"),
                               source_topic="/piper_direct/left/master_target",
                               source_can_ids=[0x155, 0x156, 0x157, 0x159], **data)
    install_vector_deserializer(monkeypatch, deserialize)
    return module, bag, base


def test_terminal_stale_uses_original_header_before_tail(tmp_path, monkeypatch):
    records = [(10, -1000, "stale"), (30, -1000, "stale"), (60, 60, "valid"),
               (180, 60, "stale"), (200, 200, "valid"),
               *[(t, 200, "valid") for t in range(220, 301, 20)],
               *[(t, 200, "stale") for t in range(320, 501, 20)]]
    # The newer image arrives late; selection must use Header, not receive order.
    module, bag, base = stale_bag(tmp_path, monkeypatch, records,
                                 [(185, 180), (390, 190), (311, 310), (490, 490)])
    before = (bag / "bag_0.db3").read_bytes()
    tail = module.terminal_stale_master_target(bag, "left", base, base + 500_000_000)
    assert tail["first_stale_receive_ns"] == base + 320_000_000
    assert tail["last_valid_header_ns"] == base + 200_000_000
    assert tail["stale_message_count"] == 10
    received, header = module.recorded_frame_before_header(
        bag, "/camera_top/color/image_raw", tail["last_valid_header_ns"], base, base + 500_000_000)
    assert header == base + 190_000_000 and received == base + 390_000_000
    assert before == (bag / "bag_0.db3").read_bytes()


@pytest.mark.parametrize("records", [
    [(100, 100, "valid"), (200, 100, "valid"), (220, 100, "stale"), (240, 240, "valid")],
    [(100, -1000, "stale"), (120, -1000, "stale")],
    [(100, 100, "unverified"), (200, 100, "unverified"), (220, 100, "stale"), (240, 100, "stale")],
    [(100, 100, "valid"), (200, 100, "valid"), (220, 100, "stale")],
    [(100, 100, "valid"), (200, 100, "valid"), (220, 100, "stale"), (240, 100, "missing")],
    [(100, 100, "valid"), (200, 100, "valid"), (220, 100, "stale"), (240, 100, "inconsistent")],
    [(100, 100, "valid"), (200, 100, "valid"), (220, 100, "stale"), (240, 110, "stale")],
])
def test_terminal_stale_does_not_infer_completion_from_other_failures(tmp_path, monkeypatch, records):
    module, bag, base = stale_bag(tmp_path, monkeypatch, records)
    assert module.terminal_stale_master_target(bag, "left", base, base + (records[-1][0] + 20) * 1_000_000) is None


def test_terminal_stale_requires_observation_continuity_to_stop(tmp_path, monkeypatch):
    records = [(100, 100, "valid"), (200, 200, "valid"), (300, 200, "valid"),
               (320, 200, "stale"), (340, 200, "stale")]
    module, bag, base = stale_bag(tmp_path, monkeypatch, records)
    assert module.terminal_stale_master_target(bag, "left", base, base + 500_000_000) is None


def test_frame_before_stale_does_not_select_a_distant_or_later_image(tmp_path, monkeypatch):
    module, bag, base = stale_bag(tmp_path, monkeypatch, [], [(50, 50), (211, 210)])
    with pytest.raises(ValueError, match="100 ms"):
        module.recorded_frame_before_header(bag, "/camera_top/color/image_raw",
                                            base + 200_000_000, base, base + 500_000_000)


@pytest.mark.parametrize("camera_frames,expected", [([(185, 180), (311, 310), (490, 490)], True),
                                                     ([(311, 310), (490, 490)], False)])
def test_stop_marks_terminal_stale_preceding_frame(recorder, monkeypatch, camera_frames, expected):
    records = [(200, 200, "valid"), *[(t, 200, "valid") for t in range(220, 301, 20)],
               *[(t, 200, "stale") for t in range(320, 501, 20)]]
    module, bag, base = stale_bag(recorder.current, monkeypatch, records, camera_frames)
    recorder.metadata.update(mode="skill", skill="pick_and_place_banana_in_box",
                             active_skill="pick_and_place_banana_in_box", ready_at_unix_ns=base,
                             task_metadata={"arm_mode": "LEFT"}, required_topics=[])
    recorder.topics, recorder.expected_fps = [], {}
    recorder.process = SimpleNamespace(poll=lambda: None)
    recorder.ready_monotonic_ns = None
    monkeypatch.setattr(recorder, "_sample_clock", lambda: None)
    monkeypatch.setattr(recorder, "_event", lambda *args, **kwargs: None)
    monkeypatch.setattr(recorder, "_terminate", lambda: None)
    monkeypatch.setattr(recorder, "status", lambda: {"state": recorder.metadata["state"]})
    globals_ = recorder.stop.__globals__
    monkeypatch.setitem(globals_, "inspect_bag", lambda *args: {})
    monkeypatch.setattr(globals_["time"], "time_ns", lambda: base + 500_000_000)
    monkeypatch.setattr(globals_["time"], "sleep", lambda _: None)
    recorder.stop(reason="duration_limit")
    assert recorder.metadata["state"] == "stopped"
    if expected:
        complete, = recorder.metadata["annotations"]
        assert complete["annotation_origin"] == "duration_limit_before_terminal_stale"
        assert complete["header_stamp_ns"] == base + 180_000_000
        assert complete["verified_in_bag"] is True
        assert recorder.metadata["auto_completion_stale_boundary"]["first_stale_receive_ns"] == base + 320_000_000
    else:
        assert recorder.metadata["annotations"] == []
        assert "100 ms" in recorder.metadata["auto_completion_error"]


def test_hold_segment_requires_explicit_operator_annotation(recorder):
    recorder.metadata.update(mode="skill", skill="pick_and_place_banana_in_box",
                             active_skill="pick_and_place_banana_in_box")
    recorder.annotate({"kind": "hold_start", "value": "operator_keep_last_target"})
    segments = recorder.metadata["hold_segments"]
    assert len(segments) == 1
    assert segments[0]["source"] == "operator_annotation"
    assert segments[0]["hold_id"]
    assert segments[0]["end_unix_ns"] is None
    with pytest.raises(ValueError, match="已有未结束"):
        recorder.annotate({"kind": "hold_start", "value": "again"})
    ended = recorder.annotate({"kind": "hold_end", "value": "operator_release"})
    assert segments[0]["end_unix_ns"] == ended["annotation"]["end_unix_ns"]
    assert segments[0]["duration_ns"] >= 0
    assert segments[0]["end_reason"] == "operator_release"
    with pytest.raises(ValueError, match="没有可结束"):
        recorder.annotate({"kind": "hold_end", "value": "again"})


def test_hold_start_requires_active_skill(recorder):
    with pytest.raises(ValueError, match="当前技能"):
        recorder.annotate({"kind": "hold_start", "value": "operator_keep_last_target"})
    assert not recorder.metadata.get("hold_segments")


def test_open_hold_is_closed_when_recording_stops(recorder):
    recorder.metadata["hold_segments"] = [
        {"hold_id": "h", "start_unix_ns": 100, "end_unix_ns": None}]
    recorder._close_open_holds(250, "recording_stopped")
    assert recorder.metadata["hold_segments"][0]["end_unix_ns"] == 250
    assert recorder.metadata["hold_segments"][0]["duration_ns"] == 150
    assert recorder.metadata["hold_segments"][0]["end_reason"] == "recording_stopped"


def test_qos_override_deepens_raw_can_reader_queue(tmp_path):
    module = load_recorder_module()
    path = module.write_qos_overrides(
        tmp_path / "qos.yaml", ["/piper_direct/left/master_target"], 5000)
    text = path.read_text(encoding="utf-8")
    assert "/piper_direct/left/master_target:" in text
    assert "history: keep_last" in text and "depth: 5000" in text
    assert "reliability: reliable" in text and "durability: volatile" in text
