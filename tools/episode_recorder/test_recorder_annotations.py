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
                                "source_topic": "/camera_top/color/image_raw",
                                "header_stamp_ns": 123})
    assert result["active_skill"] is None
    assert result["annotation"]["header_stamp_ns"] == 123
    assert not result["annotation"].get("frame_resolution_required", False)
    with pytest.raises(ValueError):
        recorder.annotate({"kind": "skill_start", "value": "latch_box_lid_bimanual"})
    assert len(recorder.metadata["annotations"]) == 2
    recorder.annotate({"kind": "skill_start", "value": "flip_box_lid_bimanual"})
    recorder.annotate({"kind": "skill_complete", "value": "flip_box_lid_bimanual"})
    recorder.annotate({"kind": "skill_start", "value": "latch_box_lid_bimanual"})
    recorder.annotate({"kind": "skill_complete", "value": "latch_box_lid_bimanual"})
    assert [item["value"] for item in recorder.metadata["annotations"] if item["kind"] == "skill_start"] == [
        "pick_and_place_banana_in_box", "flip_box_lid_bimanual", "latch_box_lid_bimanual"]


def test_invalid_frame_does_not_change_state(recorder):
    with pytest.raises(ValueError):
        recorder.annotate({"kind": "skill_start", "value": "pick_and_place_banana_in_box",
                           "source_topic": "/unknown", "header_stamp_ns": 123})
    assert recorder.metadata["active_skill"] is None
    assert recorder.metadata["annotations"] == []


def test_unresolved_completion_marked_for_audit(recorder):
    recorder.annotate({"kind": "skill_start", "value": "pick_and_place_banana_in_box"})
    result = recorder.annotate({"kind": "skill_complete", "value": "pick_and_place_banana_in_box"})
    assert result["annotation"]["frame_resolution_required"] is True
    assert (recorder.current / "episode.json").is_file()


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
