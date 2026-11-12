"""合成 rosbag2 离线转换检查。运行：pytest -q tools/episode_recorder/test_to_lerobot.py。"""

import json
from pathlib import Path
import sys

import numpy as np
import pytest
from rosbags.rosbag2 import Writer

sys.path.insert(0, str(Path(__file__).parent))
import to_lerobot as converter


def fixture(root, name, *, missing_action=False, bad_dimension=False, bad_stamp=False,
            arm_mode="LEFT", camera_only=False, wrist_lag_ns=0, state_lag_ns=0, frame_count=2):
    episode = root / name
    bag = episode / "bag"
    episode.mkdir()
    (episode / "episode.json").write_text(json.dumps({
        "schema_version": 1, "state": "saved", "episode_id": name, "task": "抓取方块",
        "started_at_unix_ns": 1_799_999_999_900_000_000,
        "stopped_at_unix_ns": 1_800_000_000_000_000_000 + frame_count * 100_000_000,
        "task_metadata": {"arm_mode": arm_mode, "time_domain": "unix_ns",
                          "action_space": "absolute_joint_position_7",
                          "action_provenance": "human_demonstration",
                          "calibration_version": "synthetic-calibration",
                          "data_convention_version": "synthetic-data-convention",
                          "normalization_version": "synthetic-normalization"}}), encoding="utf-8")
    s = converter.TYPESTORE
    Header = s.types["std_msgs/msg/Header"]
    Time = s.types["builtin_interfaces/msg/Time"]
    Joint = s.types["sensor_msgs/msg/JointState"]
    Image = s.types["sensor_msgs/msg/Image"]
    arms = ["left", "right"] if arm_mode == "BIMANUAL" else [arm_mode.lower()]
    topics = [converter.CAMERAS["top_rgb"], *(converter.CAMERAS[f"{arm}_wrist_rgb"] for arm in arms)]
    if not camera_only:
        topics.extend(converter.STATES[arm] for arm in arms)
    if not missing_action and not camera_only:
        topics.extend(converter.ACTIONS[arm] for arm in arms)
    base = 1_800_000_000_000_000_000
    with Writer(bag, version=Writer.VERSION_LATEST) as writer:
        connections = {topic: writer.add_connection(topic, converter.TYPES[topic], typestore=s) for topic in topics}
        for index in range(frame_count):
            timestamp = base + index * 100_000_000
            for topic in topics:
                actual = timestamp + (10_000_000 if bad_stamp and index == 1 and topic == converter.ACTIONS[arms[0]] else 0)
                if topic in (converter.CAMERAS["left_wrist_rgb"], converter.CAMERAS["right_wrist_rgb"]):
                    actual -= wrist_lag_ns
                if topic in converter.STATES.values():
                    actual -= state_lag_ns
                header = Header(Time(actual // 1_000_000_000, actual % 1_000_000_000), "synthetic_frame")
                if topic in converter.CAMERAS.values():
                    message = Image(header, 480, 640, "rgb8", 0, 640 * 3,
                                    np.full(480 * 640 * 3, index, dtype=np.uint8))
                else:
                    names = converter.JOINTS[:-1] if bad_dimension and topic == converter.ACTIONS[arms[0]] else converter.JOINTS
                    position = np.arange(len(names), dtype=np.float64) / 10
                    message = Joint(header, names, position, np.array([], dtype=np.float64),
                                    np.array([], dtype=np.float64))
                writer.write(connections[topic], timestamp + 1000,
                             s.serialize_cdr(message, converter.TYPES[topic]))
    return episode


def test_synthetic_rosbag_maps_rgb_state_action(tmp_path):
    episode = fixture(tmp_path, "episode_000001")
    meta, frames = converter.read_episode(episode, ["left"], 10, 20_000_000, 2_000_000, materialize=True)
    assert meta["episode_id"] == "episode_000001"
    assert len(frames) == 2
    assert frames[1]["observation.images.top_rgb"].shape == (480, 640, 3)
    assert frames[1]["observation.images.left_wrist_rgb"][0, 0, 0] == 1
    assert frames[0]["observation.state"].shape == (7,)
    assert frames[0]["action"].shape == (7,)
    assert frames[0]["task"] == "抓取方块"


def test_bimanual_order_and_cameras(tmp_path):
    episode = fixture(tmp_path, "episode_000001", arm_mode="BIMANUAL")
    _, frames = converter.read_episode(episode, ["left", "right"], 10, 20_000_000, 2_000_000)
    assert frames[0]["observation.state"].shape == (14,)
    assert frames[0]["action"].shape == (14,)
    assert converter.CAMERAS["right_wrist_rgb"] in frames[0]["__source_header_ns__"]


@pytest.mark.parametrize("kwargs,match", [
    ({"missing_action": True}, "缺少话题"),
    ({"bad_dimension": True}, "7 个 position"),
    ({"bad_stamp": True}, "不符合 10 FPS"),
])
def test_reject_invalid_demonstration(tmp_path, kwargs, match):
    episode = fixture(tmp_path, "episode_000001", **kwargs)
    with pytest.raises(ValueError, match=match):
        converter.read_episode(episode, ["left"], 10, 20_000_000, 2_000_000)


def test_reject_action_outside_episode_window(tmp_path):
    episode = fixture(tmp_path, "episode_000001")
    metadata_path = episode / "episode.json"
    meta = json.loads(metadata_path.read_text(encoding="utf-8"))
    meta["stopped_at_unix_ns"] = 1_800_000_000_050_000_000
    metadata_path.write_text(json.dumps(meta), encoding="utf-8")
    with pytest.raises(ValueError, match="不在 Episode 录制区间"):
        converter.read_episode(episode, ["left"], 10, 20_000_000, 2_000_000)


def test_episode_split_does_not_leak_frames(tmp_path, monkeypatch):
    from types import ModuleType, SimpleNamespace
    source = tmp_path / "source"
    source.mkdir()
    fixture(source, "episode_000001")
    fixture(source, "episode_000002")
    saved = {}

    class FakeDataset:
        def __init__(self, repo_id=None, root=None):
            self.root = root
            self.frames = []
            self.meta = SimpleNamespace(total_episodes=len(saved.get(root.name, [])))

        @classmethod
        def create(cls, **kwargs):
            return cls(root=kwargs["root"])

        def add_frame(self, frame):
            self.frames.append(frame)

        def save_episode(self):
            saved.setdefault(self.root.name, []).append(self.frames)
            self.frames = []

        def finalize(self):
            pass

        def __len__(self):
            return sum(map(len, saved.get(self.root.name, [])))

        def __getitem__(self, index):
            frame = saved[self.root.name][0][index]
            return {"action": frame["action"],
                    "observation.images.top_rgb": frame["observation.images.top_rgb"].transpose(2, 0, 1)}

    for name in ("lerobot", "lerobot.datasets", "lerobot.datasets.lerobot_dataset"):
        monkeypatch.setitem(sys.modules, name, ModuleType(name))
    sys.modules["lerobot.datasets.lerobot_dataset"].LeRobotDataset = FakeDataset
    import importlib.metadata
    original_version = importlib.metadata.version
    monkeypatch.setattr(importlib.metadata, "version",
                        lambda name: {"lerobot": "0.4.4", "rosbags": "0.11.7"}.get(name)
                        or original_version(name))
    output = tmp_path / "output"
    args = SimpleNamespace(input_dir=source, output_dir=output, repo_id="local/piper",
                           arm_mode="LEFT", input_format="joint-state", fps=10, max_age_ms=20,
                           jitter_ms=2, max_can_skew_ms=30, test_percent=50)
    converter.convert(args)
    manifest = json.loads((output / "conversion_manifest.json").read_text(encoding="utf-8"))
    stamps = [json.loads(line) for line in (output / "source_timestamps.jsonl").read_text(encoding="utf-8").splitlines()]
    assert {item["split"] for item in manifest["episodes"]} == {"train", "test"}
    assert len(stamps) == 4
    assert {item["episode_id"] for item in stamps if item["split"] == "test"} != {item["episode_id"] for item in stamps if item["split"] == "train"}
    assert all(converter.ACTIONS["left"] in item["header_timestamp_ns"] for item in stamps)
    assert len(saved["train"]) == len(saved["test"]) == 1
    assert len(saved["train"][0]) == len(saved["test"][0]) == 2
    assert len({item["episode_id"] for item in manifest["episodes"]}) == 2


def vector_fixture(root, name, *, verified=True, valid=True, wrong_source=False,
                   bad_can_skew=False, missing_action=False, outside_episode=False,
                   duplicate_vector=False, gripper_raw=None, frame_count=6,
                   period_ns=20_000_000, transform=None):
    """写入录制配置的 20 路话题，模拟 50 Hz 向量与相机流。"""
    from rosbags.typesys import get_types_from_msg

    episode = root / name
    bag = episode / "bag"
    episode.mkdir()
    base = 1_800_000_100_000_000_000
    (episode / "episode.json").write_text(json.dumps({
        "schema_version": 1, "state": "saved", "episode_id": name, "task": "抓取方块",
        "started_at_unix_ns": base - 20_000_000,
        "stopped_at_unix_ns": base + frame_count * period_ns + 20_000_000,
        "task_metadata": {"arm_mode": "BIMANUAL", "time_domain": "unix_ns",
                          "action_space": "absolute_joint_position_7",
                          "action_provenance": "human_demonstration",
                          "calibration_version": "synthetic-calibration",
                          "data_convention_version": "synthetic-data-convention",
                          "normalization_version": "synthetic-normalization"}}), encoding="utf-8")
    s = converter.TYPESTORE
    direct_type = "piper_msgs/msg/PiperDirectCanFrame"
    s.register(get_types_from_msg(
        (Path(__file__).parents[2] / "src/piper_msgs/msg/PiperDirectCanFrame.msg").read_text(),
        direct_type))
    Header = s.types["std_msgs/msg/Header"]
    Time = s.types["builtin_interfaces/msg/Time"]
    Vector = s.types[converter.VECTOR_TYPE]
    Direct = s.types[direct_type]
    Image = s.types["sensor_msgs/msg/Image"]
    CameraInfo = s.types["sensor_msgs/msg/CameraInfo"]
    ROI = s.types["sensor_msgs/msg/RegionOfInterest"]
    direct_topics = [f"/piper_direct/{arm}/{kind}" for arm in ("left", "right")
                     for kind in ("master_target", "slave_feedback")]
    vector_topics = [f"/piper_vector/{arm}/{kind}" for arm in ("left", "right")
                     for kind in ("master_target", "slave_feedback")]
    camera_bases = ["/camera_top/D455", "/camera_wrist_left", "/camera_wrist_right"]
    extra = []
    for camera in camera_bases:
        extra += [f"{camera}/color/image_raw", f"{camera}/color/camera_info"]
        depth = "/aligned_depth_to_color" if camera.endswith("D455") else "/depth"
        extra += [f"{camera}{depth}/image_raw", f"{camera}{depth}/camera_info"]
    types = {**{topic: direct_type for topic in direct_topics},
             **{topic: converter.VECTOR_TYPE for topic in vector_topics},
             **{topic: "sensor_msgs/msg/Image" if topic.endswith("image_raw") else "sensor_msgs/msg/CameraInfo"
                for topic in extra}}
    assert len(types) == 20
    with Writer(bag, version=Writer.VERSION_LATEST) as writer:
        connections = {topic: writer.add_connection(topic, msgtype, typestore=s)
                       for topic, msgtype in types.items()}
        for index in range(frame_count):
            t = base + index * period_ns
            stamp = Time(t // 1_000_000_000, t % 1_000_000_000)
            for topic, msgtype in types.items():
                if missing_action and topic == converter.CAN_ACTIONS["left"]:
                    continue
                arm = "left" if "/left/" in topic else "right"
                header = Header(stamp, f"can_{arm}" if topic in vector_topics + direct_topics else "synthetic_camera")
                if topic in vector_topics:
                    kind = topic.rsplit("/", 1)[-1]
                    offsets = ([40, 2, 1, 0] if bad_can_skew else
                               [24, 2, 1, 0] if outside_episode and index == 0 else
                               [3, 2, 1, 0])
                    sources = [Time((t - offset * 1_000_000) // 1_000_000_000,
                                    (t - offset * 1_000_000) % 1_000_000_000) for offset in offsets]
                    source_topic = f"/piper_direct/{arm}/{kind}"
                    if wrong_source and topic == converter.CAN_ACTIONS["left"]:
                        source_topic = f"/piper_direct/{arm}/slave_feedback"
                    positions = np.arange(7, dtype=np.float64) / 10
                    positions += (1 if kind == "master_target" else 0) + (2 if arm == "right" else 0)
                    if gripper_raw is not None:
                        positions[6] = gripper_raw[topic][index]
                    message = Vector(header, source_topic, converter.JOINTS,
                                     positions,
                                     np.array(converter.CAN_IDS[kind], dtype=np.uint32), sources,
                                     np.zeros(4, dtype=np.bool_), np.zeros(4, dtype=np.bool_),
                                     False, valid, verified)
                elif topic in direct_topics:
                    message = Direct(header, 0x155, np.zeros(8, dtype=np.uint8), 1,
                                     np.zeros(2, dtype=np.float64), 0.0, 0.0, 0, 0)
                elif topic.endswith("image_raw"):
                    if "/color/" in topic:
                        message = Image(header, 480, 640, "rgb8", 0, 640 * 3,
                                        np.full(480 * 640 * 3, index, dtype=np.uint8))
                    else:
                        message = Image(header, 1, 1, "16UC1", 0, 2, np.zeros(2, dtype=np.uint8))
                else:
                    message = CameraInfo(header, 1, 1, "plumb_bob", np.zeros(5), np.zeros(9),
                                         np.zeros(9), np.zeros(12), 0, 0, ROI(0, 0, 0, 0, False))
                if transform is not None:
                    message = transform(topic, index, message)
                    if message is None:
                        continue
                writer.write(connections[topic], t + 1000, s.serialize_cdr(message, msgtype))
                if duplicate_vector and topic in vector_topics:
                    writer.write(connections[topic], t + 2000, s.serialize_cdr(message, msgtype))
    return episode


def test_twenty_topic_can_vector_episode(tmp_path):
    episode = vector_fixture(tmp_path, "episode_000001", duplicate_vector=True)
    from rosbags.rosbag2 import Reader
    with Reader(episode / "bag") as reader:
        assert len(reader.connections) == 20
    meta, frames = converter.read_episode(episode, ["left", "right"], 10, 30_000_000, 2_000_000,
                                          input_format="can-vector")
    assert meta["episode_id"] == "episode_000001"
    assert len(frames) == 2
    assert frames[0]["action"].shape == (14,)
    assert frames[0]["observation.state"].shape == (14,)
    assert frames[0]["action"][[0, 7]].tolist() == [1.0, 3.0]
    assert frames[0]["observation.state"][[0, 7]].tolist() == [0.0, 2.0]
    assert set(frames[0]["__source_can_stamps_ns__"]) == set(converter.CAN_ACTIONS.values()) | set(converter.CAN_STATES.values())
    assert len(frames[0]["__source_header_ns__"]) == 7


def test_follower_next_state_uses_feedback_when_master_target_has_gap(tmp_path):
    def drop_master(topic, index, message):
        if topic == converter.CAN_ACTIONS["left"] and 40 <= index <= 60:
            return None
        return message

    episode = vector_fixture(tmp_path, "episode_000001", frame_count=100, transform=drop_master)
    master_meta, master_frames = converter.read_episode(
        episode, ["left", "right"], 10, 30_000_000, 2_000_000,
        input_format="can-vector", extract_valid_segments=True,
        max_source_skew_ns=30_000_000, require_training_contract=False)
    assert not any(item["meets_16_step_window"] for item in master_meta["_quality"]["segments"])

    meta, frames = converter.read_episode(
        episode, ["left", "right"], 10, 30_000_000, 2_000_000,
        input_format="can-vector", extract_valid_segments=True,
        max_source_skew_ns=30_000_000,
        require_training_contract=False, action_source="follower_next_state")
    assert meta["_quality"]["action_source"] == "follower_next_state"
    assert meta["_quality"]["alignment_policy"] == converter.FOLLOWER_ALIGNMENT_POLICY
    assert len(frames) == 20
    assert len(meta["_quality"]["segments"]) == 1
    assert meta["_quality"]["segments"][0]["frames"] >= 16
    topic = converter.CAN_STATES["left"]
    for frame in frames:
        assert frame["__action_source_header_ns__"][topic] > frame["__source_header_ns__"][topic]
    assert frames[0]["action"].tolist() == frames[1]["observation.state"].tolist()


def test_follower_next_state_contract_requires_matching_provenance(tmp_path):
    episode = vector_fixture(tmp_path, "episode_000001")
    meta = json.loads((episode / "episode.json").read_text(encoding="utf-8"))
    fields = meta["task_metadata"]
    fields.update(action_source="follower_next_state",
                  action_provenance="teleoperation_follower_next_state")
    assert converter.training_contract_reasons(meta, "BIMANUAL", (), "follower_next_state") == []
    assert converter.training_contract_reasons(meta, "BIMANUAL", (), "master_target")

    legacy = json.loads(json.dumps(meta))
    legacy["task_metadata"].pop("action_source")
    legacy["task_metadata"]["action_provenance"] = "human_demonstration"
    assert converter.training_contract_reasons(legacy, "BIMANUAL", (), "follower_next_state")
    accepted = {"action_provenance": {"status": "verified"}}
    assert converter.training_contract_reasons(legacy, "BIMANUAL", accepted,
                                               "follower_next_state") == []


def test_can_vector_gripper_calibration_preserves_arm_mapping(tmp_path):
    calibration = converter.load_gripper_calibration(
        Path(__file__).parents[2] / "config/gripper_collection_calibration.20261105.json")
    raw = {topic: [spec["closed_reference"], spec["observed_open"],
                   spec["observed_open"], spec["closed_reference"],
                   spec["closed_reference"], spec["closed_reference"]]
           for topic, spec in calibration["streams"].items()}
    episode = vector_fixture(tmp_path, "episode_000001", verified=False, valid=False,
                             gripper_raw=raw)
    _, frames = converter.read_episode(episode, ["left", "right"], 10, 30_000_000,
                                       2_000_000, input_format="can-vector",
                                       gripper_calibration=calibration)
    assert frames[0]["action"][[6, 13]].tolist() == [0.0, 0.0]
    assert frames[0]["observation.state"][[6, 13]].tolist() == [0.0, 0.0]
    assert all(0 <= frame["action"][i] <= 0.1 for frame in frames for i in (6, 13))
    assert frames[0]["action"][[0, 7]].tolist() == [1.0, 3.0]
    assert all(not item["raw_valid"] and not item["raw_gripper_mapping_verified"]
               and item["gripper_calibration_applied"]
               for item in frames[0]["__source_vector_quality__"].values())


@pytest.mark.parametrize("kwargs,match", [
    ({"verified": False}, "夹爪映射未核实"),
    ({"valid": False}, "valid=false"),
    ({"wrong_source": True}, "CAN 来源话题"),
    ({"bad_can_skew": True}, "四帧来源时间"),
    ({"outside_episode": True}, "CAN 来源时间不在 Episode"),
    ({"missing_action": True}, "没有消息"),
])
def test_can_vector_rejects_untrusted_sources(tmp_path, kwargs, match):
    episode = vector_fixture(tmp_path, "episode_000001", **kwargs)
    with pytest.raises(ValueError, match=match):
        converter.read_episode(episode, ["left", "right"], 10, 30_000_000, 2_000_000,
                               input_format="can-vector")


def test_only_camera_episode_is_rejected(tmp_path):
    episode = fixture(tmp_path, "episode_000001", camera_only=True)
    with pytest.raises(ValueError, match="缺少话题"):
        converter.read_episode(episode, ["left"], 10, 20_000_000, 2_000_000)


@pytest.mark.parametrize("input_format,arm_mode,dimension", [
    ("joint-state", "LEFT", 7),
    ("can-vector", "BIMANUAL", 14),
])
def test_real_lerobot_roundtrip(tmp_path, input_format, arm_mode, dimension):
    """用真实 LeRobotDataset 写入、finalize 和重新打开，不使用 FakeDataset。"""
    from importlib.metadata import version
    if version("lerobot") != "0.4.4":
        pytest.skip("需要 lerobot==0.4.4")
    pytest.importorskip("lerobot.datasets.lerobot_dataset")
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    from types import SimpleNamespace

    source = tmp_path / "episodes"
    source.mkdir()
    make = vector_fixture if input_format == "can-vector" else fixture
    for name in ("episode_000001", "episode_000002"):
        make(source, name)
    output = tmp_path / "dataset"
    args = SimpleNamespace(input_dir=source, output_dir=output, repo_id="local/piper_acceptance",
                           arm_mode=arm_mode, input_format=input_format, fps=10,
                           max_age_ms=30, jitter_ms=2, max_can_skew_ms=30, test_percent=50)
    converter.convert(args)
    manifest = json.loads((output / "conversion_manifest.json").read_text(encoding="utf-8"))
    assert {entry["split"] for entry in manifest["episodes"]} == {"train", "test"}
    assert len({entry["episode_id"] for entry in manifest["episodes"]}) == 2
    assert all(entry["frames"] == 2 for entry in manifest["episodes"])
    stamps = [json.loads(line) for line in (output / "source_timestamps.jsonl").read_text(encoding="utf-8").splitlines()]
    assert len(stamps) == 4
    for split in ("train", "test"):
        dataset = LeRobotDataset(f"local/piper_acceptance_{split}", root=output / split)
        assert dataset.meta.total_episodes == 1
        assert len(dataset) == 2
        first, second = dataset[0], dataset[1]
        assert first["task"] == second["task"] == "抓取方块"
        assert tuple(first["action"].shape) == (dimension,)
        assert tuple(first["observation.state"].shape) == (dimension,)
        assert tuple(first["observation.images.top_rgb"].shape) == (3, 480, 640)
        assert first["observation.images.top_rgb"][0, 0, 0].item() == pytest.approx(0)
        expected_pixel = 5 if input_format == "can-vector" else 1
        assert second["observation.images.top_rgb"][0, 0, 0].item() == pytest.approx(expected_pixel / 255)
        if input_format == "can-vector":
            assert first["action"][[0, 7]].tolist() == [1.0, 3.0]
            assert first["observation.state"][[0, 7]].tolist() == [0.0, 2.0]
            assert tuple(first["observation.images.right_wrist_rgb"].shape) == (3, 480, 640)
        else:
            assert first["action"][0].item() == pytest.approx(0)
            assert tuple(first["observation.images.left_wrist_rgb"].shape) == (3, 480, 640)


def test_real_lerobot_rejects_camera_only(tmp_path):
    from importlib.metadata import version
    if version("lerobot") != "0.4.4":
        pytest.skip("需要 lerobot==0.4.4")
    pytest.importorskip("lerobot.datasets.lerobot_dataset")
    from types import SimpleNamespace

    source = tmp_path / "episodes"
    source.mkdir()
    for name in ("episode_000001", "episode_000002"):
        fixture(source, name, camera_only=True)
    output = tmp_path / "dataset"
    args = SimpleNamespace(input_dir=source, output_dir=output, repo_id="local/piper_camera_only",
                           arm_mode="LEFT", input_format="joint-state", fps=10,
                           max_age_ms=30, jitter_ms=2, max_can_skew_ms=30, test_percent=50)
    with pytest.raises(ValueError, match="缺少话题"):
        converter.convert(args)
    assert not output.exists()


def paused_master(topic, index, message):
    """主端停发后只重复旧 Header；相机和从反馈继续前进。"""
    if topic == converter.CAN_ACTIONS["left"] and 20 <= index < 30:
        s = converter.TYPESTORE
        Time = s.types["builtin_interfaces/msg/Time"]
        Header = s.types["std_msgs/msg/Header"]
        shift = (index - 19) * 100_000_000
        def earlier(value):
            timestamp = value.sec * 1_000_000_000 + value.nanosec - shift
            return Time(timestamp // 1_000_000_000, timestamp % 1_000_000_000)
        message.header = Header(earlier(message.header.stamp), message.header.frame_id)
        message.source_stamps = [earlier(value) for value in message.source_stamps]
        message.stale[:] = True
        message.valid = False
    return message


def paused_episode(root, name):
    return vector_fixture(root, name, frame_count=50, period_ns=100_000_000,
                          transform=paused_master)


def read_segments(episode, **kwargs):
    return converter.read_episode(episode, ["left", "right"], 10, 100_000_000, 2_000_000,
                                  input_format="can-vector", max_source_skew_ns=30_000_000,
                                  extract_valid_segments=True, **kwargs)


def mark_segment_completion(episode, index=49, group_id=None):
    path = episode / "episode.json"
    meta = json.loads(path.read_text(encoding="utf-8"))
    timestamp = 1_800_000_100_000_000_000 + index * 100_000_000
    meta.update(schema_version=2, mode="skill", skill="flip_box_lid_bimanual",
                outcome="success", frame_topics=[converter.CAMERAS["top_rgb"]],
                annotations=[{"kind": "skill_complete", "value": "flip_box_lid_bimanual",
                              "source_topic": converter.CAMERAS["top_rgb"],
                              "header_stamp_ns": timestamp, "recorded_at_unix_ns": timestamp},
                             {"kind": "outcome", "value": "success", "recorded_at_unix_ns": timestamp}])
    meta["task_metadata"]["initial_conditions"] = {"perturbation_id": group_id or episode.name}
    path.write_text(json.dumps(meta), encoding="utf-8")
    label, reasons = converter.sft_label(meta)
    assert not reasons
    return label


def test_stale_duplicate_headers_split_without_rewriting_sources(tmp_path):
    episode = paused_episode(tmp_path, "episode_000001")
    before = converter.source_digest(episode)
    with pytest.raises(ValueError, match="stale"):
        converter.read_episode(episode, ["left", "right"], 10, 100_000_000, 2_000_000,
                               input_format="can-vector")
    meta, frames = read_segments(episode)
    segments = meta["_quality"]["segments"]
    assert [item["frames"] for item in segments] == [20, 20]
    assert all(item["meets_16_step_window"] for item in segments)
    assert [item["window_count"] for item in segments] == [5, 5]
    assert sum(item["count"] for item in meta["_quality"]["rejected_vector_messages"]) == 10
    assert all("stale" in item["reason"] for item in meta["_quality"]["rejected_vector_messages"])
    base = 1_800_000_100_000_000_000
    assert frames[19]["__target_timestamp_ns__"] == base + 1_900_000_000
    assert frames[19]["__source_header_ns__"][converter.CAN_ACTIONS["left"]] == base + 1_900_000_000
    assert frames[19]["__source_can_stamps_ns__"][converter.CAN_ACTIONS["left"]] == [
        base + 1_900_000_000 - offset * 1_000_000 for offset in (3, 2, 1, 0)]
    assert all(flags["raw_valid"] for frame in frames for flags in frame["__source_vector_quality__"].values())
    assert before == converter.source_digest(episode)


def test_operator_hold_reuses_last_target_without_rewriting_stale(tmp_path):
    episode = paused_episode(tmp_path, "episode_000001")
    path = episode / "episode.json"
    meta = json.loads(path.read_text(encoding="utf-8"))
    base = 1_800_000_100_000_000_000
    meta["hold_policy"] = {"source": "operator_annotation", "max_hold_sec": 10,
                           "anchor_max_gap_sec": 2}
    meta["hold_segments"] = [{"hold_id": "hold-1", "start_unix_ns": base + 2_000_000_000,
                              "end_unix_ns": base + 3_000_000_000,
                              "source": "operator_annotation", "skill": "flip_box_lid_bimanual",
                              "intent": "operator_keep_last_target",
                              "end_reason": "operator_release"}]
    path.write_text(json.dumps(meta), encoding="utf-8")
    quality, frames = read_segments(episode)
    hold_frames = [frame for frame in frames if frame.get("__hold__")]
    assert len(hold_frames) == 10
    assert all(frame["__hold__"]["hold_id"] == "hold-1" for frame in hold_frames)
    assert all(frame["__hold__"]["source"] == "operator_annotation" for frame in hold_frames)
    assert all(frame["__source_header_ns__"][converter.CAN_ACTIONS["left"]]
               == base + 1_900_000_000 for frame in hold_frames)
    assert all(frame["__hold__"]["stale_header_ns"] == base + 1_900_000_000
               for frame in hold_frames)
    assert all(any(frame["__hold__"]["stale_flags"]) for frame in hold_frames)
    assert quality["_quality"]["hold_segments"][0]["aligned_frames"] == 10
    assert max(item["frames"] for item in quality["_quality"]["segments"]) >= 16


def test_hold_without_recent_anchor_is_not_accepted(tmp_path):
    episode = paused_episode(tmp_path, "episode_000001")
    path = episode / "episode.json"
    meta = json.loads(path.read_text(encoding="utf-8"))
    base = 1_800_000_100_000_000_000
    meta["hold_policy"] = {"source": "operator_annotation", "max_hold_sec": 10,
                           "anchor_max_gap_sec": 0.01}
    meta["hold_segments"] = [{"hold_id": "hold-1", "start_unix_ns": base + 2_000_000_000,
                              "end_unix_ns": base + 3_000_000_000,
                              "source": "operator_annotation"}]
    path.write_text(json.dumps(meta), encoding="utf-8")
    quality, frames = read_segments(episode)
    assert not any(frame.get("__hold__") for frame in frames)
    assert quality["_quality"]["hold_segments"][0]["aligned_frames"] == 0
    rejected = {item["grid_index"] for item in quality["_quality"]["rejected_frames"]}
    assert set(range(20, 30)) <= rejected


def test_invalid_vector_between_grid_points_still_breaks_segment(tmp_path):
    def invalidate(topic, index, message):
        if topic == converter.CAN_ACTIONS["left"] and index == 39:
            message.valid = False
            message.stale[:] = True
        return message
    episode = vector_fixture(tmp_path, "episode_000001", frame_count=85,
                             period_ns=50_000_000, transform=invalidate)
    meta, frames = read_segments(episode)
    assert [item["frames"] for item in meta["_quality"]["segments"]] == [20, 23]
    assert not meta["_quality"]["rejected_frames"]
    assert frames[19]["__segment_index__"] != frames[20]["__segment_index__"]


def test_camera_outage_is_a_segment_boundary(tmp_path):
    def camera_outage(topic, index, message):
        if topic == converter.CAMERAS["top_rgb"] and 20 <= index < 30:
            return None
        return message
    episode = vector_fixture(tmp_path, "episode_000001", frame_count=50,
                             period_ns=100_000_000, transform=camera_outage)
    meta, frames = read_segments(episode)
    assert [item["frames"] for item in meta["_quality"]["segments"]] == [20, 20]
    assert meta["_quality"]["camera_max_gap_ns"][converter.CAMERAS["top_rgb"]] == 1_100_000_000
    assert not meta["_quality"]["rejected_vector_messages"]
    assert len(frames) == 40
    assert "图像组重复" in meta["_quality"]["rejected_frames"][0]["reason"]


@pytest.mark.parametrize("wrong_source", [False, True])
def test_segmentation_does_not_hide_conflicting_or_wrong_sources(tmp_path, wrong_source):
    def conflicting(topic, index, message):
        message = paused_master(topic, index, message)
        if topic == converter.CAN_ACTIONS["left"] and index == 20:
            message.position[0] += 0.01
        return message
    episode = vector_fixture(tmp_path, "episode_000001", frame_count=31,
                             period_ns=100_000_000, wrong_source=wrong_source,
                             transform=None if wrong_source else conflicting)
    with pytest.raises(ValueError, match="CAN 来源话题" if wrong_source else "冲突向量"):
        read_segments(episode)


def test_completion_belongs_only_to_its_continuous_segment(tmp_path):
    episode = paused_episode(tmp_path, "episode_000001")
    label = mark_segment_completion(episode)
    meta, frames = read_segments(episode)
    selected, stats = converter.select_sft_frames(meta, frames, label, ["left", "right"], "can-vector")
    assert len(selected) == 20
    assert {item["__segment_index__"] for item in selected} == {1}
    assert stats["window_count"] == 5
    label = mark_segment_completion(episode, 25)
    with pytest.raises(ValueError, match="不属于任何连续有效片段"):
        converter.select_sft_frames(meta, frames, label, ["left", "right"], "can-vector")
    label = mark_segment_completion(episode, 44)
    with pytest.raises(ValueError, match="不足 16 帧"):
        converter.select_sft_frames(meta, frames, label, ["left", "right"], "can-vector")


def test_calibration_cannot_explain_unrelated_valid_false(tmp_path):
    calibration = converter.load_gripper_calibration(
        Path(__file__).parents[2] / "config/gripper_collection_calibration.20261105.json")
    episode = vector_fixture(tmp_path, "episode_000001", valid=False, verified=True)
    with pytest.raises(ValueError, match="valid=false"):
        converter.read_episode(episode, ["left", "right"], 10, 30_000_000, 2_000_000,
                               input_format="can-vector", gripper_calibration=calibration)
    meta, frames = read_segments(episode, gripper_calibration=calibration)
    assert frames == [] and meta["_quality"]["segments"] == []
    assert meta["_quality"]["rejected_vector_messages"]


def test_segment_audit_keeps_provisional_candidates_ineligible(tmp_path):
    import audit_real_data as audit
    episode = paused_episode(tmp_path, "episode_000001")
    mark_segment_completion(episode)
    strict = audit.audit_native(episode, max_source_skew_ms=30)
    assert not strict["eligible"] and "stale" in " ".join(strict["reasons"])
    row = audit.audit_native(episode, max_source_skew_ms=30, extract_valid_segments=True)
    assert row["eligible"] and row["eligible_frame_count"] == 20
    assert row["candidate_segment_count"] == 2 and row["candidate_frame_count"] == 40
    path = episode / "episode.json"
    meta = json.loads(path.read_text(encoding="utf-8"))
    meta["task_metadata"].pop("normalization_version")
    meta["task_metadata"]["collection_contract_status"] = "training_not_ready_provisional_collection_versions"
    path.write_text(json.dumps(meta), encoding="utf-8")
    row = audit.audit_native(episode, max_source_skew_ms=30, extract_valid_segments=True)
    assert not row["eligible"] and row["eligible_frame_count"] == 0
    assert row["candidate_segment_count"] == 2 and row["candidate_frame_count"] == 40
    assert row["normalization_version"] is None
    assert row["training_normalization"]["status"] == "not_computed"
    assert not row["training_ready"]


def test_data_convention_is_not_training_normalization():
    meta = {"task_metadata": {"arm_mode": "LEFT", "time_domain": "unix_ns",
            "action_space": "absolute_joint_position_7", "action_provenance": "human_demonstration",
            "calibration_version": "synthetic-camera", "data_convention_version": "raw-fields-v1",
            "normalization_version": "raw-fields-v1"}}
    assert "原始字段约定不能用作训练 normalization_version" in converter.training_contract_reasons(meta, "LEFT")


@pytest.mark.parametrize("success_sft,counts", [(False, [20, 20]), (True, [20])])
def test_segmented_real_lerobot_roundtrip(tmp_path, success_sft, counts):
    """真实 v3 中各片段独立存储；Header 与场景组划分可以回溯。"""
    from types import SimpleNamespace
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    import pyarrow.parquet as pq
    source = tmp_path / "episodes"
    source.mkdir()
    for index in (1, 2):
        episode = paused_episode(source, f"episode_{index:06d}")
        if success_sft:
            mark_segment_completion(episode)
            path = episode / "episode.json"
            meta = json.loads(path.read_text(encoding="utf-8"))
            meta["task_metadata"].pop("normalization_version")
            path.write_text(json.dumps(meta), encoding="utf-8")
    output = tmp_path / "dataset"
    args = SimpleNamespace(input_dir=source, output_dir=output, repo_id="local/piper_segments",
                           arm_mode="BIMANUAL", input_format="can-vector", fps=10,
                           max_age_ms=100, jitter_ms=2, max_can_skew_ms=30,
                           max_source_skew_ms=30, test_percent=50, success_sft=success_sft,
                           extract_valid_segments=True)
    converter.convert(args)
    manifest = json.loads((output / "conversion_manifest.json").read_text(encoding="utf-8"))
    assert not manifest["training_ready"]
    if success_sft:
        assert all(item["normalization_version"] is None
                   and item["training_normalization"]["status"] == "not_computed"
                   for item in manifest["episodes"])
    references = [json.loads(line) for line in (output / "source_timestamps.jsonl").read_text().splitlines()]
    for split in ("train", "test"):
        dataset = LeRobotDataset(f"local/piper_segments_{split}", root=output / split)
        entries = [entry for entry in manifest["episodes"] if entry["split"] == split]
        assert dataset.meta.total_episodes == len(counts)
        assert len(dataset) == sum(counts)
        assert [entry["frames"] for entry in entries] == counts
        assert len({entry["source_episode_id"] for entry in entries}) == 1
        rows = [row for path in (output / split / "data").rglob("*.parquet")
                for row in pq.read_table(path).to_pylist()]
        for entry in entries:
            selected = [item for item in references if item["split"] == split and item["episode_id"] == entry["episode_id"]]
            assert len(selected) == entry["frames"] >= converter.ACTION_HORIZON
            assert len({item["segment_index"] for item in selected}) == 1
            assert all(right["target_timestamp_ns"] - left["target_timestamp_ns"] == 100_000_000
                       for left, right in zip(selected, selected[1:]))
            assert all(item["header_timestamp_ns"][converter.CAN_ACTIONS["left"]] <= item["target_timestamp_ns"]
                       for item in selected)
            assert all(flags["raw_valid"] for item in selected for flags in item["source_vector_quality"].values())
            parquet_frames = [row for row in rows if row["episode_index"] == entry["dataset_episode_index"]]
            assert [row["frame_index"] for row in parquet_frames] == list(range(entry["frames"]))
        assert tuple(dataset[0]["action"].shape) == (14,)
    assert {item["source_episode_id"] for item in references if item["split"] == "train"}.isdisjoint(
        {item["source_episode_id"] for item in references if item["split"] == "test"})


def new_collection_episode(root, name, *, calibration_status="operator_supplied_pending_audit", state_lag_ns=0):
    """新采集格式：源声明待逐帧审计；训练统计与正式训练审计始终待办。"""
    from test_atomic_skill_export import annotate
    episode = fixture(root, name, frame_count=16, state_lag_ns=state_lag_ns)
    meta = annotate(episode)
    fields = meta["task_metadata"]
    fields.pop("normalization_version")
    fields["collection_contract_status"] = "metadata_ready_pending_data_audit"
    fields["training_eligibility"] = {"eligible": False, "status": "pending_independent_evidence_and_audit"}
    checks = {key: {"status": "declared_pending_audit", "detail": f"{key} 已声明，须逐帧审计"}
              for key in converter.SOURCE_DECLARATION_CHECKS}
    checks.update(calibration={"status": calibration_status, "detail": "相机身份、安装和内参须独立验收"},
                  training_normalization={"status": "not_computed", "required_source_split": "train",
                                          "requires_approved_source_audit": True,
                                          "requires_checkpoint_binding": True, "detail": "训练统计待计算并绑定 checkpoint"},
                  independent_audit={"status": "pending", "detail": "训练资格须由独立审计核准"})
    fields["collection_contract_checks"] = checks
    fields["collection_contract_blockers"] = [checks[key]["detail"] for key in
                                                ("calibration", "training_normalization", "independent_audit")]
    (episode / "episode.json").write_text(json.dumps(meta, ensure_ascii=False), encoding="utf-8")
    return episode


def write_source_acceptance(path, episodes):
    import hashlib
    evidence = path.with_name("synthetic_camera_review.txt")
    evidence.write_text("仅供合成测试：相机实体、安装批次、内参与版本对应关系已复核。", encoding="utf-8")
    records = {}
    for episode in episodes:
        meta = json.loads((episode / "episode.json").read_text())
        fields = meta["task_metadata"]
        records[meta["episode_id"]] = {
            "source_sha256": converter.source_digest(episode),
            **{key: fields[key] for key in ("arm_mode", "calibration_version", "data_convention_version")},
            "reviewed_by": "synthetic-reviewer", "reviewed_at_unix_ns": meta["stopped_at_unix_ns"] + 1,
            "checks": {"calibration": {"status": "verified", "evidence": [
                {"path": evidence.name, "sha256": hashlib.sha256(evidence.read_bytes()).hexdigest()}]}}}
    path.write_text(json.dumps({"schema_version": "piper_source_export_acceptance/1", "episodes": records}),
                    encoding="utf-8")
    return converter.load_source_acceptance(path)


def test_new_collection_export_separates_source_and_training_gates(tmp_path):
    import audit_real_data as audit
    episode = new_collection_episode(tmp_path, "episode_000001")
    before = converter.source_digest(episode)
    row = audit.audit_native(episode, max_source_skew_ms=30)
    assert not row["eligible"] and "calibration 尚未验收" in " ".join(row["reasons"])
    assert "training_normalization 尚未验收" not in " ".join(row["reasons"])
    assert "independent_audit 尚未验收" not in " ".join(row["reasons"])
    acceptance = write_source_acceptance(tmp_path / "acceptance.json", [episode])
    row = audit.audit_native(episode, max_source_skew_ms=30, source_acceptance=acceptance)
    assert row["eligible"] and row["eligible_frame_count"] == 16
    assert row["normalization_version"] is None and not row["training_ready"]
    assert row["training_normalization"]["status"] == "not_computed"
    assert row["deferred_training_checks"]["independent_audit"]["status"] == "pending"
    assert row["source_export_acceptance"]["record"]["reviewed_by"] == "synthetic-reviewer"
    assert before == converter.source_digest(episode)
    stored = json.loads((episode / "episode.json").read_text())["task_metadata"]
    assert stored["collection_contract_checks"]["calibration"]["status"] == "operator_supplied_pending_audit"
    assert stored["training_eligibility"]["eligible"] is False


@pytest.mark.parametrize("problem", ["unverified_mapping", "unknown_blocker", "unknown_pending", "source_digest", "version", "evidence", "review_time"])
def test_new_collection_gate_rejects_unresolved_or_mismatched_evidence(tmp_path, problem):
    import audit_real_data as audit
    episode = new_collection_episode(tmp_path, "episode_000001")
    path = episode / "episode.json"
    meta = json.loads(path.read_text())
    fields = meta["task_metadata"]
    if problem == "unverified_mapping":
        fields["collection_contract_checks"]["teleoperation_mapping"]["status"] = "unverified"
    elif problem == "unknown_blocker":
        fields["collection_contract_blockers"].append("实体来源仍有未知问题")
    elif problem == "unknown_pending":
        fields["collection_contract_checks"]["unknown_source_check"] = {"status": "pending", "detail": "未知来源检查"}
    path.write_text(json.dumps(meta, ensure_ascii=False))
    acceptance = write_source_acceptance(tmp_path / "acceptance.json", [episode])
    record = acceptance["episodes"][meta["episode_id"]]
    if problem == "source_digest":
        record["source_sha256"] = "0" * 64
    elif problem == "version":
        record["calibration_version"] += "-changed"
    elif problem == "evidence":
        (tmp_path / "synthetic_camera_review.txt").write_text("证据内容已改变", encoding="utf-8")
    elif problem == "review_time":
        record["reviewed_at_unix_ns"] = meta["stopped_at_unix_ns"] - 1
    row = audit.audit_native(episode, max_source_skew_ms=30, source_acceptance=acceptance)
    assert not row["eligible"] and row["eligible_frame_count"] == 0
    with pytest.raises((ValueError, OSError)):
        converter.read_episode(episode, ["left"], 10, 30_000_000, 2_000_000,
                               max_source_skew_ns=30_000_000, source_acceptance=acceptance)


def test_pending_declarations_still_require_actual_source_alignment(tmp_path):
    import audit_real_data as audit
    episode = new_collection_episode(tmp_path, "episode_000001", state_lag_ns=50_000_000)
    acceptance = write_source_acceptance(tmp_path / "acceptance.json", [episode])
    row = audit.audit_native(episode, max_source_skew_ms=30, source_acceptance=acceptance)
    assert not row["eligible"] and "总体时间差" in " ".join(row["reasons"])


def test_new_collection_with_independent_evidence_real_lerobot_roundtrip(tmp_path):
    from types import SimpleNamespace
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    source = tmp_path / "episodes"
    source.mkdir()
    episodes = [new_collection_episode(source, f"episode_{index:06d}") for index in (1, 2)]
    digests = [converter.source_digest(episode) for episode in episodes]
    acceptance_path = tmp_path / "acceptance.json"
    write_source_acceptance(acceptance_path, episodes)
    output = tmp_path / "dataset"
    args = SimpleNamespace(input_dir=source, output_dir=output, repo_id="local/piper_new_collection",
                           arm_mode="LEFT", input_format="joint-state", fps=10, max_age_ms=30,
                           jitter_ms=2, max_can_skew_ms=30, max_source_skew_ms=30,
                           success_sft=True, test_percent=50, source_acceptance=acceptance_path)
    converter.convert(args)
    manifest = json.loads((output / "conversion_manifest.json").read_text())
    assert manifest["eligibility_scope"] == "source_sft_export" and manifest["training_ready"] is False
    assert len(manifest["episodes"]) == 2
    for entry in manifest["episodes"]:
        assert entry["normalization_version"] is None
        assert entry["source_export_acceptance"]["record"]["checks"]["calibration"]["status"] == "verified"
        assert entry["deferred_training_checks"]["training_normalization"]["status"] == "not_computed"
        assert entry["deferred_training_checks"]["independent_audit"]["status"] == "pending"
        dataset = LeRobotDataset(f"local/piper_new_collection_{entry['split']}", root=output / entry["split"])
        assert dataset.meta.total_episodes == 1 and len(dataset) == 16
    assert digests == [converter.source_digest(episode) for episode in episodes]


def test_frame_before_terminal_stale_is_the_sft_cutoff(tmp_path, monkeypatch):
    from test_recorder_annotations import install_vector_deserializer, load_recorder_module
    module = load_recorder_module()
    base = 1_800_000_100_000_000_000
    def stopped_target(topic, index, message):
        if topic == converter.CAN_ACTIONS["left"] and index >= 20:
            s = converter.TYPESTORE
            Time, Header = s.types["builtin_interfaces/msg/Time"], s.types["std_msgs/msg/Header"]
            timestamp = base + 19 * 100_000_000
            message.header = Header(Time(timestamp // 1_000_000_000, timestamp % 1_000_000_000), "can_left")
            message.source_stamps = [Time((timestamp - offset * 1_000_000) // 1_000_000_000,
                                         (timestamp - offset * 1_000_000) % 1_000_000_000)
                                     for offset in (3, 2, 1, 0)]
            message.stale[:] = True
            message.valid = False
        return message
    episode = vector_fixture(tmp_path, "episode_000001", frame_count=35, period_ns=100_000_000,
                             transform=stopped_target)
    meta = json.loads((episode / "episode.json").read_text())
    start, stop = meta["started_at_unix_ns"], base + 3_420_000_000
    digest_before = converter.source_digest(episode)
    install_vector_deserializer(monkeypatch, lambda raw, _: converter.TYPESTORE.deserialize_cdr(raw, converter.VECTOR_TYPE))
    tail = module.terminal_stale_master_target(episode / "bag", "left", start, stop)
    assert tail["last_valid_header_ns"] == base + 1_900_000_000
    received, header = module.recorded_frame_before_header(episode / "bag", converter.CAMERAS["top_rgb"],
                                                          tail["last_valid_header_ns"], start, stop)
    assert header == base + 1_800_000_000
    assert digest_before == converter.source_digest(episode)
    meta.update(schema_version=2, mode="skill", skill="pick_and_place_banana_in_box", outcome="success",
                frame_topics=[converter.CAMERAS["top_rgb"]], stopped_at_unix_ns=stop, saved_at_unix_ns=stop+2,
                annotations=[{"kind": "skill_complete", "value": "pick_and_place_banana_in_box",
                              "annotation_origin": "duration_limit_before_terminal_stale",
                              "source_topic": converter.CAMERAS["top_rgb"], "header_stamp_ns": header,
                              "verified_in_bag": True, "reviewed_at_unix_ns": stop+1},
                             {"kind": "outcome", "value": "success", "annotation_origin": "save_selection",
                              "reviewed_at_unix_ns": stop+1}])
    meta["task_metadata"].update(arm_mode="LEFT", initial_conditions={"perturbation_id": "synthetic-scene"})
    (episode / "episode.json").write_text(json.dumps(meta), encoding="utf-8")
    label, reasons = converter.sft_label(meta)
    assert not reasons
    validated, frames = converter.read_episode(episode, ["left"], 10, 100_000_000, 2_000_000,
                                               input_format="can-vector", max_source_skew_ns=30_000_000,
                                               extract_valid_segments=True)
    selected, coverage = converter.select_sft_frames(validated, frames, label, ["left"], "can-vector")
    assert len(selected) == 19 and coverage["window_count"] == 4
    assert all(frame["__target_timestamp_ns__"] <= header for frame in selected)
    assert all(flags["raw_valid"] and not any(flags["raw_stale"])
               for frame in selected for flags in frame["__source_vector_quality__"].values())
