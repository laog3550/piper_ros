"""合成 rosbag2 离线转换检查。运行：pytest -q tools/episode_recorder/test_to_lerobot.py。"""

import json
from pathlib import Path
import sys

import numpy as np
import pytest
from rosbags.rosbag2 import Writer

sys.path.insert(0, str(Path(__file__).parent))
import to_lerobot as converter


def fixture(root, name, *, missing_action=False, bad_dimension=False, bad_stamp=False, arm_mode="LEFT"):
    episode = root / name
    bag = episode / "bag"
    episode.mkdir()
    (episode / "episode.json").write_text(json.dumps({
        "schema_version": 1, "state": "saved", "episode_id": name, "task": "抓取方块",
        "started_at_unix_ns": 1_799_999_999_900_000_000,
        "stopped_at_unix_ns": 1_800_000_000_200_000_000,
        "task_metadata": {"arm_mode": arm_mode, "time_domain": "unix_ns",
                          "action_space": "absolute_joint_position_7",
                          "action_provenance": "human_demonstration",
                          "calibration_version": "synthetic-calibration",
                          "normalization_version": "synthetic-normalization"}}), encoding="utf-8")
    s = converter.TYPESTORE
    Header = s.types["std_msgs/msg/Header"]
    Time = s.types["builtin_interfaces/msg/Time"]
    Joint = s.types["sensor_msgs/msg/JointState"]
    Image = s.types["sensor_msgs/msg/Image"]
    arms = ["left", "right"] if arm_mode == "BIMANUAL" else [arm_mode.lower()]
    topics = [converter.CAMERAS["top_rgb"], *(converter.CAMERAS[f"{arm}_wrist_rgb"] for arm in arms),
              *(converter.STATES[arm] for arm in arms)]
    if not missing_action:
        topics.extend(converter.ACTIONS[arm] for arm in arms)
    base = 1_800_000_000_000_000_000
    with Writer(bag, version=Writer.VERSION_LATEST) as writer:
        connections = {topic: writer.add_connection(topic, converter.TYPES[topic], typestore=s) for topic in topics}
        for index in range(2):
            timestamp = base + index * 100_000_000
            for topic in topics:
                actual = timestamp + (10_000_000 if bad_stamp and index == 1 and topic == converter.ACTIONS[arms[0]] else 0)
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
    output = tmp_path / "output"
    args = SimpleNamespace(input_dir=source, output_dir=output, repo_id="local/piper",
                           arm_mode="LEFT", fps=10, max_age_ms=20, jitter_ms=2, test_percent=50)
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
