#!/usr/bin/env python3
"""离线合成测试。运行：python3 -m unittest discover -s tools/vla_observation -p 'test_*.py'。"""

from copy import deepcopy
import json
from pathlib import Path
import sqlite3
import tempfile
from types import SimpleNamespace
import unittest

import numpy as np

from snapshot import CAMERAS, STATES, SnapshotBuilder, depth_array, roi_summary, run_bag
from tools.vla_contract.validate_contract import validate_observation

CONFIG = json.loads((Path(__file__).resolve().parents[2] / "config/vla_observation.example.json").read_text())
BASE = 1_800_000_000_000_000_000


def msg(ts, frame="optical", **fields):
    return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=ts // 1_000_000_000,
                                      nanosec=ts % 1_000_000_000), frame_id=frame), **fields)


def rgb(ts):
    return msg(ts, encoding="rgb8", width=640, height=480)


def depth(ts, value=1000, encoding="16UC1", frame="optical"):
    dtype = "<u2" if encoding == "16UC1" else "<f4"
    raw = np.full((480, 640), value, dtype=dtype)
    return msg(ts, frame, encoding=encoding, width=640, height=480, step=raw.strides[0],
               is_bigendian=0, data=raw.tobytes())


def info(ts, fx=500, frame="optical"):
    return msg(ts, frame, width=640, height=480, k=[fx, 0, 320, 0, 500, 240, 0, 0, 1])


def joints(ts):
    names = ["gripper"] + [f"joint{i}" for i in range(6, 0, -1)]
    return msg(ts, name=names, position=[0.03, 6, 5, 4, 3, 2, 1])


class SnapshotTests(unittest.TestCase):
    def setUp(self):
        self.builder = SnapshotBuilder(deepcopy(CONFIG))

    def add_camera(self, role, rgb_ts, depth_ts=None, info_ts=None, depth_msg=None, info_msg=None):
        topics = CAMERAS[role]
        self.builder.add(topics[0], rgb(rgb_ts), f"bag://{role}/rgb")
        if depth_ts is not None:
            self.builder.add(topics[1], depth_msg or depth(depth_ts), f"bag://{role}/depth")
        if info_ts is not None:
            self.builder.add(topics[2], info_msg or info(info_ts), f"bag://{role}/info")

    def test_cross_camera_nearest_and_internal_rgb_depth_pair(self):
        self.add_camera("top", BASE, BASE + 2_000_000, BASE + 2_000_000)
        self.add_camera("left_wrist", BASE - 20_000_000, BASE - 19_000_000, BASE - 19_000_000)
        self.builder.add(CAMERAS["left_wrist"][0], rgb(BASE + 4_000_000), "bag://left/newer")
        self.builder.add(CAMERAS["left_wrist"][1], depth(BASE + 5_000_000), "bag://left/depth")
        self.builder.add(STATES["left"], joints(BASE + 3_000_000), "bag://left/state")
        record = self.builder.build("LEFT", BASE, BASE + 50_000_000)
        self.assertTrue(record["contract_observation_valid"])
        self.assertEqual(record["sources"]["left_wrist"]["rgb"]["source_timestamp_ns"], BASE + 4_000_000)
        self.assertEqual(record["sources"]["left_wrist"]["rgb_depth_delta_ns"], 1_000_000)
        self.assertEqual(record["observation"]["state"]["left"]["position"], [1, 2, 3, 4, 5, 6, 0.03])
        self.assertEqual(record["sources"]["top"]["roi_depth"]["distance_median_m"], 1.0)
        self.assertFalse(record["sources"]["top"]["world_geometry"]["valid"])

    def test_missing_depth_and_state_are_explicit(self):
        self.add_camera("top", BASE)
        self.add_camera("left_wrist", BASE)
        record = self.builder.build("LEFT", BASE, BASE + 50_000_000)
        self.assertFalse(record["contract_observation_valid"])
        self.assertIsNone(record["observation"])
        self.assertEqual(record["sources"]["top"]["roi_depth"]["reason"], "missing_depth")
        self.assertFalse(record["sources"]["top"]["depth"]["stale"])
        self.assertFalse(record["sources"]["left_state"]["stale"])
        self.assertIn("left.state:missing", record["rejection_reasons"])

    def test_paired_rgb_wins_over_closer_unpaired_rgb(self):
        self.add_camera("top", BASE, BASE, BASE)
        self.add_camera("left_wrist", BASE - 20_000_000, BASE - 19_000_000, BASE - 19_000_000)
        self.builder.add(CAMERAS["left_wrist"][0], rgb(BASE), "bag://unpaired")
        self.builder.add(STATES["left"], joints(BASE), "bag://state")
        record = self.builder.build("LEFT", BASE, BASE + 50_000_000)
        self.assertEqual(record["sources"]["left_wrist"]["rgb"]["source_timestamp_ns"], BASE - 20_000_000)
        self.assertTrue(record["sources"]["left_wrist"]["roi_depth"]["valid"])

    def test_stale_and_pair_tolerance(self):
        self.add_camera("top", BASE, BASE + 20_000_000, BASE)
        self.add_camera("left_wrist", BASE, BASE, BASE)
        self.builder.add(STATES["left"], joints(BASE), "bag://state")
        record = self.builder.build("LEFT", BASE, BASE + 200_000_000)
        self.assertEqual(record["sources"]["top"]["rgb"]["reason"], "stale")
        self.assertEqual(record["sources"]["top"]["roi_depth"]["reason"], "missing_depth")
        self.assertFalse(record["contract_observation_valid"])

    def test_depth_units_and_invalid_intrinsics(self):
        self.assertAlmostEqual(roi_summary(depth(BASE, 1500), info(BASE), [320, 240, 1, 1], .001)
                               ["distance_median_m"], 1.5)
        self.assertAlmostEqual(roi_summary(depth(BASE, 1.5, "32FC1"), info(BASE), [320, 240, 1, 1], .001)
                               ["distance_median_m"], 1.5)
        with self.assertRaises(ValueError):
            roi_summary(depth(BASE), info(BASE, fx=0), [320, 240, 1, 1], .001)
        self.assertEqual(depth_array(depth(BASE, 0), .001)[240, 320], 0)

    def test_frame_mismatch_invalidates_geometry(self):
        self.add_camera("top", BASE, BASE, BASE, depth_msg=depth(BASE, frame="depth_optical"))
        record = self.builder.build("LEFT", BASE, BASE + 50_000_000)
        self.assertEqual(record["sources"]["top"]["roi_depth"]["reason"], "optical_frame_mismatch")

    def test_reject_invalid_config(self):
        config = deepcopy(CONFIG)
        del config["depth_scale_m_per_unit"]["top"]
        with self.assertRaises(ValueError):
            SnapshotBuilder(config)

    def test_sqlite_bag_to_snapshot(self):
        from rclpy.serialization import serialize_message
        from sensor_msgs.msg import CameraInfo, Image, JointState
        with tempfile.TemporaryDirectory() as temporary:
            bag = Path(temporary) / "bag"
            bag.mkdir()
            output = Path(temporary) / "snapshots.jsonl"
            connection = sqlite3.connect(bag / "sample.db3")
            connection.execute("CREATE TABLE topics(id INTEGER PRIMARY KEY, name TEXT, type TEXT)")
            connection.execute("CREATE TABLE messages(id INTEGER PRIMARY KEY, topic_id INTEGER, timestamp INTEGER, data BLOB)")
            entries = []
            for role in ("top", "left_wrist"):
                source_ts = BASE if role == "top" else BASE - 20_000_000
                for topic, kind in zip(CAMERAS[role], ("rgb", "depth", "info")):
                    if kind == "info":
                        source = info(source_ts)
                        message = CameraInfo()
                        message.width, message.height, message.k = source.width, source.height, [float(x) for x in source.k]
                    else:
                        source = rgb(source_ts) if kind == "rgb" else depth(source_ts)
                        message = Image()
                        message.width, message.height, message.encoding = source.width, source.height, source.encoding
                        message.step = source.width * (3 if kind == "rgb" else 2)
                        message.data = b"\0" * (source.width * source.height * 3) if kind == "rgb" else source.data
                    message.header.stamp.sec = source_ts // 1_000_000_000
                    message.header.stamp.nanosec = source_ts % 1_000_000_000
                    message.header.frame_id = "optical"
                    entries.append((topic, f"sensor_msgs/msg/{type(message).__name__}", message))
            state = JointState()
            state.header.stamp.sec = BASE // 1_000_000_000
            state.name, state.position = joints(BASE).name, [float(x) for x in joints(BASE).position]
            entries.append((STATES["left"], "sensor_msgs/msg/JointState", state))
            for index, (topic, msg_type, message) in enumerate(entries, 1):
                connection.execute("INSERT INTO topics VALUES(?,?,?)", (index, topic, msg_type))
                connection.execute("INSERT INTO messages VALUES(?,?,?,?)",
                                   (index, index, BASE + 100 - index, serialize_message(message)))
            connection.commit()
            connection.close()
            args = SimpleNamespace(bag=str(bag), output=str(output), arm_mode="LEFT", limit=0)
            run_bag(args, CONFIG)
            records = [json.loads(line) for line in output.read_text().splitlines()]
            self.assertEqual(len(records), 1)
            self.assertTrue(records[0]["contract_observation_valid"])
            self.assertEqual(records[0]["observation"]["contract_version"], "piper_vla_mvp/1.0")
            validate_observation(records[0]["observation"])
            self.assertEqual(records[0]["sources"]["left_wrist"]["rgb"]["source_timestamp_ns"],
                             BASE - 20_000_000)
            self.assertTrue(records[0]["observation"]["images"]["top_rgb"]["uri"].startswith("rosbag2://"))


if __name__ == "__main__":
    unittest.main()
