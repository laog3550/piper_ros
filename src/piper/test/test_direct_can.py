# 用途：验证直连 CAN 八类帧的 SDK 解码、原始数据及只读筛选。
# 运行：python3 -m pytest src/piper/test/test_direct_can.py
import math
import importlib.util
import pathlib
import sys
import types
import unittest
from unittest import mock

import can

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from piper.piper_direct_can_decode import decode_frame  # noqa: E402


def frame(can_id, data, **kwargs):
    options = {'is_extended_id': False}
    options.update(kwargs)
    return can.Message(arbitration_id=can_id, data=data, timestamp=1720000000.125,
                       **options)


class DirectCanDecodeTest(unittest.TestCase):
    def test_joint_pairs_from_both_roles(self):
        raw = (-90000).to_bytes(4, 'big', signed=True) + (180000).to_bytes(4, 'big', signed=True)
        for can_id, kind, index in ((0x155, 'master_target', 1),
                                    (0x156, 'master_target', 3),
                                    (0x157, 'master_target', 5),
                                    (0x2A5, 'slave_feedback', 1),
                                    (0x2A6, 'slave_feedback', 3),
                                    (0x2A7, 'slave_feedback', 5)):
            with self.subTest(can_id=hex(can_id)):
                result = decode_frame(frame(can_id, raw))
                self.assertEqual(result['kind'], kind)
                self.assertEqual(result['joint_index'], index)
                self.assertEqual(result['stamp'], 1720000000.125)
                self.assertEqual(result['raw_data'], list(raw))
                self.assertAlmostEqual(result['joint_position_rad'][0], -math.pi / 2)
                self.assertAlmostEqual(result['joint_position_rad'][1], math.pi)

    def test_gripper_target_and_feedback_are_distinct(self):
        target = ((12345).to_bytes(4, 'big', signed=True)
                  + (2500).to_bytes(2, 'big') + bytes((3, 0xAE)))
        actual = ((-8000).to_bytes(4, 'big', signed=True)
                  + (-1200).to_bytes(2, 'big', signed=True) + bytes((0x40, 0)))
        command = decode_frame(frame(0x159, target))
        measured = decode_frame(frame(0x2A8, actual))
        self.assertEqual(command['kind'], 'master_target')
        self.assertEqual(measured['kind'], 'slave_feedback')
        self.assertAlmostEqual(command['gripper_position_m'], 0.012345)
        self.assertAlmostEqual(measured['gripper_position_m'], -0.008)
        self.assertAlmostEqual(command['gripper_effort_nm'], 2.5)
        self.assertAlmostEqual(measured['gripper_effort_nm'], -1.2)
        self.assertEqual(command['gripper_status'], 3)
        self.assertEqual(command['gripper_set_zero'], 0xAE)
        self.assertEqual(measured['gripper_status'], 0x40)
        self.assertEqual(measured['gripper_set_zero'], 0)

    def test_only_complete_standard_data_frames_with_timestamps(self):
        raw = bytes(8)
        self.assertIsNone(decode_frame(frame(0x151, raw)))
        self.assertIsNone(decode_frame(frame(0x155, raw[:7])))
        self.assertIsNone(decode_frame(frame(0x155, raw, is_extended_id=True)))
        self.assertIsNone(decode_frame(frame(0x155, raw, is_remote_frame=True)))
        self.assertIsNone(decode_frame(frame(0x155, raw, is_fd=True)))
        self.assertIsNone(decode_frame(can.Message(arbitration_id=0x155, data=raw, timestamp=0)))

    def test_listener_receives_and_publishes_without_sending(self):
        publications = {}

        class Message:
            def __init__(self):
                self.header = types.SimpleNamespace(
                    stamp=types.SimpleNamespace(sec=0, nanosec=0), frame_id='')

        class Publisher:
            def __init__(self, topic, *_args, **_kwargs):
                self.topic = topic
                publications[topic] = []

            def publish(self, value):
                publications[self.topic].append(value)

        class FakeNode:
            def __init__(self, _name):
                self._publishers = []

            def create_publisher(self, _message_type, topic, _queue_size):
                publisher = Publisher(topic)
                self._publishers.append(publisher)
                return publisher

        fake_rclpy = types.ModuleType('rclpy')
        fake_rclpy.ok = mock.Mock(side_effect=(True, True, False))
        fake_rclpy.shutdown = mock.Mock()
        fake_rclpy_node = types.ModuleType('rclpy.node')
        fake_rclpy_node.Node = FakeNode
        fake_rclpy_core = types.ModuleType('rclpy._rclpy_pybind11')
        fake_rclpy_core.RCLError = type('RCLError', (RuntimeError,), {})
        fake_rclpy_executors = types.ModuleType('rclpy.executors')
        fake_rclpy_executors.ExternalShutdownException = type(
            'ExternalShutdownException', (RuntimeError,), {})
        fake_messages = types.ModuleType('piper_msgs.msg')
        fake_messages.PiperDirectCanFrame = Message

        bus = mock.MagicMock()
        bus.__enter__.return_value = bus
        bus.recv.side_effect = [
            frame(0x155, bytes(8)), frame(0x2A8, bytes(8))
        ]
        node_path = (pathlib.Path(__file__).resolve().parents[1]
                     / 'piper' / 'piper_direct_can_node.py')
        spec = importlib.util.spec_from_file_location('piper_direct_can_node_test', str(node_path))
        module = importlib.util.module_from_spec(spec)
        with mock.patch.dict(sys.modules, {'rclpy': fake_rclpy,
                                           'rclpy.node': fake_rclpy_node,
                                           'rclpy._rclpy_pybind11': fake_rclpy_core,
                                           'rclpy.executors': fake_rclpy_executors,
                                           'piper_msgs.msg': fake_messages}):
            spec.loader.exec_module(module)
            with mock.patch.object(can, 'Bus', return_value=bus) as bus_factory:
                with mock.patch.object(module.threading.Thread, 'start'):
                    node = module.PiperDirectCanNode()
                node._listen('can_left')

        bus_factory.assert_called_once()
        self.assertEqual(bus.recv.call_count, 2)
        bus.send.assert_not_called()
        bus.send_periodic.assert_not_called()
        self.assertEqual(len(publications['/piper_direct/left/master_target']), 1)
        self.assertEqual(len(publications['/piper_direct/left/slave_feedback']), 1)
        output = publications['/piper_direct/left/master_target'][0]
        self.assertEqual(output.header.stamp.sec, 1720000000)
        self.assertEqual(output.header.stamp.nanosec, 125000000)
        self.assertEqual(output.header.frame_id, 'can_left')
        self.assertEqual(output.can_id, 0x155)
        self.assertEqual(output.raw_data, [0] * 8)


if __name__ == '__main__':
    unittest.main()
