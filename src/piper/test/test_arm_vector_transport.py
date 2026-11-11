"""验证全缺失向量不发布零 Header，已有来源的 stale 标志仍保留。"""

import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def test_missing_source_waits_but_stale_source_keeps_original_header():
    modules = {name: ModuleType(name) for name in
               ('rclpy', 'rclpy.node', 'rclpy.executors', 'piper_msgs.msg')}
    modules['rclpy.node'].Node = object
    modules['rclpy.executors'].ExternalShutdownException = RuntimeError
    def message():
        return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(), frame_id=''),
                               source_stamps=[SimpleNamespace() for _ in range(4)])
    modules['piper_msgs.msg'].PiperArmVector = message
    modules['piper_msgs.msg'].PiperDirectCanFrame = object
    path = Path(__file__).resolve().parents[1] / 'piper/piper_arm_vector_node.py'
    spec = importlib.util.spec_from_file_location('vector_transport_under_test', path)
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    node = module.PiperArmVectorNode.__new__(module.PiperArmVectorNode)
    node._gripper_verified = True
    publisher, assembler = Mock(), Mock()
    node._streams = [(assembler, publisher)]
    assembler.snapshot.return_value = {'stamp_ns': 0}
    node._publish()
    publisher.publish.assert_not_called()
    stamp = 1_790_000_000_123_456_789
    assembler.snapshot.return_value = {
        'stamp_ns': stamp, 'frame_id': 'can_left',
        'source_topic': '/piper_direct/left/master_target',
        'joint_names': [f'joint{i}' for i in range(1, 7)] + ['gripper'],
        'position': [0.] * 7, 'source_can_ids': [0x155, 0x156, 0x157, 0x159],
        'source_stamps_ns': [stamp] * 4, 'missing': [False] * 4,
        'stale': [True] * 4, 'time_inconsistent': False, 'valid': False}
    node._publish()
    emitted = publisher.publish.call_args.args[0]
    assert (emitted.header.stamp.sec * 1_000_000_000 + emitted.header.stamp.nanosec) == stamp
    assert emitted.stale == [True] * 4 and not emitted.valid
