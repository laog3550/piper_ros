#!/usr/bin/env python3
# 用途：只订阅直连 CAN 解码话题，发布左右臂七维示教目标和实测反馈及完整性标记。
# 运行：ros2 run piper piper_arm_vector --ros-args -p gripper_mapping_verified:=false
"""ROS transport for the read-only, four-frame arm vector assembler."""

import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from piper_msgs.msg import PiperArmVector, PiperDirectCanFrame

from piper.piper_arm_vector import ArmVectorAssembler
from piper.collection_lock import CollectionLocks


def set_stamp(stamp, timestamp_ns):
    stamp.sec, stamp.nanosec = divmod(timestamp_ns, 1_000_000_000)


class PiperArmVectorNode(Node):
    def __init__(self):
        super().__init__('piper_arm_vector')
        max_age_ms = self.declare_parameter('max_age_ms', 100.0).value
        max_skew_ms = self.declare_parameter('max_skew_ms', 30.0).value
        publish_hz = self.declare_parameter('publish_hz', 50.0).value
        side = self.declare_parameter('side', 'all').value
        self._gripper_verified = self.declare_parameter(
            'gripper_mapping_verified', False).value
        if max_age_ms <= 0 or max_skew_ms < 0 or publish_hz <= 0:
            raise ValueError('invalid assembler timing parameters')
        if side not in ('all', 'left', 'right'):
            raise ValueError('side must be all, left, or right')
        self._streams = []
        sides = ('left', 'right') if side == 'all' else (side,)
        self._collection_locks = CollectionLocks(['/piper_vector/' + side for side in sides])
        for side in sides:
            for kind in ('master_target', 'slave_feedback'):
                assembler = ArmVectorAssembler(
                    side, kind, int(max_age_ms * 1e6), int(max_skew_ms * 1e6))
                output_topic = '/piper_vector/{}/{}'.format(side, kind)
                publisher = self.create_publisher(PiperArmVector, output_topic, 10)
                self.create_subscription(
                    PiperDirectCanFrame, assembler.source_topic,
                    lambda msg, a=assembler: a.add(msg), 1000)
                self._streams.append((assembler, publisher))
        self.create_timer(1.0 / publish_hz, self._publish)

    def _publish(self):
        now_ns = time.time_ns()
        for assembler, publisher in self._streams:
            value = assembler.snapshot(now_ns)
            # An entirely missing source has no Header to preserve. Wait for
            # the first real CAN frame; later missing/stale flags stay intact.
            if value['stamp_ns'] <= 0:
                continue
            msg = PiperArmVector()
            set_stamp(msg.header.stamp, value['stamp_ns'])
            msg.header.frame_id = value['frame_id']
            msg.source_topic = value['source_topic']
            msg.joint_names = list(value['joint_names'])
            msg.position = value['position']
            msg.source_can_ids = list(value['source_can_ids'])
            for target, stamp_ns in zip(msg.source_stamps, value['source_stamps_ns']):
                set_stamp(target, stamp_ns)
            msg.missing = value['missing']
            msg.stale = value['stale']
            msg.time_inconsistent = value['time_inconsistent']
            msg.valid = value['valid'] and self._gripper_verified
            msg.gripper_mapping_verified = self._gripper_verified
            publisher.publish(msg)

    def destroy_node(self):
        result = super().destroy_node()
        self._collection_locks.close()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PiperArmVectorNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
