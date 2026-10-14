#!/usr/bin/env python3
# 用途：只读监听 can_left/can_right，逐帧发布主臂目标与从臂实测反馈。
# 运行：ros2 run piper piper_direct_can
import threading

import can
import rclpy
from rclpy.node import Node
from piper_msgs.msg import PiperDirectCanFrame

from piper.piper_direct_can_decode import C_PiperParserV2, FRAME_IDS, decode_frame


class PiperDirectCanNode(Node):
    def __init__(self):
        super().__init__('piper_direct_can')
        self._stop = threading.Event()
        self._publishers = {}
        for channel in ('can_left', 'can_right'):
            side = channel[4:]
            self._publishers[channel] = {
                kind: self.create_publisher(
                    PiperDirectCanFrame, '/piper_direct/{}/{}'.format(side, kind), 1000
                ) for kind in ('master_target', 'slave_feedback')
            }
        self._threads = [threading.Thread(target=self._listen, args=(channel,), daemon=True)
                         for channel in self._publishers]
        for thread in self._threads:
            thread.start()

    def _listen(self, channel):
        parser = C_PiperParserV2()
        filters = [{'can_id': can_id, 'can_mask': 0x7FF, 'extended': False}
                   for can_id in sorted(FRAME_IDS)]
        try:
            with can.Bus(channel=channel, interface='socketcan', can_filters=filters) as bus:
                while rclpy.ok() and not self._stop.is_set():
                    frame = bus.recv(timeout=0.2)
                    if frame is not None:
                        self._publish_frame(channel, frame, parser)
        except can.CanError as exc:
            self.get_logger().error('CAN listener {} stopped: {}'.format(channel, exc))
            self._stop.set()
            if rclpy.ok():
                rclpy.shutdown()

    def _publish_frame(self, channel, frame, parser):
        value = decode_frame(frame, parser)
        if value is None:
            return
        msg = PiperDirectCanFrame()
        seconds = int(value['stamp'])
        msg.header.stamp.sec = seconds
        msg.header.stamp.nanosec = int((value['stamp'] - seconds) * 1e9)
        msg.header.frame_id = channel
        for key in ('can_id', 'raw_data', 'joint_index', 'joint_position_rad',
                    'gripper_position_m', 'gripper_effort_nm', 'gripper_status',
                    'gripper_set_zero'):
            setattr(msg, key, value[key])
        self._publishers[channel][value['kind']].publish(msg)

    def destroy_node(self):
        self._stop.set()
        for thread in self._threads:
            thread.join(timeout=1.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PiperDirectCanNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
