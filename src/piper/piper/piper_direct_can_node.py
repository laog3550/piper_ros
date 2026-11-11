#!/usr/bin/env python3
# 用途：只读监听 can_left/can_right，逐帧发布主臂目标与从臂实测反馈。
# 运行：ros2 run piper piper_direct_can
import threading
import socket

import can
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from piper_msgs.msg import PiperDirectCanFrame

from piper.piper_direct_can_decode import C_PiperParserV2, FRAME_IDS, decode_frame
from piper.collection_lock import CollectionLocks


class PiperDirectCanNode(Node):
    def __init__(self):
        super().__init__('piper_direct_can')
        side = self.declare_parameter('side', 'all').value
        if side not in ('all', 'left', 'right'):
            raise ValueError('side must be all, left, or right')
        channels = ('can_left', 'can_right') if side == 'all' else ('can_' + side,)
        self._collection_locks = CollectionLocks(
            ['/piper_direct/' + channel[4:] for channel in channels])
        self._stop = threading.Event()
        receive_buffer_bytes = self.declare_parameter(
            'receive_buffer_bytes', 16_777_216).value
        if (isinstance(receive_buffer_bytes, bool)
                or not isinstance(receive_buffer_bytes, int)
                or receive_buffer_bytes < 0):
            raise ValueError('receive_buffer_bytes must be a non-negative integer')
        self._receive_buffer_bytes = receive_buffer_bytes
        self._frame_publishers = {}
        for channel in channels:
            channel_side = channel[4:]
            self._frame_publishers[channel] = {
                kind: self.create_publisher(
                    PiperDirectCanFrame, '/piper_direct/{}/{}'.format(channel_side, kind), 1000
                ) for kind in ('master_target', 'slave_feedback')
            }
        self._threads = [threading.Thread(target=self._listen, args=(channel,), daemon=True)
                         for channel in self._frame_publishers]
        for thread in self._threads:
            thread.start()

    def _listen(self, channel):
        parser = C_PiperParserV2()
        filters = [{'can_id': can_id, 'can_mask': 0x7FF, 'extended': False}
                   for can_id in sorted(FRAME_IDS)]
        try:
            with can.Bus(channel=channel, interface='socketcan', can_filters=filters) as bus:
                self._configure_receive_buffer(bus, channel)
                while rclpy.ok() and not self._stop.is_set():
                    frame = bus.recv(timeout=0.2)
                    if frame is not None:
                        self._publish_frame(channel, frame, parser)
        except RCLError:
            if rclpy.ok():
                raise
        except can.CanError as exc:
            self.get_logger().error('CAN listener {} stopped: {}'.format(channel, exc))
            self._stop.set()
            if rclpy.ok():
                rclpy.shutdown()

    def _configure_receive_buffer(self, bus, channel):
        """Enlarge the SocketCAN receive queue so a short stall is not frame loss.

        The kernel caps the value at net.core.rmem_max; a warning here means the
        host must raise that limit before collecting training data.
        """
        requested = self._receive_buffer_bytes
        if not requested:
            return
        sock = getattr(bus, 'socket', None)
        if sock is None:
            self.get_logger().warning(
                '{}: cannot access the SocketCAN socket; receive buffer unchanged'.format(channel))
            return
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, requested)
            effective = sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)
        except OSError as exc:
            self.get_logger().warning(
                '{}: cannot set SocketCAN receive buffer: {}'.format(channel, exc))
            return
        if not isinstance(effective, int):
            return
        if effective < requested:
            self.get_logger().warning(
                '{}: SocketCAN receive buffer is {} bytes (requested {}); '
                'raise net.core.rmem_max before collection'.format(channel, effective, requested))
        else:
            self.get_logger().info(
                '{}: SocketCAN receive buffer {} bytes'.format(channel, effective))

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
        self._frame_publishers[channel][value['kind']].publish(msg)

    def destroy_node(self):
        self._stop.set()
        for thread in self._threads:
            thread.join(timeout=1.0)
        result = super().destroy_node()
        self._collection_locks.close()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PiperDirectCanNode()
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
