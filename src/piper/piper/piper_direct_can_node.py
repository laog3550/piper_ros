#!/usr/bin/env python3
# 用途：只读监听 can_left/can_right，逐帧发布主臂目标与从臂实测反馈。
# 运行：ros2 run piper piper_direct_can
import threading
import socket
import json
import queue
import time
import os
from pathlib import Path

import can
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from piper_msgs.msg import PiperDirectCanFrame
from std_msgs.msg import String

from piper.piper_direct_can_decode import C_PiperParserV2, FRAME_IDS, decode_frame, PiperMessage
from piper.collection_lock import CollectionLocks


def configure_transport():
    """Configure the CAN process before rclpy initializes the DDS participant."""
    implementation = os.environ.get('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')
    if implementation not in ('rmw_fastrtps_cpp', 'rmw_fastrtps_dynamic_cpp'):
        raise RuntimeError('当前 CAN 非阻塞发布配置只支持 Fast DDS，请设置 RMW_IMPLEMENTATION=rmw_fastrtps_cpp')
    profile = Path(__file__).with_name('direct_can_fastdds.xml')
    if not profile.is_file():
        raise RuntimeError(f'缺少 CAN DDS 配置：{profile}；请重新构建 piper')
    os.environ['RMW_IMPLEMENTATION'] = implementation
    os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = str(profile)
    os.environ['FASTDDS_DEFAULT_PROFILES_FILE'] = str(profile)
    os.environ['RMW_FASTRTPS_USE_QOS_FROM_XML'] = '1'
    os.environ['RMW_FASTRTPS_PUBLICATION_MODE'] = 'ASYNCHRONOUS'


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
        self._publish_stop = threading.Event()
        self._stats_lock = threading.Lock()
        self._fatal_error = None
        self._fatal_reported = False
        self._fatal_shutdown_at = None
        queue_depth = self.declare_parameter('receive_queue_depth', 4096).value
        publisher_depth = self.declare_parameter('publisher_qos_depth', 1000).value
        for name, value in (('receive_queue_depth', queue_depth), ('publisher_qos_depth', publisher_depth)):
            if isinstance(value, bool) or not isinstance(value, int) or not 1 <= value <= 100_000:
                raise ValueError(f'{name} 必须为 1 至 100000 的整数')
        receive_buffer_bytes = self.declare_parameter(
            'receive_buffer_bytes', 16_777_216).value
        if (isinstance(receive_buffer_bytes, bool)
                or not isinstance(receive_buffer_bytes, int)
                or receive_buffer_bytes < 0):
            raise ValueError('receive_buffer_bytes must be a non-negative integer')
        self._receive_buffer_bytes = receive_buffer_bytes
        self._frame_publishers = {}
        self._queues = {}
        self._stats = {}
        self._diagnostic_publishers = {}
        self._buffer_status = {}
        self._reported_buffers = set()
        raw_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=publisher_depth,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE)
        for channel in channels:
            channel_side = channel[4:]
            self._frame_publishers[channel] = {
                kind: self.create_publisher(
                    PiperDirectCanFrame, '/piper_direct/{}/{}'.format(channel_side, kind), raw_qos
                ) for kind in ('master_target', 'slave_feedback')
            }
            self._diagnostic_publishers[channel] = self.create_publisher(
                String, f'/piper_direct/{channel_side}/diagnostics', 10)
            for kind in self._frame_publishers[channel]:
                key = (channel, kind)
                self._queues[key] = queue.Queue(maxsize=queue_depth)
                self._stats[key] = dict(received=0, enqueued=0, published=0, filtered=0,
                                        overflow=0, publish_errors=0, shutdown_discarded=0,
                                        max_queue_wait_ms=0., max_publish_call_ms=0.,
                                        max_kernel_to_app_ms=0.,
                                        interval_max_queue_wait_ms=0.,
                                        interval_max_publish_call_ms=0.,
                                        interval_max_kernel_to_app_ms=0.,
                                        last_received_header_ns=0, last_published_header_ns=0,
                                        last_overflow_header_ns=0)
        self._receive_threads = [threading.Thread(target=self._listen, args=(channel,), daemon=True,
                                                  name=f'can_rx_{channel}') for channel in channels]
        self._publish_threads = [threading.Thread(target=self._publish_loop, args=key, daemon=True,
                                                  name=f'can_pub_{key[0]}_{key[1]}') for key in self._queues]
        self._threads = self._receive_threads + self._publish_threads
        self.create_timer(.2, self._report_health)
        for thread in self._publish_threads + self._receive_threads:
            thread.start()

    def _listen(self, channel):
        filters = [{'can_id': can_id, 'can_mask': 0x7FF, 'extended': False}
                   for can_id in sorted(FRAME_IDS)]
        try:
            with can.Bus(channel=channel, interface='socketcan', can_filters=filters) as bus:
                self._configure_receive_buffer(bus, channel)
                while not self._stop.is_set():
                    frame = bus.recv(timeout=0.2)
                    if frame is not None:
                        self._enqueue_frame(channel, frame)
        except can.CanError as exc:
            self._fail(f'CAN 接收线程 {channel} 已停止：{exc}')
        except Exception as exc:
            self._fail(f'CAN 接收线程 {channel} 异常：{type(exc).__name__}: {exc}')

    @staticmethod
    def _header_ns(frame):
        seconds = int(frame.timestamp)
        return seconds * 1_000_000_000 + int((frame.timestamp - seconds) * 1e9)

    def _fail(self, reason):
        # Receiver threads never call ROS logging or publishing, even on failure.
        with self._stats_lock:
            if self._fatal_error is None:
                self._fatal_error = reason

    def _enqueue_frame(self, channel, frame):
        kind = ('master_target' if frame.arbitration_id in (0x155, 0x156, 0x157, 0x159)
                else 'slave_feedback' if frame.arbitration_id in (0x2A5, 0x2A6, 0x2A7, 0x2A8) else None)
        if kind is None:
            return
        key = (channel, kind)
        header = self._header_ns(frame)
        kernel_to_app_ms = max(0., (time.time_ns() - header) / 1e6)
        overflow = False
        with self._stats_lock:
            self._stats[key]['received'] += 1
            self._stats[key]['last_received_header_ns'] = header
            self._stats[key]['max_kernel_to_app_ms'] = max(
                self._stats[key]['max_kernel_to_app_ms'], kernel_to_app_ms)
            self._stats[key]['interval_max_kernel_to_app_ms'] = max(
                self._stats[key]['interval_max_kernel_to_app_ms'], kernel_to_app_ms)
            try:
                self._queues[key].put_nowait((frame, time.monotonic_ns()))
            except queue.Full:
                self._stats[key]['overflow'] += 1
                self._stats[key]['last_overflow_header_ns'] = header
                overflow = True
            else:
                self._stats[key]['enqueued'] += 1
        if overflow:
            self._fail(f'{channel}/{kind} 接收队列溢出，采集不完整；停止节点并检查下游负载')

    def _publish_loop(self, channel, kind):
        key = (channel, kind)
        pending = self._queues[key]
        parser = C_PiperParserV2()
        decoded_message = PiperMessage()
        while not (self._publish_stop.is_set() and pending.empty()):
            try:
                frame, received = pending.get(timeout=.05)
            except queue.Empty:
                continue
            try:
                wait_ms = (time.monotonic_ns() - received) / 1e6
                with self._stats_lock:
                    self._stats[key]['max_queue_wait_ms'] = max(self._stats[key]['max_queue_wait_ms'], wait_ms)
                    self._stats[key]['interval_max_queue_wait_ms'] = max(
                        self._stats[key]['interval_max_queue_wait_ms'], wait_ms)
                if not rclpy.ok():
                    with self._stats_lock:
                        self._stats[key]['shutdown_discarded'] += 1
                    continue
                published = self._publish_frame(channel, frame, parser, decoded_message)
                with self._stats_lock:
                    self._stats[key]['published' if published else 'filtered'] += 1
                    if published:
                        self._stats[key]['last_published_header_ns'] = self._header_ns(frame)
            except Exception as exc:
                if not rclpy.ok():
                    with self._stats_lock:
                        self._stats[key]['shutdown_discarded'] += 1
                    continue
                with self._stats_lock:
                    self._stats[key]['publish_errors'] += 1
                self._fail(f'{channel}/{kind} 发布失败：{type(exc).__name__}: {exc}')
                return
            finally:
                pending.task_done()

    def diagnostics(self, channel):
        with self._stats_lock:
            streams = {kind: dict(self._stats[channel, kind]) for kind in self._frame_publishers[channel]}
            error = self._fatal_error
            buffer_status = self._buffer_status.get(channel)
            for kind in self._frame_publishers[channel]:
                stats = self._stats[channel, kind]
                stats['interval_max_kernel_to_app_ms'] = 0.
                stats['interval_max_queue_wait_ms'] = 0.
                stats['interval_max_publish_call_ms'] = 0.
        for kind, stats in streams.items():
            pending = self._queues[channel, kind]
            with pending.mutex:
                stats['queue_size'] = len(pending.queue)
                stats['queue_capacity'] = pending.maxsize
                stats['oldest_queue_wait_ms'] = ((time.monotonic_ns() - pending.queue[0][1]) / 1e6
                                                 if pending.queue else 0.)
        return {'schema_version': 'piper_direct_can_diagnostics/1', 'stamp_ns': time.time_ns(),
                'channel': channel, 'capture_failed': error is not None, 'error': error,
                'dds_profile': os.environ.get('FASTRTPS_DEFAULT_PROFILES_FILE'),
                'dds_qos_from_xml': os.environ.get('RMW_FASTRTPS_USE_QOS_FROM_XML'),
                'receive_buffer': buffer_status, 'streams': streams}

    def _report_health(self):
        for channel, publisher in self._diagnostic_publishers.items():
            snapshot = self.diagnostics(channel)
            buffer_status = snapshot['receive_buffer']
            if buffer_status is not None and channel not in self._reported_buffers:
                self._reported_buffers.add(channel)
                if buffer_status['warning']:
                    self.get_logger().warning(buffer_status['warning'])
                else:
                    self.get_logger().info(f"{channel}: SocketCAN 接收缓冲 {buffer_status['effective_bytes']} bytes")
            message = String()
            message.data = json.dumps(snapshot, ensure_ascii=False)
            try:
                publisher.publish(message)
            except Exception as exc:
                if not rclpy.ok():
                    return
                self._fail(f'{channel} 诊断发布失败：{exc}')
        with self._stats_lock:
            error = self._fatal_error
        if error and not self._fatal_reported:
            self._fatal_reported = True
            self.get_logger().error(error)
            self._stop.set()
            # Keep DDS alive for two more diagnostic periods so asynchronous
            # fatal reports can reach the recorder before participant teardown.
            self._fatal_shutdown_at = time.monotonic() + .4
        if error and self._fatal_shutdown_at is not None and time.monotonic() >= self._fatal_shutdown_at:
            if rclpy.ok():
                rclpy.shutdown()

    def _configure_receive_buffer(self, bus, channel):
        """Enlarge the SocketCAN receive queue so a short stall is not frame loss.

        The kernel caps the value at net.core.rmem_max; a warning here means the
        host must raise that limit before collecting training data.
        """
        requested = self._receive_buffer_bytes
        status = {'requested_bytes': requested, 'effective_bytes': None, 'warning': None}
        if not requested:
            with self._stats_lock:
                self._buffer_status[channel] = status
            return
        sock = getattr(bus, 'socket', None)
        if sock is None:
            status['warning'] = f'{channel}: 无法访问 SocketCAN socket，接收缓冲未调整'
        else:
            try:
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, requested)
                effective = sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)
                if isinstance(effective, int):
                    status['effective_bytes'] = effective
                    if effective < requested:
                        status['warning'] = (f'{channel}: 接收缓冲 {effective} bytes 小于请求 {requested} bytes；'
                                             '采集前请检查 net.core.rmem_max')
            except OSError as exc:
                status['warning'] = f'{channel}: 无法设置 SocketCAN 接收缓冲：{exc}'
        with self._stats_lock:
            self._buffer_status[channel] = status

    def _publish_frame(self, channel, frame, parser, decoded_message=None):
        value = decode_frame(frame, parser, decoded_message)
        if value is None:
            return False
        msg = PiperDirectCanFrame()
        seconds = int(value['stamp'])
        msg.header.stamp.sec = seconds
        msg.header.stamp.nanosec = int((value['stamp'] - seconds) * 1e9)
        msg.header.frame_id = channel
        for key in ('can_id', 'raw_data', 'joint_index', 'joint_position_rad',
                    'gripper_position_m', 'gripper_effort_nm', 'gripper_status',
                    'gripper_set_zero'):
            setattr(msg, key, value[key])
        started = time.monotonic_ns()
        try:
            self._frame_publishers[channel][value['kind']].publish(msg)
        finally:
            elapsed = (time.monotonic_ns() - started) / 1e6
            with self._stats_lock:
                stats = self._stats[channel, value['kind']]
                stats['max_publish_call_ms'] = max(stats['max_publish_call_ms'], elapsed)
                stats['interval_max_publish_call_ms'] = max(
                    stats['interval_max_publish_call_ms'], elapsed)
        return True

    def destroy_node(self):
        self._stop.set()
        for thread in self._receive_threads:
            if thread.ident is not None:
                thread.join(timeout=1.0)
        self._publish_stop.set()
        for thread in self._publish_threads:
            if thread.ident is not None:
                thread.join(timeout=2.0)
        if any(thread.is_alive() for thread in self._threads):
            # Do not destroy a publisher while a worker still owns its handle.
            raise RuntimeError('采集线程尚未退出，不能并发销毁 ROS 发布者；请检查发布阻塞')
        if rclpy.ok():
            self._report_health()
        else:
            for channel in self._frame_publishers:
                print(json.dumps(self.diagnostics(channel), ensure_ascii=False), flush=True)
        result = super().destroy_node()
        self._collection_locks.close()
        return result


def main(args=None):
    configure_transport()
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
