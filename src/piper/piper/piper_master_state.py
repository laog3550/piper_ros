#!/usr/bin/env python3
"""Publish one Piper arm's joint state from raw CAN frames, offset IDs included."""

# 用途：只读解析主臂偏移后的 0x2Cx 反馈，发布 ROS 状态用于观察。
# 主臂保持 0xFC，反馈和控制偏移为 0x20；实体按钮负责示教进出。
# 本节点不属于实时控制环，不发送配置、使能或运动帧。
# 关节弧度使用 math.pi / 180，夹爪单位为 m。

import argparse
import threading
import time
from typing import Dict, Optional, Tuple

import can
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from piper.piper_feedback import (
    DEG_TO_RAD,
    GRIPPER_FEEDBACK_CAN_ID,
    JOINT_ANGLE_IDS,
    JOINT_COUNT,
    decode_gripper,
    decode_joint_angles,
    parse_offset,
)

DEFAULT_OFFSET = '0x20'
DEFAULT_SIDE = 'left'
DEFAULT_RATE_HZ = 200.0
DEFAULT_STALE_TIMEOUT_S = 0.3
STATUS_PERIOD_S = 5.0
RECV_TIMEOUT_S = 0.2

EXIT_OK = 0
EXIT_FAILED = 3

JOINT_NAMES = [f'joint{joint}' for joint in range(1, JOINT_COUNT + 1)]
JOINT_NAMES.append('gripper')


def frame_filters(offset: int):
    """Return the socket filters for one arm's offset frame family."""
    return [
        {'can_id': can_id + offset, 'can_mask': 0x7FF, 'extended': False}
        for can_id in (*JOINT_ANGLE_IDS, GRIPPER_FEEDBACK_CAN_ID)
    ]


def build_joint_state(joints: Dict[int, float], gripper: Optional[float],
                      stamp=None) -> JointState:
    """Build the state piper_teleop expects: joints in radians, gripper in m."""
    message = JointState()
    if stamp is not None:
        message.header.stamp = stamp
    names = list(JOINT_NAMES)
    if gripper is None:
        names = names[:-1]
    message.name = names
    message.position = [joints[joint] * DEG_TO_RAD
                        for joint in sorted(joints)]
    if gripper is not None:
        message.position.append(gripper)
    return message


class OffsetFrameReader(threading.Thread):
    """Read one interface's offset joint/gripper frames; never transmits."""

    def __init__(self, port: str, offset: int):
        super().__init__(daemon=True)
        self.port = port
        self.offset = offset
        self.error: Optional[str] = None
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._angles: Dict[int, Tuple[float, float]] = {}
        self._gripper: Optional[Tuple[float, float]] = None
        self._frames = 0

    def _open(self) -> can.BusABC:
        """Open a receive-only view filtered to this arm's offset frames."""
        bus = can.Bus(
            interface='socketcan', channel=self.port,
            receive_own_messages=False,
        )
        bus.set_filters(frame_filters(self.offset))
        return bus

    def run(self) -> None:
        """Store the newest angles and gripper opening until asked to stop."""
        try:
            bus = self._open()
        except Exception as exc:  # noqa: BLE001 - 打开失败要如实报告
            self.error = str(exc)
            return
        try:
            while not self._stop_event.is_set():
                frame = bus.recv(timeout=RECV_TIMEOUT_S)
                if frame is None:
                    continue
                can_id = frame.arbitration_id - self.offset
                angles = decode_joint_angles(can_id, frame.data)
                gripper = decode_gripper(can_id, frame.data)
                if not angles and gripper is None:
                    continue
                now = time.monotonic()
                with self._lock:
                    self._frames += 1
                    for joint, degrees in angles.items():
                        self._angles[joint] = (degrees, now)
                    if gripper is not None:
                        self._gripper = (gripper, now)
        finally:
            bus.shutdown()

    def stop(self) -> None:
        """Ask the reader thread to finish."""
        self._stop_event.set()

    def snapshot(self):
        """Return (joints in degrees, gripper in metres, frame count)."""
        with self._lock:
            joints = {joint: degrees for joint, (degrees, _) in
                      self._angles.items()}
            gripper = self._gripper[0] if self._gripper else None
            return joints, gripper, self._frames

    def age(self, now: float) -> Optional[float]:
        """Return how old the newest frame is, or None when nothing arrived."""
        with self._lock:
            stamps = [stamp for _, stamp in self._angles.values()]
            if self._gripper is not None:
                stamps.append(self._gripper[1])
            return None if not stamps else now - max(stamps)

    def gripper_age(self, now: float) -> Optional[float]:
        """Return how old the newest gripper frame is, or None."""
        with self._lock:
            return None if self._gripper is None else now - self._gripper[1]


class MasterStatePublisher(Node):
    """Publish one offset arm's JointState for the host-bridge teleop."""

    def __init__(self, side: str, port: str, offset: int, topic: str,
                 rate_hz: float, stale_timeout_s: float):
        super().__init__(f'piper_master_state_{side}')
        self.side = side
        self.port = port
        self.offset = offset
        self.stale_timeout_s = stale_timeout_s
        self.reader = OffsetFrameReader(port, offset)
        self.publisher = self.create_publisher(JointState, topic, 1)
        self.topic = topic
        self._published = 0
        self._last_stale_log = 0.0
        self._last_status_log = time.monotonic()
        self._last_frames = 0
        self._announced = False
        self.reader.start()
        self.create_timer(1.0 / rate_hz, self._tick)

    def _build(self, joints: Dict[int, float], gripper: Optional[float]):
        """Build one JointState; the gripper is appended only when fresh."""
        return build_joint_state(
            joints, gripper, self.get_clock().now().to_msg())

    def _tick(self) -> None:
        """Publish the newest state, or hold off while the arm is silent."""
        now = time.monotonic()
        joints, gripper, frames = self.reader.snapshot()
        age = self.reader.age(now)
        fresh = (len(joints) == JOINT_COUNT and age is not None
                 and age <= self.stale_timeout_s)
        if not fresh:
            if now - self._last_stale_log >= 1.0:
                self._last_stale_log = now
                state = '尚未收到任何帧' if age is None else f'最新帧 {age:.2f}s 前'
                self.get_logger().warn(
                    f'{self.port} 上读不到 {self.offset:#04x} 偏移的关节帧'
                    f'（{state}），暂不发布'
                )
            return
        gripper_age = self.reader.gripper_age(now)
        if gripper_age is not None and gripper_age > self.stale_timeout_s:
            gripper = None
        if not self._announced:
            self._announced = True
            self.get_logger().info(
                f'已读到 {self.port} 上偏移 {self.offset:#04x} 的帧，'
                f'开始发布到 {self.topic}'
            )
        self.publisher.publish(self._build(joints, gripper))
        self._published += 1
        self._log_status(now, frames, gripper)

    def _log_status(self, now: float, frames: int, gripper) -> None:
        """Print a compact status line every few seconds."""
        if now - self._last_status_log < STATUS_PERIOD_S:
            return
        elapsed = now - self._last_status_log
        rate = (self._published / elapsed) if self._published else 0.0
        frame_rate = (frames - self._last_frames) / elapsed
        gripper_text = ('无夹爪帧' if gripper is None
                        else f'{gripper * 1000:+.2f} mm')
        self.get_logger().info(
            f'发布 {rate:.0f} Hz，帧 {frame_rate:.0f} 帧/秒，'
            f'夹爪 {gripper_text} → {self.topic}'
        )
        self._last_status_log = now
        self._last_frames = frames
        self._published = 0

    def shutdown(self) -> None:
        """Stop the reader thread and close its socket."""
        self.reader.stop()


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--port', required=True, metavar='IFACE',
        help='该臂所在的 CAN 接口，例如 can_left（必填）',
    )
    parser.add_argument(
        '--offset', default=DEFAULT_OFFSET, metavar='HEX',
        help=f'反馈 ID 偏移，0x00/0x10/0x20（默认 {DEFAULT_OFFSET}）',
    )
    parser.add_argument(
        '--side', default=DEFAULT_SIDE, choices=('left', 'right'),
        help=f'侧别，用于节点名与默认话题（默认 {DEFAULT_SIDE}）',
    )
    parser.add_argument(
        '--topic', default=None, metavar='TOPIC',
        help='发布话题（默认 /joint_states_master_<side>）',
    )
    parser.add_argument(
        '--rate-hz', type=float, default=DEFAULT_RATE_HZ,
        help=f'发布频率（默认 {DEFAULT_RATE_HZ}）',
    )
    parser.add_argument(
        '--stale-timeout', type=float, default=DEFAULT_STALE_TIMEOUT_S,
        help=f'超过多久没有新帧就停止发布，秒（默认 '
             f'{DEFAULT_STALE_TIMEOUT_S}）',
    )
    return parser


def main(args=None) -> int:
    """Run the read-only offset state publisher until interrupted."""
    options = _parser().parse_args(args)
    if options.rate_hz <= 0:
        print('错误：--rate-hz 必须为正')
        return EXIT_FAILED
    if options.stale_timeout <= 0:
        print('错误：--stale-timeout 必须为正')
        return EXIT_FAILED
    try:
        offset = parse_offset(options.offset)
    except ValueError as exc:
        print(f'错误：{exc}')
        return EXIT_FAILED
    topic = options.topic or f'/joint_states_master_{options.side}'

    print(f'只读读取：从 {options.port} 读偏移 '
          f'{offset:#04x} 的关节/夹爪反馈，发布到 {topic}')
    print('不会发送任何帧（无使能、失能、模式或运动指令）')
    rclpy.init()
    node = MasterStatePublisher(
        options.side, options.port, offset, topic,
        options.rate_hz, options.stale_timeout,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n收到中断，停止发布')
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()
    return EXIT_OK


if __name__ == '__main__':
    raise SystemExit(main())
