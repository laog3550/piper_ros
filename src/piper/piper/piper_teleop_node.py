"""ROS adapter for the Piper teleoperation state machine."""

import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from piper_msgs.msg import PiperEnableStatusMsg
from piper_msgs.srv import Enable

from piper.piper_feedback import JOINT_COUNT
from piper.piper_interfaces import arm_interface


STATUS_TIMEOUT_S = 1.0


class TeleopBridge(Node):
    """Own one side's subscriptions, command publisher and enable service."""

    def __init__(self, side, master_topic):
        super().__init__(f'piper_teleop_{side}')
        interface = arm_interface(side)
        self.interface = interface
        self.cmd_topic = interface.command_topic
        self.follower_topic = interface.feedback_topic
        self.status_topic = interface.enable_status_topic
        self.enable_service = interface.enable_service
        self.arm = interface.display_name
        self.master_topic = master_topic
        self.master = None
        self.follower = None
        self.master_gripper = None
        self.follower_gripper = None
        self.status = None
        self.status_time = None
        self.create_subscription(
            JointState, master_topic, self._on_master, 10)
        self.create_subscription(
            JointState, self.follower_topic, self._on_follower, 10)
        self.create_subscription(
            PiperEnableStatusMsg, self.status_topic, self._on_status, 10)
        self.publisher = self.create_publisher(
            JointState, self.cmd_topic, 10)
        self._enable_client = self.create_client(Enable, self.enable_service)

    @staticmethod
    def _angles(msg):
        if len(msg.position) < JOINT_COUNT:
            return None
        values = [float(value) for value in msg.position[:JOINT_COUNT]]
        if not all(math.isfinite(value) for value in values):
            return None
        return {index + 1: math.degrees(value)
                for index, value in enumerate(values)}

    @staticmethod
    def _gripper(msg):
        """Read the gripper opening in metres, or None when absent."""
        if len(msg.position) < JOINT_COUNT + 1:
            return None
        value = float(msg.position[JOINT_COUNT])
        return value if math.isfinite(value) else None

    def _on_master(self, msg):
        angles = self._angles(msg)
        if angles is not None:
            self.master = angles
        gripper = self._gripper(msg)
        if gripper is not None:
            self.master_gripper = gripper

    def _on_follower(self, msg):
        angles = self._angles(msg)
        if angles is not None:
            self.follower = angles
        gripper = self._gripper(msg)
        if gripper is not None:
            self.follower_gripper = gripper

    def _on_status(self, msg):
        self.status = msg
        self.status_time = time.monotonic()

    def enable_error(self):
        """Return None when the arm is confirmed enabled, else why not."""
        if self.status is None:
            return f'{self.status_topic} 上没有收到使能状态'
        age = time.monotonic() - self.status_time
        if age > STATUS_TIMEOUT_S:
            return f'使能状态已过期（{age:.1f}s 没有更新）'
        if not self.status.all_enabled:
            return (f'整臂未确认使能（state={self.status.state} '
                    f'all_enabled={self.status.all_enabled}）')
        return None

    def command_owner_error(self):
        """Reject a second publisher on the follower command interface."""
        own_name = self.get_name()
        own_namespace = self.get_namespace()
        conflicts = []
        for endpoint in self.get_publishers_info_by_topic(self.cmd_topic):
            if (endpoint.node_name == own_name
                    and endpoint.node_namespace == own_namespace):
                continue
            conflicts.append(
                f'{endpoint.node_namespace.rstrip("/")}/{endpoint.node_name}')
        if conflicts:
            owners = '、'.join(sorted(set(conflicts)))
            return f'{self.cmd_topic} 已有发布者：{owners}'
        return None

    def spin_for(self, seconds):
        """Pump callbacks for a bounded period."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def request_enabled(self, enabled, timeout=5.0):
        """Set enable state through this session's service client.

        Keeping this request in the teleop node removes the detached
        ``ros2 service call`` helper process and gives one owner responsibility
        for startup and shutdown sequencing.
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._enable_client.wait_for_service(timeout_sec=0.1):
                break
        else:
            return False, f'{self.enable_service} 服务不可用'

        request = Enable.Request()
        request.enable_request = bool(enabled)
        future = self._enable_client.call_async(request)
        remaining = max(0.0, deadline - time.monotonic())
        rclpy.spin_until_future_complete(self, future, timeout_sec=remaining)
        if not future.done():
            return False, f'{self.enable_service} 请求超时'
        try:
            response = future.result()
        except Exception as error:  # rclpy transports the service exception.
            return False, f'{self.enable_service} 请求失败：{error}'
        if response is None or not response.enable_response:
            return False, f'{self.enable_service} 拒绝请求'
        return True, None
