#!/usr/bin/env python3
"""Run both sides' two-CAN teleop sessions with a ROS start/stop interface."""

# 用途：把 piper_two_can_teleop 的单侧会话跑成左右两条同时运行的服务化进程。
#
# 分工（与 docs/TWO_CAN_HOST_TELEOP_IMPLEMENTATION.md 第 5、9 节一致）：
#   - 每侧一个 TeleopSession 在自己的线程里跑实时 CAN 环，运动数据不经过 ROS；
#   - 本节点只做启停服务、双侧联锁、状态与诊断发布；
#   - 双侧联锁：/teleop/start 要求两侧都先通过只读门禁；两侧对齐完成后一起进入跟随。
#
# 默认干跑：不加 --send 时两侧只执行只读门禁，start 服务会拒绝使能与运动。

from argparse import ArgumentParser
import threading
import time
from typing import Dict, Optional

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from std_srvs.srv import Trigger

from piper_msgs.msg import PiperTeleopStatusMsg

from piper.piper_two_can_align import SafetyError
from piper.piper_two_can_teleop import (
    EXIT_FAILED,
    install_sigterm_handler,
    EXIT_OK,
    EXIT_PENDING_RELEASE,
    EXIT_REFUSED,
    OperatorLink,
    TeleopSession,
    TeleopState,
    argument_parser,
    build_config,
)

SIDES = ('left', 'right')
DEFAULT_DUAL_BARRIER_TIMEOUT_S = 60.0
STATUS_PERIOD_S = 0.5
SHUTDOWN_GRACE_S = 10.0


def _service_name(side: str, action: str) -> str:
    return f'/teleop/{side}/{action}'


class ActiveBarrier:
    """Hold both sides until both have finished aligning."""

    def __init__(self, sides: int = 2,
                 timeout_s: float = DEFAULT_DUAL_BARRIER_TIMEOUT_S):
        self._remaining = sides
        self._lock = threading.Lock()
        self._ready = threading.Event()
        self.timeout_s = float(timeout_s)

    def wait(self, keep_receiving) -> None:
        """Wait without letting CAN feedback age in the receive queue."""
        with self._lock:
            self._remaining -= 1
            if self._remaining == 0:
                self._ready.set()
        deadline = time.monotonic() + self.timeout_s
        while not self._ready.is_set():
            keep_receiving()
            if time.monotonic() >= deadline:
                raise SafetyError('等待另一侧完成对齐超时，双侧同步启动失败')
        keep_receiving()


class ServiceOperator(OperatorLink):
    """An operator channel driven by ROS services, not a terminal."""

    def __init__(self, side: str, log):
        self.side = side
        self.log = log
        self._start = threading.Event()
        self._teach = threading.Event()
        self._release = threading.Event()
        self._lock = threading.Lock()
        self._stop_reason: Optional[str] = None
        self._aborted = threading.Event()
        self._waiting: Optional[str] = None

    # ---- 服务回调 -----------------------------------------------------

    def request_start(self) -> None:
        """Accept a start request from a service callback."""
        self._start.set()

    def confirm_teach_engaged(self) -> None:
        """Accept the operator's 'master is teaching' confirmation."""
        self._teach.set()

    def confirm_teach_released(self) -> None:
        """Accept the operator's 'teach button released' confirmation."""
        self._release.set()

    def request_stop(self, reason: str) -> None:
        """Ask the session to stop streaming and stand by."""
        with self._lock:
            self._stop_reason = reason

    def abort(self) -> None:
        """Unblock every wait, for a prompt shutdown."""
        self._aborted.set()
        self._start.set()
        self._teach.set()
        self._release.set()

    @property
    def waiting_for(self) -> Optional[str]:
        """Which operator confirmation this side is waiting for."""
        return self._waiting

    # ---- OperatorLink -------------------------------------------------

    def notify(self, message: str) -> None:
        """Print one line of session narration."""
        self.log(message)

    def _wait(self, event: threading.Event, name: str,
              timeout_s: float) -> bool:
        """Wait for one confirmation until it arrives, aborts or times out."""
        self._waiting = name
        deadline = time.monotonic() + max(float(timeout_s), 0.0)
        try:
            while True:
                if self._aborted.is_set():
                    return False
                if name != 'teach_released' and self.poll_stop():
                    raise KeyboardInterrupt
                if event.wait(timeout=0.01):
                    return not self._aborted.is_set()
                if time.monotonic() >= deadline:
                    return False
        finally:
            self._waiting = None

    def require_start(self, timeout_s: float) -> None:
        """Wait for the start service; refuse on timeout or abort."""
        if not self._wait(self._start, 'start', timeout_s):
            raise SafetyError('等待 start 服务超时或已中止，拒绝进入摇操')

    def wait_teach_engaged(self, timeout_s: float) -> bool:
        """Wait for the teach_engaged service call."""
        return self._wait(self._teach, 'teach_engaged', timeout_s)

    def poll_stop(self) -> Optional[str]:
        """Return the pending stop reason, if a service asked for one."""
        with self._lock:
            reason = self._stop_reason
            self._stop_reason = None
        return reason

    def wait_teach_released(self, timeout_s: float) -> bool:
        """Wait for the teach_released service call."""
        return self._wait(self._release, 'teach_released', timeout_s)

    def abort_waits(self) -> None:
        """Return from any blocked wait immediately."""
        self.abort()


class SideRunner:
    """One side's session plus the thread and operator that drive it."""

    def __init__(self, side: str, config, log):
        self.side = side
        self.operator = ServiceOperator(side, log)
        self.session = TeleopSession(config, self.operator, log=log)
        self.thread: Optional[threading.Thread] = None
        self.exit_code: Optional[int] = None

    def start(self) -> None:
        """Run the session in its own thread."""
        self.thread = threading.Thread(
            target=self._run, name=f'teleop_{self.side}', daemon=True)
        self.thread.start()

    def _run(self) -> None:
        try:
            self.exit_code = self.session.run()
        except BaseException as exc:      # noqa: BLE001 - report, never hide
            self.session.fault = f'会话线程异常：{exc!r}'
            self.session.state = TeleopState.FAULT_LATCHED
            self.exit_code = EXIT_FAILED

    @property
    def alive(self) -> bool:
        """Whether the session thread is still running."""
        return self.thread is not None and self.thread.is_alive()

    @property
    def state(self) -> TeleopState:
        """Return this side's session state."""
        return self.session.state

    def join(self, timeout_s: float) -> bool:
        """Wait for the session thread; False when it is still running."""
        if self.thread is None:
            return True
        self.thread.join(timeout_s)
        return not self.thread.is_alive()


class TeleopManager(Node):
    """Expose the two-CAN teleop sessions as ROS services and status."""

    def __init__(self, configs: Dict[str, object], *, send: bool,
                 barrier_timeout_s: float = DEFAULT_DUAL_BARRIER_TIMEOUT_S):
        super().__init__('piper_two_can_manager')
        self._sides: Dict[str, SideRunner] = {}
        self._send = send
        self._barrier_timeout_s = barrier_timeout_s
        self._barrier: Optional[ActiveBarrier] = None
        for side in SIDES:
            self._sides[side] = SideRunner(side, configs[side], self._log)
        self._status_publishers = {
            side: self.create_publisher(PiperTeleopStatusMsg, '/teleop/state',
                                        10)
            for side in SIDES
        }
        self._diagnostics = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10)
        self._services = []
        for side in SIDES:
            self._services.append(self.create_service(
                Trigger, _service_name(side, 'start'),
                self._make_side_start(side)))
            self._services.append(self.create_service(
                Trigger, _service_name(side, 'stop'),
                self._make_side_stop(side)))
            self._services.append(self.create_service(
                Trigger, _service_name(side, 'teach_engaged'),
                self._make_side_teach(side, engaged=True)))
            self._services.append(self.create_service(
                Trigger, _service_name(side, 'teach_released'),
                self._make_side_teach(side, engaged=False)))
        self.create_service(Trigger, '/teleop/start', self._dual_start)
        self.create_service(Trigger, '/teleop/stop', self._dual_stop)
        self.create_timer(STATUS_PERIOD_S, self._publish_status)
        for side in SIDES:
            self._sides[side].start()
        self._log('两 CAN 摇操管理器已启动：'
                  + ('--send 已关闭：只做只读门禁，start 服务会拒绝使能与运动。'
                     if not send else '--send 已开启：start 服务可以真正使能与运动。'))

    # ---- 日志与状态 ---------------------------------------------------

    def _log(self, message: str) -> None:
        self.get_logger().info(str(message).strip())

    def _publish_status(self) -> None:
        if self._barrier is not None and any(
                runner.session.fault is not None
                or not runner.alive for runner in self._sides.values()):
            for runner in self._sides.values():
                runner.operator.request_stop('双侧联锁：另一侧故障或已退出')
            self._barrier = None
        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = self.get_clock().now().to_msg()
        for side in SIDES:
            status = self._sides[side].session.status()
            message = PiperTeleopStatusMsg()
            message.side = status.side
            message.state = status.state.value
            message.state_label = status.label
            message.fault = status.fault or ''
            message.running = status.state is TeleopState.ACTIVE
            message.command_hz = float(status.command_hz)
            message.master_hz = float(status.master_hz)
            message.follower_hz = float(status.follower_hz)
            message.can_error_frames = int(status.error_frames)
            message.can_error_rate_hz = float(status.error_rate_hz)
            message.tracking_error_deg = float(status.tracking_error_deg)
            message.master_gripper_m = (
                float(status.master_gripper_m)
                if status.master_gripper_m is not None else float('nan'))
            message.follower_gripper_m = (
                float(status.follower_gripper_m)
                if status.follower_gripper_m is not None else float('nan'))
            self._status_publishers[side].publish(message)
            entry = DiagnosticStatus()
            entry.name = f'piper_two_can_teleop: {side}'
            entry.hardware_id = self._sides[side].session.config.port
            entry.level = (DiagnosticStatus.OK if status.fault is None
                           else DiagnosticStatus.ERROR)
            entry.message = (status.label if status.fault is None
                             else f'{status.label}：{status.fault}')
            entry.values = [
                KeyValue(key='state', value=status.state.value),
                KeyValue(key='command_hz', value=f'{status.command_hz:.1f}'),
                KeyValue(key='master_hz', value=f'{status.master_hz:.1f}'),
                KeyValue(key='follower_hz',
                         value=f'{status.follower_hz:.1f}'),
                KeyValue(key='can_error_frames',
                         value=str(status.error_frames)),
                KeyValue(key='can_error_rate_hz',
                         value=f'{status.error_rate_hz:.2f}'),
                KeyValue(key='tracking_error_deg',
                         value=f'{status.tracking_error_deg:.3f}'),
            ]
            diagnostics.status.append(entry)
        self._diagnostics.publish(diagnostics)

    # ---- 生命周期 -----------------------------------------------------

    # ---- 服务 ---------------------------------------------------------

    def _side_ready(self, side: str) -> Optional[str]:
        """Return None when this side may start, else the reason it may not."""
        if not self._send:
            return '干跑模式（没有 --send）：拒绝使能与运动'
        if not self._sides[side].alive:
            return f'{side} 会话已退出，请重新启动管理器'
        state = self._sides[side].state
        if state is TeleopState.OFFLINE:
            return f'{side} 还没有通过只读门禁（当前 {state.value}）'
        if state is not TeleopState.IDLE_DISABLED:
            return f'{side} 当前状态是 {state.value}，不能再次启动'
        return None

    def _trigger(self, ok: bool, message: str) -> Trigger.Response:
        response = Trigger.Response()
        response.success = ok
        response.message = message
        if not ok:
            self._log(f'拒绝服务请求：{message}')
        return response

    def _make_side_start(self, side: str):
        def handler(_request, response):
            reason = self._side_ready(side)
            if reason:
                return self._trigger(False, reason)
            self._sides[side].operator.request_start()
            return self._trigger(
                True, f'{side} 已提交启动请求；'
                      '请在主臂进入实体示教后调用 teach_engaged')
        return handler

    def _make_side_stop(self, side: str):
        def handler(_request, response):
            self._sides[side].operator.request_stop(f'{side} stop 服务')
            return self._trigger(
                True, f'{side} 已请求停止目标流；'
                      '关闭实体示教按钮后调用 teach_released')
        return handler

    def _make_side_teach(self, side: str, *, engaged: bool):
        def handler(_request, response):
            operator = self._sides[side].operator
            state = self._sides[side].state
            if engaged:
                if state is not TeleopState.WAIT_TEACH:
                    return self._trigger(
                        False, f'{side} 当前状态是 {state.value}，'
                               '只有 WAIT_TEACH 才接受 teach_engaged 确认')
                operator.confirm_teach_engaged()
                return self._trigger(
                    True, f'{side} 已确认主臂进入实体示教，开始受限对齐')
            if state is not TeleopState.WAIT_TEACH_RELEASE:
                return self._trigger(
                    False, f'{side} 当前状态是 {state.value}，'
                           '只有 WAIT_TEACH_RELEASE 才接受 teach_released 确认')
            operator.confirm_teach_released()
            return self._trigger(
                True, f'{side} 已确认实体示教退出，开始整侧失能')
        return handler

    def _dual_start(self, _request, response):
        reasons = [reason for reason in
                   (self._side_ready(side) for side in SIDES) if reason]
        if reasons:
            return self._trigger(False, '；'.join(reasons))
        self._barrier = ActiveBarrier(
            len(SIDES), timeout_s=self._barrier_timeout_s)
        for side in SIDES:
            runner = self._sides[side]
            runner.session.enter_active_hook = (
                lambda session=runner.session, barrier=self._barrier:
                barrier.wait(session.wait_for_peer))
            runner.operator.request_start()
        return self._trigger(
            True, '两侧已提交启动请求；请在两侧主臂都进入实体示教后分别调用 '
                  'teach_engaged，两侧对齐完成后会一起进入跟随')

    def _dual_stop(self, _request, response):
        for side in SIDES:
            self._sides[side].operator.request_stop('双侧 stop 服务')
        return self._trigger(
            True, '两侧已请求停止目标流；请分别关闭实体示教按钮后调用各自的 '
                  'teach_released 完成失能')

    # ---- 退出 ---------------------------------------------------------

    def shutdown_sessions(self) -> int:
        """Stop both sides and return the worst session exit code."""
        for side in SIDES:
            runner = self._sides[side]
            runner.operator.request_stop('管理器退出')
            runner.session.skip_release_wait(
                '管理器退出：只停目标流，不失能，等待人工关闭实体示教')
        codes = {}
        for side in SIDES:
            runner = self._sides[side]
            if not runner.join(SHUTDOWN_GRACE_S):
                self._log(f'!! {side} 的会话线程在 {SHUTDOWN_GRACE_S:g}s 内'
                          '没有退出：机械臂状态未知，请人工确认使能状态')
            codes[side] = runner.exit_code
            self._log(f'{side} 最终状态 {runner.state.value}'
                      f'（退出码 {runner.exit_code}）')
        if any(code == EXIT_FAILED for code in codes.values()):
            self._log('至少一侧以故障停止结束；请按日志确认使能状态并人工失能。')
            return EXIT_FAILED
        if any(code == EXIT_PENDING_RELEASE for code in codes.values()):
            self._log('至少一侧没有完成失能（也没有得到实体示教退出确认）。'
                      '请关闭实体示教后运行 '
                      'ros2 run piper piper_arm_enable --disable --send。')
            return EXIT_PENDING_RELEASE
        if any(code != EXIT_OK for code in codes.values()):
            self._log('至少一侧没有进入过摇操（只读门禁未通过或请求被拒绝）。')
            return EXIT_REFUSED
        return EXIT_OK


def _application_args(args):
    """Strip ROS arguments so argparse only sees this node's options."""
    application_args = remove_ros_args(args)
    # With ``args=None`` rclpy reads sys.argv and keeps argv[0] in the
    # returned non-ROS list. argparse expects only argv[1:].
    if args is None:
        application_args = application_args[1:]
    return application_args


def manager_parser() -> ArgumentParser:
    """Build the manager's option set on top of the shared teleop options."""
    parser = ArgumentParser(
        description=__doc__, parents=[argument_parser(add_help=False)])
    parser.add_argument('--port-left', default=None,
                        help='左侧 SocketCAN 接口（默认 can_left）')
    parser.add_argument('--port-right', default=None,
                        help='右侧 SocketCAN 接口（默认 can_right）')
    parser.add_argument('--barrier-timeout', type=float,
                        default=DEFAULT_DUAL_BARRIER_TIMEOUT_S,
                        help='双侧同步启动时等待另一侧对齐的秒数'
                             '（默认 %(default)s）')
    return parser


def main(args=None) -> int:
    """Run the manager node until it is shut down."""
    install_sigterm_handler()
    options = manager_parser().parse_args(_application_args(args))
    if options.send and not options.workspace_clear:
        print('错误：--send 必须同时给出 --workspace-clear')
        return EXIT_REFUSED
    configs = {}
    for side, port in (('left', options.port_left),
                       ('right', options.port_right)):
        options.side = side
        options.port = port
        configs[side] = build_config(options)

    rclpy.init(args=None)
    manager = TeleopManager(
        configs, send=bool(options.send),
        barrier_timeout_s=options.barrier_timeout)
    code = EXIT_OK
    try:
        rclpy.spin(manager)
    except KeyboardInterrupt:
        print('收到 Ctrl-C 或 SIGTERM：正在停止两侧目标流…')
    finally:
        code = manager.shutdown_sessions()
        manager.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == '__main__':
    raise SystemExit(main())
