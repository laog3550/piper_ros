"""Launch the two-CAN teleop manager for both sides."""

# 用途：起 piper_two_can_manager，它在两条总线上各跑一个直接 SocketCAN 的实时跟随环，
# 并提供 /teleop/* 启停服务与 /teleop/state、/diagnostics 状态发布。
#
# 默认干跑（send:=false）：两侧只执行只读门禁并打印对齐计划，start 服务会拒绝使能与
# 运动。真正运动必须显式 send:=true，表示操作员已确认人员和线缆退出运动空间。
#
# 接口名不写死在这里，而是从机器本地的 config/pi05_can_map.json 读——它是接口身份的
# 唯一权威来源（可用 PIPER_PI05_CAN_CONFIG 指定别的路径）。

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import json
import os
from pathlib import Path

os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"

SIDES = ('left', 'right')


def _mapping_path() -> Path:
    """Locate the machine-local Pi05 bus mapping."""
    configured = os.environ.get('PIPER_PI05_CAN_CONFIG')
    if configured:
        return Path(configured).expanduser()
    return Path.home() / 'piper_ros' / 'config' / 'pi05_can_map.json'


def _interfaces() -> dict:
    """Return the interface of each side, or fail with a usable message."""
    path = _mapping_path()
    try:
        data = json.loads(path.read_text(encoding='utf-8'))
        return {side: str(data[side]['interface']) for side in SIDES}
    except (OSError, KeyError, TypeError, json.JSONDecodeError) as exc:
        raise RuntimeError(
            f'读不出 CAN 映射 {path}：{exc}；'
            f'先按 docs/PI05_CAN_MAPPING.md 把它写对'
        ) from exc


def _manager(context):
    """Build the manager node with the local interface names."""
    interfaces = _interfaces()
    send = LaunchConfiguration('send').perform(context).lower()
    arguments = [
        '--port-left', interfaces['left'],
        '--port-right', interfaces['right'],
    ]
    if send in ('true', '1', 'yes'):
        arguments.extend(['--send', '--workspace-clear'])
    return [
        Node(
            package='piper',
            executable='piper_two_can_manager',
            name='piper_two_can_manager',
            output='screen',
            arguments=arguments,
        )
    ]


def generate_launch_description():
    """Return the launch description for the two-CAN teleop manager."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'send',
            default_value='false',
            description='是否真正发送使能与运动帧；默认 false 只执行只读门禁。',
        ),
        LogInfo(msg=(
            '两 CAN 摇操管理器：默认干跑。加 send:=true 以前必须确认人员和线缆'
            '已退出运动空间。启动后用 /teleop/start 或 /teleop/<side>/start 请求'
            '进入，用 teach_engaged / teach_released 完成操作员确认。'
        )),
        OpaqueFunction(function=_manager),
    ])
