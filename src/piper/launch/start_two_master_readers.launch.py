"""Start both offset master readers; they only receive, never transmit."""

# 用途：只读解析主臂偏移后的 0x2Cx 反馈，发布 ROS 状态用于观察。
# 主臂保持 0xFC，反馈和控制偏移为 0x20；实体按钮负责示教进出。
# 本节点不属于实时控制环，不发送配置、使能或运动帧。
# 关节弧度使用 math.pi / 180，夹爪单位为 m。

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


def _readers(context):
    """Build one offset master reader per side."""
    offset = LaunchConfiguration('offset').perform(context)
    interfaces = _interfaces()
    return [
        Node(
            package='piper',
            executable='piper_master_state',
            name=f'piper_master_state_{side}',
            output='screen',
            arguments=[
                '--port', interfaces[side],
                '--offset', offset,
                '--side', side,
            ],
        )
        for side in SIDES
    ]


def generate_launch_description():
    """Return the launch description for both master readers."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'offset',
            default_value='0x20',
            description='主臂反馈 ID 偏移，必须与 0x470 设置的一致（0x00/0x10/0x20）。',
        ),
        LogInfo(msg=(
            '只读读取：两个节点只收不发，不使能、不失能、不下发运动指令。'
            '主臂必须已离线配置为 0xFC + 0x20 偏移（且偏移量与 --offset 一致），'
            '否则这两个话题上没有数据。'
        )),
        OpaqueFunction(function=_readers),
    ])
