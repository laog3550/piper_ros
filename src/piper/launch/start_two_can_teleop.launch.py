"""Launch read-only monitors for both official FA/FC teleoperation buses."""

import json
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import LogInfo, OpaqueFunction
from launch_ros.actions import Node


def _mapping_path() -> Path:
    configured = os.environ.get('PIPER_PI05_CAN_CONFIG')
    return (Path(configured).expanduser() if configured
            else Path.home() / 'piper_ros' / 'config' / 'pi05_can_map.json')


def _monitors(_context):
    path = _mapping_path()
    try:
        data = json.loads(path.read_text(encoding='utf-8'))
        interfaces = {side: str(data[side]['interface'])
                      for side in ('left', 'right')}
    except (OSError, KeyError, TypeError, json.JSONDecodeError) as exc:
        raise RuntimeError(f'读不出 CAN 映射 {path}：{exc}') from exc
    if interfaces['left'] == interfaces['right']:
        raise RuntimeError('左右两侧不能共用同一个 CAN 接口')
    return [Node(package='piper', executable='piper_two_can_teleop',
                 name=f'piper_native_teleop_{side}', output='screen',
                 arguments=['--side', side, '--port', port])
            for side, port in interfaces.items()]


def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg='启动两侧官方固件联动的只读监视；按设备实体流程停臂，'
                    '关闭监视进程不会停臂或失能。'),
        OpaqueFunction(function=_monitors),
    ])
