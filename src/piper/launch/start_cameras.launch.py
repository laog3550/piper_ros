"""按机器本地序列号映射启动三路彩色相机。"""

import json
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
    LogInfo, OpaqueFunction, TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


ROLES = ('left_wrist', 'right_wrist', 'top')


def _start_cameras(context):
    path = Path(LaunchConfiguration('camera_map').perform(context)).expanduser()
    try:
        mapping = json.loads(path.read_text(encoding='utf-8'))
        serials = [mapping[role]['serial'] for role in ROLES]
        names = [mapping[role]['camera_name'] for role in ROLES]
        if not all(isinstance(value, str) and value.strip()
                   for value in serials + names):
            raise ValueError('serial 和 camera_name 必须为非空字符串')
        if len(set(serials)) != 3 or len(set(names)) != 3:
            raise ValueError('三台相机的 serial 和 camera_name 均不能重复')
    except (OSError, KeyError, TypeError, ValueError) as exc:
        raise RuntimeError(f'无法读取相机映射 {path}：{exc}') from exc

    selection = LaunchConfiguration('camera').perform(context)
    if selection not in ('all',) + ROLES:
        raise RuntimeError('camera 必须为 all、left_wrist、right_wrist 或 top')
    selected = ROLES if selection == 'all' else (selection,)
    actions = []
    if mapping.get('position_verified') is not True:
        actions.append(LogInfo(msg='相机位置尚待现场核对，当前角色沿用旧配对记录。'))

    for index, role in enumerate(selected):
        camera = mapping[role]
        if role == 'top':
            package, filename = 'realsense2_camera', 'rs_launch.py'
            arguments = {
                'camera_namespace': camera['camera_name'],
                'camera_name': 'D455',
                'serial_no': '_' + camera['serial'],
                'rgb_camera.color_profile': '640,480,30',
                'enable_color': 'true',
                'enable_depth': 'false',
                'enable_infra1': 'false',
                'enable_infra2': 'false',
                'pointcloud.enable': 'false',
            }
        else:
            package, filename = 'orbbec_camera', 'gemini_301_series.launch.py'
            arguments = {
                'camera_name': camera['camera_name'],
                'serial_number': camera['serial'],
                'color_width': '640',
                'color_height': '480',
                'color_fps': '30',
                'enable_color': 'true',
                'enable_depth': 'false',
                'enable_left_ir': 'false',
                'enable_right_ir': 'false',
                'enable_frame_sync': 'false',
                'enable_point_cloud': 'false',
                'color.image_raw.enable_pub_plugins': '["image_transport/raw"]',
            }
        launch_path = Path(get_package_share_directory(package)) / 'launch' / filename
        # Each vendor launch must have an isolated argument scope.
        group = GroupAction(actions=[
            LogInfo(msg=f'启动 {role}：{camera["serial"]}，640x480 @ 30 fps 彩色流'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(launch_path)),
                launch_arguments=arguments.items(),
            ),
        ], scoped=True)
        actions.append(TimerAction(period=float(index * 4), actions=[group]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_map',
            default_value=os.environ.get(
                'PIPER_CAMERA_CONFIG',
                str(Path.home() / 'piper_ros' / 'config' / 'camera_map.json'),
            ),
            description='本机相机序列号映射 JSON 路径。',
        ),
        DeclareLaunchArgument(
            'camera', default_value='all',
            description='启动 all、left_wrist、right_wrist 或 top。',
        ),
        OpaqueFunction(function=_start_cameras),
    ])
