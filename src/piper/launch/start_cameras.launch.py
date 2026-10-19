"""按序列号启动三路 RGB-D 相机；运行：ros2 launch piper start_cameras.launch.py."""

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
from launch_ros.actions import Node


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
            arguments = {
                'camera_namespace': camera['camera_name'],
                'camera_name': 'D455',
                'serial_no': '_' + camera['serial'],
                'rgb_camera.color_profile': '640,480,30',
                'depth_module.depth_profile': '640,480,30',
                'enable_color': 'true',
                'enable_depth': 'true',
                'align_depth.enable': 'true',
                'enable_sync': 'true',
                'enable_infra1': 'false',
                'enable_infra2': 'false',
                'pointcloud.enable': 'false',
            }
            launch_path = (Path(get_package_share_directory('realsense2_camera'))
                           / 'launch' / 'rs_launch.py')
            camera_action = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(launch_path)),
                launch_arguments=arguments.items(),
            )
        else:
            parameters = {
                'camera_name': camera['camera_name'],
                'serial_number': camera['serial'],
                'color_width': 640,
                'color_height': 480,
                'color_fps': 30,
                'color_format': 'YUYV',
                'depth_width': 640,
                'depth_height': 480,
                'depth_fps': 30,
                'depth_format': 'Y16',
                'enable_color': True,
                'enable_depth': True,
                'depth_registration': True,
                'enable_left_ir': False,
                'enable_right_ir': False,
                'enable_frame_sync': True,
                'enable_point_cloud': False,
                'enable_spatial_filter': True,
                'enable_temporal_filter': True,
                'enable_hole_filling_filter': True,
                'color.image_raw.enable_pub_plugins': ['image_transport/raw'],
                'depth.image_raw.enable_pub_plugins': ['image_transport/raw'],
            }
            camera_action = Node(
                package='orbbec_camera',
                executable='orbbec_camera_node',
                namespace=camera['camera_name'],
                name=camera['camera_name'],
                parameters=[parameters],
                # The terminal signals launch; launch then signals this process once.
                prefix='setsid',
                output='screen',
            )
        group = GroupAction(actions=[
            LogInfo(msg=f'启动 {role}：{camera["serial"]}，640x480 @ 30 fps RGB-D 流'),
            camera_action,
        ], scoped=True, forwarding=False)
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
