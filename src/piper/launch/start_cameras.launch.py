"""按序列号启动三路 RGB-D 相机；运行：ros2 launch piper start_cameras.launch.py."""

import json
import os
import signal
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, GroupAction, LogInfo,
    OpaqueFunction, RegisterEventHandler, TimerAction,
)
from launch.event_handlers import OnShutdown
from launch.events import matches_action
from launch.events.process import SignalProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


ROLES = ('left_wrist', 'right_wrist', 'top')


def _signal_isolated_cameras(event, context, nodes):
    if not event.due_to_sigint or context.noninteractive:
        return []
    return [EmitEvent(event=SignalProcess(
        signal_number=signal.SIGINT,
        process_matcher=matches_action(node),
    )) for node in nodes]


def _start_cameras(context):
    path = Path(LaunchConfiguration('camera_map').perform(context)).expanduser()
    fps_text = LaunchConfiguration('camera_fps').perform(context)
    try:
        fps = int(fps_text)
    except ValueError as exc:
        raise RuntimeError(f'camera_fps 必须为整数：{fps_text}') from exc
    if fps not in (10, 15, 30, 60):
        raise RuntimeError('camera_fps 只能为 10、15、30 或 60')
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
    if fps == 10 and 'top' in selected:
        raise RuntimeError('D455 在 640x480 下没有 10 fps 档位；请用 camera_fps:=15 或只启动腕部相机')
    actions = []
    nodes = []
    if mapping.get('position_verified') is not True:
        actions.append(LogInfo(
            msg='相机位置尚待现场核对，当前角色沿用旧配对记录。',
        ))

    for index, role in enumerate(selected):
        camera = mapping[role]
        if role == 'top':
            parameters = {
                'camera_namespace': camera['camera_name'],
                'camera_name': 'D455',
                'serial_no': '_' + camera['serial'],
                'rgb_camera.color_profile': f'640,480,{fps}',
                'depth_module.depth_profile': f'640,480,{fps}',
                'enable_color': True,
                'enable_depth': True,
                'align_depth.enable': True,
                'enable_sync': True,
                'enable_infra1': False,
                'enable_infra2': False,
                'enable_infra': False,
                'enable_gyro': False,
                'enable_accel': False,
                'enable_motion': False,
                'pointcloud.enable': False,
            }
            camera_action = Node(
                package='realsense2_camera',
                executable='realsense2_camera_node',
                namespace=camera['camera_name'],
                name='D455',
                parameters=[parameters],
                prefix='setsid',
                output='screen',
            )
        else:
            parameters = {
                'camera_name': camera['camera_name'],
                'serial_number': camera['serial'],
                'color_width': 640,
                'color_height': 480,
                'color_fps': fps,
                'color_format': 'YUYV',
                'depth_width': 640,
                'depth_height': 480,
                'depth_fps': fps,
                'depth_format': 'Y16',
                'enable_color': True,
                'enable_depth': True,
                'depth_registration': True,
                'enable_left_ir': False,
                'enable_right_ir': False,
                'enable_frame_sync': True,
                'enable_point_cloud': False,
                'color.image_raw.enable_pub_plugins': ['image_transport/raw'],
                'depth.image_raw.enable_pub_plugins': ['image_transport/raw'],
            }
            camera_action = Node(
                package='orbbec_camera',
                executable='orbbec_camera_node',
                namespace=camera['camera_name'],
                name=camera['camera_name'],
                parameters=[parameters],
                # Keep each camera outside launch's terminal signal group.
                prefix='setsid',
                # Orbbec can need longer than launch's 5 s default to stop.
                sigterm_timeout='15',
                output='screen',
            )
        nodes.append(camera_action)
        group = GroupAction(actions=[
            LogInfo(msg=f'启动 {role}：{camera["serial"]}，640x480 @ {fps} fps RGB-D 流'),
            camera_action,
        ], scoped=True, forwarding=False)
        actions.append(TimerAction(period=float(index * 4), actions=[group]))
    actions.insert(0, RegisterEventHandler(OnShutdown(
        on_shutdown=lambda event, ctx: _signal_isolated_cameras(event, ctx, nodes),
    )))
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
        DeclareLaunchArgument(
            'camera_fps', default_value='15',
            description='三路相机帧率；采集默认 15 fps（D455 无 10 fps 档位），训练转换仍按 10 fps 采样。',
        ),
        OpaqueFunction(function=_start_cameras),
    ])
