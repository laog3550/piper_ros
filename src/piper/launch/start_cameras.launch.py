"""按序列号启动三路 RGB-D 相机；运行：ros2 launch piper start_cameras.launch.py."""

import json
import math
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
    depth_text = LaunchConfiguration('enable_depth').perform(context).strip().lower()
    if depth_text not in ('true', 'false'):
        raise RuntimeError('enable_depth 只能为 true 或 false')
    enable_depth = depth_text == 'true'
    top_lock_text = LaunchConfiguration('top_fps_lock').perform(context).strip().lower()
    if top_lock_text not in ('true', 'false'):
        raise RuntimeError('top_fps_lock 只能为 true 或 false')
    top_fps_lock = top_lock_text == 'true'
    try:
        d455_exposure = int(LaunchConfiguration('d455_exposure').perform(context))
        d455_gain = int(LaunchConfiguration('d455_gain').perform(context))
        orbbec_color_exposure = int(
            LaunchConfiguration('orbbec_color_exposure').perform(context))
        orbbec_color_gain = int(LaunchConfiguration('orbbec_color_gain').perform(context))
    except ValueError as exc:
        raise RuntimeError('曝光和增益参数必须为整数') from exc
    try:
        d455_white_balance = float(
            LaunchConfiguration('d455_white_balance').perform(context))
        orbbec_color_white_balance = int(
            LaunchConfiguration('orbbec_color_white_balance').perform(context))
    except ValueError as exc:
        raise RuntimeError('白平衡参数必须为数值') from exc
    if (not 1 <= d455_exposure <= 10000 or not 0 <= d455_gain <= 128
            or orbbec_color_exposure < 0 or orbbec_color_gain < 0
            or not 1000 <= d455_white_balance <= 10000
            or not 1000 <= orbbec_color_white_balance <= 10000):
        raise RuntimeError('曝光、增益或白平衡超出允许范围')
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
    sync_profile = LaunchConfiguration('camera_sync').perform(context)
    if sync_profile not in ('standalone', 'd455_master', 'external_trigger'):
        raise RuntimeError('camera_sync 只能为 standalone、d455_master 或 external_trigger')
    try:
        sync_delay = float(LaunchConfiguration('camera_sync_delay_sec').perform(context))
    except ValueError as exc:
        raise RuntimeError('camera_sync_delay_sec 必须为数值') from exc
    if not math.isfinite(sync_delay) or sync_delay < 0:
        raise RuntimeError('camera_sync_delay_sec 必须为非负有限数值')
    try:
        orbbec_sync_voltage = int(
            LaunchConfiguration('orbbec_sync_io_voltage_level').perform(context))
    except ValueError as exc:
        raise RuntimeError('orbbec_sync_io_voltage_level 必须为整数') from exc
    if sync_profile in ('d455_master', 'external_trigger') and set(selected) != set(ROLES):
        raise RuntimeError('camera_sync 的硬件同步方案需要同时启动三路相机')
    actions = []
    nodes = []
    if mapping.get('position_verified') is not True:
        actions.append(LogInfo(
            msg='相机位置尚待现场核对，当前角色沿用旧配对记录。',
        ))
    if sync_profile == 'd455_master':
        actions.append(LogInfo(
            msg='d455_master：D455 sync out 必须接到两路 Orbbec 的 sync in；'
                'Orbbec 设为 secondary_synced 且 trigger_out_enabled=false。未接线时 Orbbec 可能不出流。',
        ))
    elif sync_profile == 'external_trigger':
        actions.append(LogInfo(
            msg='external_trigger：先启动外部 15 Hz 触发源；D455 设为 inter_cam_sync_mode=2（slave），'
                'Orbbec 设为 secondary_synced，所有 trigger_out_enabled=false。',
        ))
    if top_fps_lock and fps == 15 and 'top' in selected and sync_profile != 'external_trigger':
        actions.append(LogInfo(
            msg='顶部 D455 启用 15 Hz 帧同步主模式（深度传感器参与帧时钟）；'
                '腕部仍独立运行。固定曝光下该模式可修正实际 16.59 fps。',
        ))
    order = list(selected)
    if sync_profile == 'd455_master':
        order = ['top', *[role for role in ('left_wrist', 'right_wrist') if role in selected]]

    for index, role in enumerate(order):
        camera = mapping[role]
        if role == 'top':
            parameters = {
                'camera_namespace': camera['camera_name'],
                'camera_name': 'D455',
                'serial_no': '_' + camera['serial'],
                'rgb_camera.color_profile': f'640,480,{fps}',
                'depth_module.depth_profile': f'640,480,{fps}',
                'enable_color': True,
                'enable_depth': enable_depth,
                'align_depth.enable': enable_depth,
                'enable_sync': True,
                'enable_infra1': False,
                'enable_infra2': False,
                'enable_infra': False,
                'enable_gyro': False,
                'enable_accel': False,
                'enable_motion': False,
                'pointcloud.enable': False,
                'rgb_camera.enable_auto_exposure': False,
                'rgb_camera.exposure': d455_exposure,
                'rgb_camera.gain': d455_gain,
                'rgb_camera.enable_auto_white_balance': False,
                'rgb_camera.white_balance': d455_white_balance,
            }
            if top_fps_lock and fps == 15:
                # 固定曝光时 D455 的 640x480@15 档位实际会跑 16.59 fps。
                # 启用深度帧同步主模式后由深度帧时钟锁定到 15.0 fps；
                # 深度流仅用于帧时钟，不启用 align_depth，SFT 录制仍只录彩色。
                parameters.update({
                    'enable_depth': True,
                    'align_depth.enable': False,
                    'depth_module.inter_cam_sync_mode': 1,
                    'depth_module.output_trigger_enabled': True,
                    'depth_module.global_time_enabled': True,
                    'rgb_camera.global_time_enabled': True,
                })
            if sync_profile in ('d455_master', 'external_trigger'):
                parameters.update({
                    'depth_module.inter_cam_sync_mode': (1 if sync_profile == 'd455_master' else 2),
                    'depth_module.output_trigger_enabled': sync_profile == 'd455_master',
                    'depth_module.global_time_enabled': True,
                    'rgb_camera.global_time_enabled': True,
                })
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
                'enable_depth': enable_depth,
                'depth_registration': enable_depth,
                'enable_left_ir': False,
                'enable_right_ir': False,
                'enable_frame_sync': True,
                'enable_point_cloud': False,
                'enable_color_auto_exposure': False,
                'color_exposure': orbbec_color_exposure,
                'color_gain': orbbec_color_gain,
                'enable_color_auto_white_balance': False,
                'color_white_balance': orbbec_color_white_balance,
                'color.image_raw.enable_pub_plugins': ['image_transport/raw'],
                'depth.image_raw.enable_pub_plugins': ['image_transport/raw'],
            }
            if sync_profile != 'standalone':
                parameters.update({
                    'sync_mode': 'secondary_synced',
                    'trigger_out_enabled': False,
                    'frames_per_trigger': 1,
                })
                if orbbec_sync_voltage >= 0:
                    parameters['sync_io_voltage_level'] = orbbec_sync_voltage
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
        delay = sync_delay * index if sync_profile == 'd455_master' else 4.0 * index
        actions.append(TimerAction(period=float(delay), actions=[group]))
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
            description='三路相机帧率；默认 15 fps（D455 无 10 fps 档位）；30 fps 深度+彩色会使 Orbbec 发布塌陷，需显式启用并验证负载。',
        ),
        DeclareLaunchArgument(
            'enable_depth', default_value='true',
            description='是否启用深度流；SFT/录制只使用彩色时设为 false 可降低负载。顶部 15 fps 锁定时会单独启用顶部深度，不受此参数影响。',
        ),
        DeclareLaunchArgument(
            'top_fps_lock', default_value='true',
            description='顶部 D455 在 15 fps 下启用深度帧同步主模式，修正固定曝光时实际 16.59 fps；会额外启用顶部深度流，腕部不受影响。',
        ),
        DeclareLaunchArgument(
            'd455_exposure', default_value='80',
            description='D455 顶部彩色固定曝光（µs，1～10000）；设置后关闭自动曝光。',
        ),
        DeclareLaunchArgument(
            'd455_gain', default_value='32',
            description='D455 顶部彩色固定增益（0～128）。',
        ),
        DeclareLaunchArgument(
            'orbbec_color_exposure', default_value='100',
            description='Orbbec 腕部彩色固定曝光；设置后关闭自动曝光，按现场亮度调整。',
        ),
        DeclareLaunchArgument(
            'orbbec_color_gain', default_value='0',
            description='Orbbec 腕部彩色固定增益，按现场亮度调整。',
        ),
        DeclareLaunchArgument(
            'd455_white_balance', default_value='4000',
            description='D455 顶部固定白平衡色温（K）；设置后关闭自动白平衡。',
        ),
        DeclareLaunchArgument(
            'orbbec_color_white_balance', default_value='4600',
            description='Orbbec 腕部固定白平衡色温（K）；设置后关闭自动白平衡。',
        ),
        DeclareLaunchArgument(
            'camera_sync', default_value='standalone',
            description='standalone、d455_master（D455 主）或 external_trigger（外部/无线触发，三路都做从机）。',
        ),
        DeclareLaunchArgument(
            'camera_sync_delay_sec', default_value='5.0',
            description='d455_master 模式下 D455 主相机与从相机的启动间隔（秒）。',
        ),
        DeclareLaunchArgument(
            'orbbec_sync_io_voltage_level', default_value='-1',
            description='d455_master 模式下 Orbbec sync IO 电平；-1 使用驱动默认，按相机手册设置。',
        ),
        OpaqueFunction(function=_start_cameras),
    ])
