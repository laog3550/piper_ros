"""启动左右两路不限时遥操作及独立的 master 双击快速复位。"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    """先分别使能两台 follower，再启动两路快速遥操作。"""
    enable_left = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call', '/enable_srv_left',
            'piper_msgs/srv/Enable', '{enable_request: true}',
        ],
        output='screen',
    )
    enable_right = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call', '/enable_srv_right',
            'piper_msgs/srv/Enable', '{enable_request: true}',
        ],
        output='screen',
    )
    teleop_left = Node(
        package='piper',
        executable='piper_teleop_fast',
        output='screen',
        arguments=['--side', 'left', '--enable'],
    )
    teleop_right = Node(
        package='piper',
        executable='piper_teleop_fast',
        output='screen',
        arguments=['--side', 'right', '--enable'],
    )
    return LaunchDescription([
        RegisterEventHandler(
            OnProcessExit(
                target_action=enable_left,
                on_exit=[teleop_left],
            ),
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=enable_right,
                on_exit=[teleop_right],
            ),
        ),
        enable_left,
        enable_right,
    ])
