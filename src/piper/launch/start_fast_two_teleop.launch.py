"""启动左右两路由单一会话管理使能状态的快速遥操作。"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """每侧只启动一个遥操作会话，由它完成使能和退出失能。"""
    teleop_left = Node(
        package='piper',
        executable='piper_teleop_fast',
        output='screen',
        arguments=[
            '--side', 'left', '--enable', '--manage-enable',
            '--disable-on-exit',
        ],
    )
    teleop_right = Node(
        package='piper',
        executable='piper_teleop_fast',
        output='screen',
        arguments=[
            '--side', 'right', '--enable', '--manage-enable',
            '--disable-on-exit',
        ],
    )
    return LaunchDescription([
        teleop_left,
        teleop_right,
    ])
