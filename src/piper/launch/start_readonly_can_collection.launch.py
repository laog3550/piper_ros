"""分侧启动直连 CAN 只读采集旁路，避免两条高频总线共用 Python 解释器。"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    nodes = []
    for side in ('left', 'right'):
        nodes.append(Node(
            package='piper', executable='piper_direct_can',
            name=f'piper_direct_can_{side}', output='screen',
            parameters=[{'side': side}],
        ))
        nodes.append(Node(
            package='piper', executable='piper_arm_vector',
            name=f'piper_arm_vector_{side}', output='screen',
            parameters=[{'side': side, 'gripper_mapping_verified': False}],
        ))
    return LaunchDescription(nodes)
