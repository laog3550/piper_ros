"""分侧启动直连 CAN 只读采集旁路，避免两条高频总线共用 Python 解释器。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    nodes = [
        DeclareLaunchArgument('left_gripper_mapping_verified', default_value='false'),
        DeclareLaunchArgument('right_gripper_mapping_verified', default_value='false'),
    ]
    for side in ('left', 'right'):
        nodes.append(Node(
            package='piper', executable='piper_direct_can',
            name=f'piper_direct_can_{side}', output='screen',
            parameters=[{'side': side}],
        ))
        nodes.append(Node(
            package='piper', executable='piper_arm_vector',
            name=f'piper_arm_vector_{side}', output='screen',
            parameters=[{'side': side, 'gripper_mapping_verified': ParameterValue(
                LaunchConfiguration(f'{side}_gripper_mapping_verified'), value_type=bool)}],
        ))
    return LaunchDescription(nodes)
