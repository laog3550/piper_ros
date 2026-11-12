"""分侧启动直连 CAN 只读采集旁路，避免两条高频总线共用 Python 解释器。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    profile = os.path.join(get_package_share_directory('piper'), 'config', 'direct_can_fastdds.xml')
    can_environment = {'FASTRTPS_DEFAULT_PROFILES_FILE': profile,
                       'FASTDDS_DEFAULT_PROFILES_FILE': profile,
                       'RMW_FASTRTPS_USE_QOS_FROM_XML': '1',
                       'RMW_FASTRTPS_PUBLICATION_MODE': 'ASYNCHRONOUS'}
    nodes = [
        DeclareLaunchArgument('left_gripper_mapping_verified', default_value='false'),
        DeclareLaunchArgument('right_gripper_mapping_verified', default_value='false'),
        DeclareLaunchArgument('receive_queue_depth', default_value='4096'),
        DeclareLaunchArgument('publisher_qos_depth', default_value='1000'),
        DeclareLaunchArgument('vector_input_qos_depth', default_value='32'),
    ]
    for side in ('left', 'right'):
        nodes.append(Node(
            package='piper', executable='piper_direct_can',
            name=f'piper_direct_can_{side}', output='screen',
            additional_env=can_environment,
            parameters=[{'side': side,
                         'receive_queue_depth': ParameterValue(LaunchConfiguration('receive_queue_depth'), value_type=int),
                         'publisher_qos_depth': ParameterValue(LaunchConfiguration('publisher_qos_depth'), value_type=int)}],
        ))
        nodes.append(Node(
            package='piper', executable='piper_arm_vector',
            name=f'piper_arm_vector_{side}', output='screen',
            parameters=[{'side': side, 'gripper_mapping_verified': ParameterValue(
                LaunchConfiguration(f'{side}_gripper_mapping_verified'), value_type=bool),
                         'input_qos_depth': ParameterValue(LaunchConfiguration('vector_input_qos_depth'), value_type=int)}],
        ))
    return LaunchDescription(nodes)
