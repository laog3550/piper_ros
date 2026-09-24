"""启动左右两台 master，并为每台 master 隔离 ROS 话题。"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"


def _master_node(side, can_port, log_level, auto_enable, gripper_exist,
                 gripper_val_mutiple):
    """创建一台 master 节点及其侧别专用的 ROS 接口。"""
    return Node(
        package='piper',
        executable='piper_single_ctrl',
        name=f'piper_master_{side}_ctrl_node',
        output='screen',
        ros_arguments=['--log-level', log_level],
        parameters=[{
            'can_port': can_port,
            'auto_enable': auto_enable,
            'gripper_exist': gripper_exist,
            'gripper_val_mutiple': gripper_val_mutiple,
        }],
        remappings=[
            # 控制输入。master 默认失能，因此这些话题不应有发布者。
            ('pos_cmd', f'/pos_cmd_master_{side}'),
            ('joint_ctrl_single', f'/joint_ctrl_master_{side}'),
            ('enable_flag', f'/enable_flag_master_{side}'),
            ('enable_srv', f'/enable_srv_master_{side}'),
            # 反馈输出。
            ('joint_states_single', f'/joint_states_master_{side}'),
            ('joint_states_feedback', f'/joint_states_master_feedback_{side}'),
            ('joint_ctrl', f'/joint_states_master_ctrl_{side}'),
            ('arm_status', f'/arm_status_master_{side}'),
            ('arm_enable_status', f'/arm_enable_status_master_{side}'),
            ('end_pose', f'/end_pose_master_{side}'),
            ('end_pose_stamped', f'/end_pose_stamped_master_{side}'),
        ],
    )


def generate_launch_description():
    """返回左右 master 的 launch 描述。"""
    log_level = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='日志级别（debug、info、warn、error、fatal）。',
    )
    can_left_port = DeclareLaunchArgument(
        'can_left_port',
        default_value='can_ml',
        description='左 master 的 CAN 接口。',
    )
    can_right_port = DeclareLaunchArgument(
        'can_right_port',
        default_value='can_mr',
        description='右 master 的 CAN 接口。',
    )
    auto_enable = DeclareLaunchArgument(
        'auto_enable',
        default_value='false',
        description='是否自动使能 master；默认关闭以便手动拖动。',
    )
    gripper_exist = DeclareLaunchArgument(
        'gripper_exist',
        default_value='false',
        description='是否允许 master 节点发送夹爪控制指令。',
    )
    gripper_val_mutiple = DeclareLaunchArgument(
        'gripper_val_mutiple',
        default_value='1',
        description='夹爪控制倍数。',
    )

    common = (
        LaunchConfiguration('log_level'),
        LaunchConfiguration('auto_enable'),
        LaunchConfiguration('gripper_exist'),
        LaunchConfiguration('gripper_val_mutiple'),
    )
    master_left = _master_node(
        'left', LaunchConfiguration('can_left_port'), *common)
    master_right = _master_node(
        'right', LaunchConfiguration('can_right_port'), *common)

    return LaunchDescription([
        log_level,
        can_left_port,
        can_right_port,
        auto_enable,
        gripper_exist,
        gripper_val_mutiple,
        master_left,
        master_right,
    ])
