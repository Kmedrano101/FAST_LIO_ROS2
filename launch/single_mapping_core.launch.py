#!/usr/bin/env python3

import os.path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    # ==========================================================================
    # PATH CONFIGURATION
    # ==========================================================================
    package_path = get_package_share_directory('fast_lio_ros2')
    default_config_path = os.path.join(package_path, 'config')

    # ==========================================================================
    # LAUNCH ARGUMENTS
    # ==========================================================================
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')
    config_file = LaunchConfiguration('config_file')


    # Argument: use_sim_time
    # Set to 'false' for real hardware, 'true' for Gazebo simulation
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',  # Real hardware by default
        description='Use simulation (Gazebo) clock if true, real-time if false'
    )

    # Argument: config_path
    # Directory containing YAML config files
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path',
        default_value=default_config_path,
        description='Directory path for YAML configuration files'
    )

    # Argument: config_file
    # YAML configuration filename (must exist in config_path)
    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file',
        default_value='single_L1.yaml',  # SDK fusion config
        description='YAML config filename for SDK fusion dual MID-360 setup'
    )

    fast_lio_node = Node(
        package='fast_lio_ros2',
        executable='fastlio_mapping',
        name='fastlio_single_mapping',
        output='screen',
        parameters=[
            PathJoinSubstitution([config_path, config_file]),
            {'use_sim_time': use_sim_time}
        ],
        # Increase output buffer to prevent log truncation
        arguments=['--ros-args', '--log-level', 'info'],
    )
    # ==========================================================================
    # LAUNCH DESCRIPTION ASSEMBLY
    # ==========================================================================
    ld = LaunchDescription()

    # Add all argument declarations
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(declare_config_file_cmd)

    # Add informational log message
    ld.add_action(LogInfo(
        msg=[
            '\n',
            '═══════════════════════════════════════════════════════════════\n',
            ' FAST-LIO Single Mode - Underground Mine Mapping\n',
            '═══════════════════════════════════════════════════════════════\n',
            ' Config: ', config_file, '\n',
            ' Sim Time: ', use_sim_time, '\n',
            '───────────────────────────────────────────────────────────────\n',
            ' Expected Topics (SDK Fusion):\n',
            '   → /livox/lidar \n',
            '   → /livox/imu \n',
            '───────────────────────────────────────────────────────────────\n',
            ' Output Topics:\n',
            '   ← /Odometry (Pose + Velocity)\n',
            '   ← /path (Trajectory)\n',
            '   ← /cloud_registered (Global Map)\n',
            '   ← /cloud_registered_body (Current Scan)\n',
            '───────────────────────────────────────────────────────────────\n',
            ' NOTE: Requires livox_ros_driver2 with multi_topic: 0\n',
            '═══════════════════════════════════════════════════════════════\n'
        ]
    ))

    # Add nodes
    ld.add_action(fast_lio_node)

    return ld