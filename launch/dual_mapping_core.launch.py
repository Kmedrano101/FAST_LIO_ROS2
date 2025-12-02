#!/usr/bin/env python3
###############################################################################
# FAST-LIO Dual LiDAR Core Launch File
# Target: Core mapping computation for dual Livox MID-360 setup
# Hardware: NVIDIA Jetson ORIN + 2x MID-360
# ROS2: Humble/Iron
#
# Usage:
#   ros2 launch fast_lio_ros2 dual_mapping_core.launch.py
#   ros2 launch fast_lio_ros2 dual_mapping_core.launch.py config_file:=my_config.yaml
#   ros2 launch fast_lio_ros2 dual_mapping_core.launch.py update_method:=0  # BUNDLE mode
#   ros2 launch fast_lio_ros2 dual_mapping_core.launch.py update_method:=1  # ASYNC mode
#   ros2 launch fast_lio_ros2 dual_mapping_core.launch.py update_method:=2  # ADAPTIVE mode
#
# This launch file only runs the FAST-LIO core computation node.
# For visualization, use dual_mapping_desktop.launch.py on a separate machine.
###############################################################################

import os.path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    """
    Generate launch description for dual LiDAR core mapping only
    No visualization - runs on embedded system (Jetson)
    """
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
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # Argument: config_path
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path',
        default_value=default_config_path,
        description='Directory containing config files'
    )

    # Argument: config_file
    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file',
        default_value='dual_mid360.yaml',
        description='YAML configuration file name (must exist in config_path)'
    )

    # ==========================================================================
    # FAST-LIO CORE NODE
    # ==========================================================================
    # Main SLAM computation node
    # Subscribes to:
    #   - /livox/lidar_192_168_1_10 (LiDAR 1 point cloud)
    #   - /livox/lidar_192_168_1_18 (LiDAR 2 point cloud)
    #   - /livox/imu_transformed (IMU data)
    # Publishes:
    #   - /Odometry (6-DOF pose and velocity)
    #   - /path (Trajectory)
    #   - /cloud_registered (Global map)
    #   - /cloud_registered_body (Current scan in body frame)
    #   - /lidar1_colored (LiDAR 1 visualization - GREEN)
    #   - /lidar2_colored (LiDAR 2 visualization - RED)
    # ==========================================================================
    fast_lio_node = Node(
        package='fast_lio_ros2',
        executable='fastlio_mapping',
        name='fastlio_mapping',
        output='screen',
        parameters=[
            PathJoinSubstitution([config_path, config_file]),
            {'use_sim_time': use_sim_time}
        ],
        # Increase priority for real-time performance
        prefix='nice -n -10'
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
            ' FAST-LIO Dual LiDAR Core (No Visualization)\n',
            '═══════════════════════════════════════════════════════════════\n',
            ' Config: ', config_file, '\n',
            ' Sim Time: ', use_sim_time, '\n',
            '───────────────────────────────────────────────────────────────\n',
            ' Subscribed Topics:\n',
            '   → /livox/lidar_192_168_1_10 (LiDAR 1 - Front MID-360)\n',
            '   → /livox/lidar_192_168_1_18 (LiDAR 2 - Rear MID-360)\n',
            '   → /livox/imu_transformed (IMU)\n',
            '───────────────────────────────────────────────────────────────\n',
            ' Published Topics:\n',
            '   → /Odometry (Pose + Velocity)\n',
            '   → /path (Trajectory)\n',
            '   → /cloud_registered (Global Map)\n',
            '   → /cloud_registered_body (Current Scan)\n',
            '   → /lidar1_colored (L1 Visualization - Green)\n',
            '   → /lidar2_colored (L2 Visualization - Red)\n',
            '───────────────────────────────────────────────────────────────\n',
            ' Note: For visualization, run dual_mapping_desktop.launch.py\n',
            '       on a separate desktop machine.\n',
            '═══════════════════════════════════════════════════════════════\n'
        ]
    ))

    # Add FAST-LIO node
    ld.add_action(fast_lio_node)

    return ld
