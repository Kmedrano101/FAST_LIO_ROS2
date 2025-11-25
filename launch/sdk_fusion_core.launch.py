#!/usr/bin/env python3
###############################################################################
# FAST-LIO SDK Fusion Core Launch File
# Target: Underground Mine Navigation with SDK-Fused Dual MID-360
# Hardware: NVIDIA Jetson ORIN
# ROS2: Humble/Iron
# Fusion Mode: Livox SDK (driver-level fusion)
#
# Usage:
#   ros2 launch fast_lio_ros2 sdk_fusion_core.launch.py
#   ros2 launch fast_lio_ros2 sdk_fusion_core.launch.py rviz:=true
#   ros2 launch fast_lio_ros2 sdk_fusion_core.launch.py config_file:=custom.yaml
###############################################################################

import os.path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    """
    Generate launch description for SDK fusion FAST-LIO mapping
    Assumes livox_ros_driver2 is configured with multi_topic: 0
    """
    # ==========================================================================
    # PATH CONFIGURATION
    # ==========================================================================
    package_path = get_package_share_directory('fast_lio_ros2')
    default_config_path = os.path.join(package_path, 'config')
    default_rviz_config_path = os.path.join(package_path, 'rviz', 'fastlio.rviz')

    # ==========================================================================
    # LAUNCH ARGUMENTS
    # ==========================================================================
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')
    config_file = LaunchConfiguration('config_file')
    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')

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
        default_value='dual_mid360_sdk_fusion.yaml',  # SDK fusion config
        description='YAML config filename for SDK fusion dual MID-360 setup'
    )

    # Argument: rviz
    # Enable/disable RViz visualization
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz',
        default_value='false',  # Disable by default for headless Jetson
        description='Launch RViz for visualization (true/false)'
    )

    # Argument: rviz_cfg
    # RViz configuration file path
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_cfg',
        default_value=default_rviz_config_path,
        description='RViz configuration file path'
    )

    # ==========================================================================
    # FAST-LIO NODE (SDK FUSION MODE)
    # ==========================================================================
    # Main SLAM node that performs LiDAR-inertial odometry and mapping
    # SDK Fusion Mode - subscribes to:
    #   - /livox/lidar (Fused point cloud from both MID-360 sensors)
    #   - /livox/imu (IMU data from primary sensor 192.168.1.10)
    # Publishes:
    #   - /Odometry (nav_msgs/Odometry)
    #   - /path (nav_msgs/Path)
    #   - /cloud_registered (sensor_msgs/PointCloud2) - Global map
    #   - /cloud_registered_body (sensor_msgs/PointCloud2) - Current scan in body frame
    # ==========================================================================
    fast_lio_node = Node(
        package='fast_lio_ros2',
        executable='fastlio_mapping',
        name='fastlio_sdk_fusion_node',
        output='screen',
        parameters=[
            PathJoinSubstitution([config_path, config_file]),
            {'use_sim_time': use_sim_time}
        ],
        # Increase output buffer to prevent log truncation
        arguments=['--ros-args', '--log-level', 'info'],
        # Topic remappings if needed (uncomment and modify if necessary)
        # remappings=[
        #     ('/livox/lidar', '/custom/fused_lidar'),
        #     ('/livox/imu', '/custom/imu'),
        # ]
    )

    # ==========================================================================
    # RVIZ NODE
    # ==========================================================================
    # Visualization node for monitoring SLAM performance
    # Displays:
    #   - Point clouds (current scan + global map)
    #   - Odometry path/trajectory
    #   - TF frames (transformations)
    # ==========================================================================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz_use)  # Only launch if rviz:=true
    )

    # ==========================================================================
    # LAUNCH DESCRIPTION ASSEMBLY
    # ==========================================================================
    ld = LaunchDescription()

    # Add all argument declarations
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(declare_config_file_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)

    # Add informational log message
    ld.add_action(LogInfo(
        msg=[
            '\n',
            '═══════════════════════════════════════════════════════════════\n',
            ' FAST-LIO SDK Fusion Mode - Underground Mine Mapping\n',
            '═══════════════════════════════════════════════════════════════\n',
            ' Config: ', config_file, '\n',
            ' RViz: ', rviz_use, '\n',
            ' Sim Time: ', use_sim_time, '\n',
            '───────────────────────────────────────────────────────────────\n',
            ' Expected Topics (SDK Fusion):\n',
            '   → /livox/lidar (Fused cloud from both MID-360s)\n',
            '   → /livox/imu (IMU from primary sensor 192.168.1.10)\n',
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
    ld.add_action(rviz_node)

    return ld