#!/usr/bin/env python3
###############################################################################
# FAST-LIO Dual LiDAR Desktop Visualization Launch File
# Target: Remote desktop visualization for dual LiDAR SLAM
# ROS2: Humble/Iron
#
# Usage:
#   ros2 launch fast_lio_ros2 dual_mapping_desktop.launch.py
#   ros2 launch fast_lio_ros2 dual_mapping_desktop.launch.py rviz_cfg:=dual_lidar_test.rviz
#
# This launch file only provides visualization tools (RViz).
# The FAST-LIO core must be running on the robot/Jetson.
###############################################################################

import os.path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Generate launch description for desktop visualization only
    No FAST-LIO computation node - connects to existing ROS2 topics
    """
    # ==========================================================================
    # PATH CONFIGURATION
    # ==========================================================================
    package_path = get_package_share_directory('fast_lio_ros2')
    default_rviz_config_path = os.path.join(package_path, 'rviz', 'dual_lidar_test.rviz')

    # ==========================================================================
    # LAUNCH ARGUMENTS
    # ==========================================================================
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_cfg = LaunchConfiguration('rviz_cfg')

    # Argument: use_sim_time
    # Set to 'false' for real hardware, 'true' for Gazebo simulation
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',  # Real hardware by default
        description='Use simulation (Gazebo) clock if true, real-time if false'
    )

    # Argument: rviz_cfg
    # RViz configuration file path
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_cfg',
        default_value=default_rviz_config_path,
        description='RViz configuration file path (default: dual_lidar_test.rviz)'
    )

    # ==========================================================================
    # RVIZ NODE
    # ==========================================================================
    # Visualization node for monitoring dual LiDAR SLAM
    # Displays:
    #   - LiDAR 1 point cloud (GREEN)
    #   - LiDAR 2 point cloud (RED)
    #   - Merged/registered point cloud
    #   - Global map
    #   - Odometry path/trajectory
    #   - TF frames (transformations)
    # Subscribes to topics published by FAST-LIO on robot:
    #   - /lidar1_colored
    #   - /lidar2_colored
    #   - /cloud_registered
    #   - /Odometry
    #   - /path
    # ==========================================================================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ==========================================================================
    # LAUNCH DESCRIPTION ASSEMBLY
    # ==========================================================================
    ld = LaunchDescription()

    # Add all argument declarations
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_rviz_config_path_cmd)

    # Add informational log message
    ld.add_action(LogInfo(
        msg=[
            '\n',
            '═══════════════════════════════════════════════════════════════\n',
            ' FAST-LIO Dual LiDAR Desktop Visualization\n',
            '═══════════════════════════════════════════════════════════════\n',
            ' RViz Config: ', rviz_cfg, '\n',
            ' Sim Time: ', use_sim_time, '\n',
            '───────────────────────────────────────────────────────────────\n',
            ' Subscribed Topics:\n',
            '   → /Odometry (Pose + Velocity)\n',
            '   → /path (Trajectory)\n',
            '   → /cloud_registered (Global Map - Merged)\n',
            '   → /cloud_registered_body (Current Scan)\n',
            '   → /lidar1_colored (LiDAR 1 - GREEN)\n',
            '   → /lidar2_colored (LiDAR 2 - RED)\n',
            '───────────────────────────────────────────────────────────────\n',
            ' Visualization Features:\n',
            '   ✓ Separate color-coded LiDAR clouds for alignment check\n',
            '   ✓ Real-time trajectory display\n',
            '   ✓ Global map accumulation\n',
            '   ✓ TF frame relationships\n',
            '───────────────────────────────────────────────────────────────\n',
            ' Extrinsic Alignment Verification:\n',
            '   1. Enable both /lidar1_colored and /lidar2_colored\n',
            '   2. Look for overlapping regions (GREEN + RED)\n',
            '   3. Verify geometric features align (walls, edges, corners)\n',
            '   4. If misaligned, adjust extrinsics in dual_mid360.yaml\n',
            '───────────────────────────────────────────────────────────────\n',
            ' Update Methods (configured on robot):\n',
            '   • ASYNC (update_method=1): Show individual scans\n',
            '   • BUNDLE (update_method=0): Show merged scans\n',
            '   • ADAPTIVE (update_method=2): Auto-switch based on quality\n',
            '───────────────────────────────────────────────────────────────\n',
            ' Note: FAST-LIO core must be running on the robot/Jetson.\n',
            '       Use: ros2 launch fast_lio_ros2 dual_mapping_core.launch.py\n',
            '═══════════════════════════════════════════════════════════════\n'
        ]
    ))

    # Add RViz node
    ld.add_action(rviz_node)

    return ld
