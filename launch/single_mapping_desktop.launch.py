#!/usr/bin/env python3
###############################################################################
# FAST-LIO SDK Fusion Desktop Visualization Launch File
# Target: Remote desktop visualization for SDK-fused underground mine mapping
# ROS2: Humble/Iron
#
# Usage:
#   ros2 launch fast_lio_ros2 sdk_fusion_desktop.launch.py
#   ros2 launch fast_lio_ros2 sdk_fusion_desktop.launch.py rviz_cfg:=/path/to/custom.rviz
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
    default_rviz_config_path = os.path.join(package_path, 'rviz', 'fastlio.rviz')

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
        description='RViz configuration file path'
    )

    # ==========================================================================
    # RVIZ NODE
    # ==========================================================================
    # Visualization node for monitoring SLAM performance
    # Displays:
    #   - Point clouds (current scan + global map)
    #   - Odometry path/trajectory
    #   - TF frames (transformations)
    # Subscribes to existing topics published by FAST-LIO on robot
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
            ' FAST-LIO Single Mode Desktop Visualization\n',
            '═══════════════════════════════════════════════════════════════\n',
            ' RViz Config: ', rviz_cfg, '\n',
            ' Sim Time: ', use_sim_time, '\n',
            '───────────────────────────────────────────────────────────────\n',
            ' Subscribed Topics:\n',
            '   → /Odometry (Pose + Velocity)\n',
            '   → /path (Trajectory)\n',
            '   → /cloud_registered (Global Map)\n',
            '   → /cloud_registered_body (Current Scan)\n',
            '───────────────────────────────────────────────────────────────\n',
            ' Note: This launch file only provides visualization.\n',
            '       FAST-LIO core must be running on the payload.\n',
            '       (Use single_mapping_core.launch.py on Jetson)\n',
            '═══════════════════════════════════════════════════════════════\n'
        ]
    ))

    # Add RViz node
    ld.add_action(rviz_node)

    return ld