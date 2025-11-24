#!/usr/bin/env python3
"""
TEST 4: Dual LiDAR Static Fusion Core (Jetson)
===============================================

Purpose: Run core dual LiDAR fusion on Jetson without visualization

This launch file starts:
- Livox driver with both LiDARs
- FAST-LIO with dual LiDAR in BUNDLE mode

Usage:
    ros2 launch fast_lio_ros2 test_4_dual_static_core.launch.py

For visualization, run test_4_dual_static_viz.launch.py on desktop machine
Expected: Both LiDAR scans fused cleanly, thin walls, no ghosting
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Find packages
    livox_share = FindPackageShare('livox_ros_driver2')
    fast_lio_share = FindPackageShare('fast_lio_ros2')

    # Config file
    config_file = PathJoinSubstitution([
        fast_lio_share, 'config', 'test_4_dual_static.yaml'
    ])

    # Livox driver launch
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([livox_share, 'launch', 'multiple_lidars.launch.py'])
        ])
    )

    # FAST-LIO node
    fast_lio_node = Node(
        package='fast_lio_ros2',
        executable='fastlio_mapping',
        name='fast_lio_ros2_node',
        output='screen',
        parameters=[config_file],
    )

    return LaunchDescription([
        livox_launch,
        fast_lio_node,
    ])
