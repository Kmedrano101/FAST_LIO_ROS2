#!/usr/bin/env python3
"""
TEST 3: Single LiDAR FAST-LIO Visualization (Desktop)
=====================================================

Purpose: Run RViz visualization on desktop machine for remote monitoring

This launch file starts:
- RViz2 with FAST-LIO configuration

Usage:
    ros2 launch fast_lio_ros2 test_3_single_lidar_viz.launch.py

Prerequisites:
- Core launch file (test_3_single_lidar_core.launch.py) must be running on Jetson
- ROS_DOMAIN_ID must match between Jetson and Desktop
- Network connectivity between machines must be established
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Find package
    fast_lio_share = FindPackageShare('fast_lio_ros2')

    # RViz config
    rviz_config = PathJoinSubstitution([
        fast_lio_share, 'rviz', 'fastlio.rviz'
    ])

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        rviz_node,
    ])
