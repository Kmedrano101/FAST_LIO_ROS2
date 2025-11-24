#!/usr/bin/env python3
"""
DESKTOP TEST 1: Raw LiDAR Data Visualization (Desktop PC)
==========================================================

This launch file runs on the DESKTOP PC to visualize data from Jetson.

Setup:
- Jetson: Runs livox_ros_driver2 (publishes point clouds + IMU)
- Desktop: Runs this launch file (visualizes with RViz)
- Both on same network

On Jetson:
    ros2 launch livox_ros_driver2 multiple_lidars.launch.py

On Desktop:
    ros2 launch fast_lio_ros2 desktop_test_1_raw_lidar.launch.py

What this does:
- Publishes static TF: livox_frame (sensor frame) ← needed for RViz
- Launches RViz with calibration config
- NO Livox driver (already running on Jetson)
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    # Arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2 for visualization'
    )

    # Find package
    fast_lio_share = FindPackageShare('fast_lio_ros2')

    # Static TF: Connect base_link to frame_default for RViz visualization
    # This allows RViz (fixed frame: base_link) to display lidars (frame_id: frame_default)
    tf_base_to_frame_default = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_frame_default',
        arguments=[
            '0', '0', '0',  # x, y, z
            '0', '0', '0',  # roll, pitch, yaw
            'base_link', 'frame_default'
        ],
        output='screen'
    )

    # RViz config
    rviz_config = PathJoinSubstitution([
        fast_lio_share, 'rviz', 'dual_lidar_calibration.rviz'
    ])

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    return LaunchDescription([
        use_rviz_arg,
        tf_base_to_frame_default,
        rviz_node,
    ])
