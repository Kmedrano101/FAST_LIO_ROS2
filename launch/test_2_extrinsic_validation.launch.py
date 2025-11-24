#!/usr/bin/env python3
"""
TEST 2: Extrinsic Calibration Validation
=========================================

This launch file starts:
- Livox driver with both LiDARs
- Static TF publisher for extrinsic transforms
- RViz with calibration view

Purpose: Verify extrinsic parameters align sensors correctly

Usage:
    ros2 launch fast_lio_ros2 test_2_extrinsic_validation.launch.py

Expected: Point clouds should align on flat walls (single thin plane)
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
import math


def generate_launch_description():
    # Arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2 for visualization'
    )

    # Find packages
    livox_share = FindPackageShare('livox_ros_driver2')
    fast_lio_share = FindPackageShare('fast_lio_ros2')

    # Livox driver launch
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([livox_share, 'launch', 'multiple_lidars.launch.py'])
        ])
    )

    # Static TF: base_link → lidar_1 (IMU reference frame)
    # From Livox config: [0, 0.11, 0], Roll=90°, Yaw=180°
    # But for visualization, we show the transform from base_link
    tf_base_to_l1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_lidar1',
        arguments=[
            '0', '0.11', '0',  # x, y, z
            '3.14159', '0', '1.5708',  # yaw, pitch, roll (rad)
            'base_link', 'lidar_1_frame'
        ]
    )

    # Static TF: base_link → lidar_2
    # From Livox config: [0, -0.11, 0], Roll=90°, Yaw=0°
    tf_base_to_l2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_lidar2',
        arguments=[
            '0', '-0.11', '0',  # x, y, z
            '0', '0', '1.5708',  # yaw, pitch, roll (rad)
            'base_link', 'lidar_2_frame'
        ]
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
        livox_launch,
        tf_base_to_l1,
        tf_base_to_l2,
        rviz_node,
    ])
