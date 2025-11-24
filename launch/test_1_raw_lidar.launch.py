#!/usr/bin/env python3
"""
TEST 1: Raw LiDAR Data Visualization
======================================

This launch file starts ONLY the Livox driver to verify:
- Both LiDARs are publishing data
- IMU is working
- Network connectivity is good

No FAST-LIO processing, just raw sensor data.

Usage:
    ros2 launch fast_lio_ros2 test_1_raw_lidar.launch.py

Then in separate terminal:
    rviz2 -d ~/ros2_ws/src/fast_lio_ros2/rviz/dual_lidar_calibration.rviz
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition


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
        rviz_node,
    ])
