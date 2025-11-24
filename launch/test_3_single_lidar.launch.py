#!/usr/bin/env python3
"""
TEST 3: Single LiDAR FAST-LIO Baseline
=======================================

Purpose: Establish baseline performance with single LiDAR before dual fusion

This launch file starts:
- Livox driver (only LiDAR 1 will be used)
- FAST-LIO with single LiDAR configuration
- RViz for visualization

Usage:
    ros2 launch fast_lio_ros2 test_3_single_lidar.launch.py

Expected: Clean SLAM with single LiDAR, establish performance baseline
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

    # Config file
    config_file = PathJoinSubstitution([
        fast_lio_share, 'config', 'test_3_single_lidar.yaml'
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
        remappings=[
            # Standard remappings if needed
        ]
    )

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
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    return LaunchDescription([
        use_rviz_arg,
        livox_launch,
        fast_lio_node,
        rviz_node,
    ])
