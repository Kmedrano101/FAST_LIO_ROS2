#!/usr/bin/env python3
"""
DESKTOP TEST 2: Extrinsic Calibration Validation (Desktop PC)
==============================================================

This launch file runs on the DESKTOP PC to validate extrinsic calibration.

Setup:
- Jetson: Runs livox_ros_driver2 (publishes point clouds + IMU)
- Desktop: Runs this launch file (visualizes with TF transforms)
- Both on same network

On Jetson:
    ros2 launch livox_ros_driver2 multiple_lidars.launch.py

On Desktop:
    ros2 launch fast_lio_ros2 desktop_test_2_extrinsic_validation.launch.py

What this does:
- Publishes TF tree: world → base_link → lidar_1_frame, lidar_2_frame
- Uses extrinsics from livox driver config
- Launches RViz for alignment visualization

Expected: Point clouds align on walls (single thin plane, 10-20cm)
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
import math


def generate_launch_description():
    # Arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2 for visualization'
    )

    # Find package
    fast_lio_share = FindPackageShare('fast_lio_ros2')

    # Static TF: world → base_link (identity, just for reference)
    tf_world_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_base_link',
        arguments=[
            '0', '0', '0',  # x, y, z
            '0', '0', '0',  # roll, pitch, yaw
            'world', 'base_link'
        ],
        output='screen'
    )

    # Static TF: base_link → lidar_1_frame (LiDAR 1)
    # From multiple_netconfigs.json:
    #   Position: [x=0, y=110mm, z=0] = [0, 0.11, 0]
    #   Orientation: Roll=90°, Pitch=0°, Yaw=180°
    # Convert to radians: Roll=1.5708, Pitch=0, Yaw=3.14159
    # Order for static_transform_publisher: x y z yaw pitch roll
    tf_base_to_l1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_lidar1',
        arguments=[
            '0', '0.11', '0',           # x, y, z (meters)
            '3.14159', '0', '1.5708',   # yaw, pitch, roll (radians)
            'base_link', 'lidar_1_frame'
        ],
        output='screen'
    )

    # Static TF: base_link → lidar_2_frame (LiDAR 2)
    # From multiple_netconfigs.json:
    #   Position: [x=0, y=-110mm, z=0] = [0, -0.11, 0]
    #   Orientation: Roll=90°, Pitch=0°, Yaw=0°
    # Convert to radians: Roll=1.5708, Pitch=0, Yaw=0
    tf_base_to_l2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_lidar2',
        arguments=[
            '0', '-0.11', '0',          # x, y, z (meters)
            '0', '0', '1.5708',         # yaw, pitch, roll (radians)
            'base_link', 'lidar_2_frame'
        ],
        output='screen'
    )

    # Static TF: base_link → frame_default (for point cloud visualization)
    # The Livox driver publishes clouds with frame_id: "frame_default"
    # We need to connect this to base_link so RViz can display the clouds
    tf_base_to_frame_default = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_frame_default',
        arguments=[
            '0', '0', '0',  # identity
            '0', '0', '0',
            'base_link', 'frame_default'
        ],
        output='screen'
    )

    # RViz config - update fixed frame to base_link
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
        tf_world_to_base,
        tf_base_to_l1,
        tf_base_to_l2,
        tf_base_to_frame_default,
        rviz_node,
    ])
