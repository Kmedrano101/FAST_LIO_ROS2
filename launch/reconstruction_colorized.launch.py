#!/usr/bin/env python3
###############################################################################
# Colorized Reconstruction Launch File
# Runs FAST-LIO (reconstruction mode) + colorizer for rosbag replay
#
# Usage:
#   Terminal 1:
#     ros2 launch fast_lio_ros2 reconstruction_colorized.launch.py
#
#   Terminal 2 (replay bag — raw sensor + GoPro only):
#     ros2 bag play <bag> --clock --rate 0.5 \
#       --topics /livox/lidar_192_168_1_10 /livox/lidar_192_168_1_18 \
#               /livox/imu_192_168_1_10 /livox/imu_192_168_1_18 \
#               /gopro/camera_0/image_raw/compressed \
#               /gopro/camera_1/image_raw/compressed
###############################################################################

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('fast_lio_ros2')

    reconstruction_config = os.path.join(pkg, 'config', 'reconstruction.yaml')
    colorizer_config = os.path.join(pkg, 'config', 'colorizer.yaml')

    # FAST-LIO in reconstruction mode
    fast_lio_node = Node(
        package='fast_lio_ros2',
        executable='fastlio_mapping',
        name='fastlio_mapping',
        output='screen',
        parameters=[
            reconstruction_config,
            {'use_sim_time': True},
        ],
        prefix='nice -n -10',
    )

    # Colorizer — subscribes to /cloud_registered_body + /Odometry + GoPro images
    colorizer_node = Node(
        package='fast_lio_ros2',
        executable='colorizer_node',
        name='pointcloud_colorizer',
        output='screen',
        parameters=[
            colorizer_config,
            {
                'use_sim_time': True,
                # Full density accumulation for reconstruction
                'pcd_save_en': True,
                'pcd_voxel_size': 0.0,
            },
        ],
    )

    return LaunchDescription([
        LogInfo(msg=[
            '\n',
            '===================================================================\n',
            ' Colorized Reconstruction (Bag Replay)\n',
            '===================================================================\n',
            ' Pipeline:\n',
            '   [Bag] LiDAR+IMU -> FAST-LIO -> /cloud_registered_body\n',
            '   [Bag] GoPro --------------------v\n',
            '   FAST-LIO /Odometry + body cloud -> Colorizer -> PCD save\n',
            '-------------------------------------------------------------------\n',
            ' Replay command:\n',
            '   ros2 bag play <bag> --clock --rate 0.5 \\\n',
            '     --topics /livox/lidar_192_168_1_10 /livox/lidar_192_168_1_18 \\\n',
            '             /livox/imu_192_168_1_10 /livox/imu_192_168_1_18 \\\n',
            '             /gopro/camera_0/image_raw/compressed \\\n',
            '             /gopro/camera_1/image_raw/compressed\n',
            '===================================================================\n',
        ]),
        fast_lio_node,
        colorizer_node,
    ])
