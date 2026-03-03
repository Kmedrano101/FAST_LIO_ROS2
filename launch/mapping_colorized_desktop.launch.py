#!/usr/bin/env python3
###############################################################################
# Colorized Mapping Desktop Visualization Launch File
# Target: Remote desktop — RViz subscribed to Jetson topics
#
# Usage:
#   ros2 launch fast_lio_ros2 mapping_colorized_desktop.launch.py
#   ros2 launch fast_lio_ros2 mapping_colorized_desktop.launch.py rviz:=/path/to/custom.rviz
#
# The core pipeline must be running on the Jetson:
#   ros2 launch fast_lio_ros2 mapping_colorized_core.launch.py
###############################################################################

import os.path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_path = get_package_share_directory('fast_lio_ros2')
    default_rviz_config_path = os.path.join(package_path, 'rviz', 'colorized.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_cfg = LaunchConfiguration('rviz')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true',
    )

    declare_rviz_config_cmd = DeclareLaunchArgument(
        'rviz',
        default_value=default_rviz_config_path,
        description='RViz configuration file path',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_rviz_config_cmd)

    ld.add_action(LogInfo(msg=[
        '\n',
        '===================================================================\n',
        ' Colorized Mapping Desktop Visualization\n',
        '===================================================================\n',
        ' RViz Config: ', rviz_cfg, '\n',
        ' Sim Time: ', use_sim_time, '\n',
        '-------------------------------------------------------------------\n',
        ' Subscribed Topics:\n',
        '   -> /Odometry (Pose + Velocity)\n',
        '   -> /path (Trajectory)\n',
        '   -> /cloud_registered (Global Map)\n',
        '   -> /cloud_registered_body (Current Scan)\n',
        '   -> /cloud_registered_rgb (Colorized Cloud)\n',
        '   -> /lidar1_colored (LiDAR 1 - Green)\n',
        '   -> /lidar2_colored (LiDAR 2 - Red)\n',
        '-------------------------------------------------------------------\n',
        ' Note: Core pipeline must be running on Jetson:\n',
        '   ros2 launch fast_lio_ros2 mapping_colorized_core.launch.py\n',
        '===================================================================\n',
    ]))

    ld.add_action(rviz_node)

    return ld
