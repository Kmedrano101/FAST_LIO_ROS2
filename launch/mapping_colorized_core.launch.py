#!/usr/bin/env python3
###############################################################################
# Colorized Mapping Core Launch File
# Target: Jetson — Livox driver + FAST-LIO + GoPro cameras + colorizer
# No visualization — run mapping_colorized_desktop.launch.py on desktop
#
# Usage:
#   ros2 launch fast_lio_ros2 mapping_colorized_core.launch.py
#   ros2 launch fast_lio_ros2 mapping_colorized_core.launch.py lidar_config:=single
#   ros2 launch fast_lio_ros2 mapping_colorized_core.launch.py modo:=reconstruction
###############################################################################

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _launch_setup(context):
    lidar_config = LaunchConfiguration('lidar_config').perform(context)
    modo = LaunchConfiguration('modo').perform(context)

    fast_lio_pkg = get_package_share_directory('fast_lio_ros2')

    # ── Livox driver ──
    livox_pkg = get_package_share_directory('livox_ros_driver2')
    if lidar_config == 'dual':
        livox_launch_file = os.path.join(livox_pkg, 'launch', 'multiple_lidars.launch.py')
    else:
        livox_launch_file = os.path.join(livox_pkg, 'launch', 'single_mid360.launch.py')

    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(livox_launch_file)
    )

    # ── FAST-LIO mapping ──
    if lidar_config == 'dual':
        lio_launch_file = os.path.join(fast_lio_pkg, 'launch', 'dual_mapping_core.launch.py')
    else:
        lio_launch_file = os.path.join(fast_lio_pkg, 'launch', 'single_mapping_core.launch.py')

    # Forward modo argument to the mapping core launch
    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lio_launch_file),
        launch_arguments={'modo': modo}.items() if lidar_config == 'dual' else [],
    )

    # ── GoPro cameras ──
    gopro_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gopro_ros2'),
                'launch',
                'gopro_cameras.launch.py',
            )
        )
    )

    # ── Point cloud colorizer ──
    colorizer_config = os.path.join(
        fast_lio_pkg,
        'config',
        'colorizer.yaml',
    )

    colorizer_node = Node(
        package='fast_lio_ros2',
        executable='colorizer_node',
        name='pointcloud_colorizer',
        parameters=[colorizer_config],
        output='screen',
    )

    return [livox_launch, fast_lio_launch, gopro_launch, colorizer_node]


def generate_launch_description():
    lidar_config = LaunchConfiguration('lidar_config')
    modo = LaunchConfiguration('modo')

    return LaunchDescription([
        DeclareLaunchArgument(
            'lidar_config',
            default_value='dual',
            choices=['single', 'dual'],
            description='LiDAR configuration: single MID-360 or dual MID-360',
        ),
        DeclareLaunchArgument(
            'modo',
            default_value='default',
            description='Operation mode: "default", "navigation", or "reconstruction"',
        ),
        LogInfo(msg=[
            '\n',
            '===================================================================\n',
            ' Colorized Mapping Core (Jetson)\n',
            '===================================================================\n',
            ' LiDAR Config: ', lidar_config, '\n',
            ' Mode: ', modo, '\n',
            '-------------------------------------------------------------------\n',
            ' Pipeline:\n',
            '   Livox Driver -> FAST-LIO -> Colorizer\n',
            '                   GoPro ----^\n',
            '-------------------------------------------------------------------\n',
            ' Published Topics:\n',
            '   -> /Odometry (Pose + Velocity)\n',
            '   -> /path (Trajectory)\n',
            '   -> /cloud_registered (Global Map)\n',
            '   -> /cloud_registered_body (Current Scan)\n',
            '   -> /cloud_registered_rgb (Colorized Cloud)\n',
            '-------------------------------------------------------------------\n',
            ' Note: For visualization, run on desktop:\n',
            '   ros2 launch fast_lio_ros2 mapping_colorized_desktop.launch.py\n',
            '===================================================================\n',
        ]),
        OpaqueFunction(function=_launch_setup),
    ])
