#!/usr/bin/env python3
###############################################################################
# FAST-LIO Dual MID-360 Launch File
# Target: Underground Mine Navigation with 2x Livox MID-360
# Hardware: NVIDIA Jetson ORIN
# ROS2: Humble/Iron
#
# Usage:
#   ros2 launch fast_lio_ros2 dual_mid360_mine.launch.py
#   ros2 launch fast_lio_ros2 dual_mid360_mine.launch.py rviz:=false
#   ros2 launch fast_lio_ros2 dual_mid360_mine.launch.py config_file:=custom.yaml
###############################################################################

import os.path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    """
    Generate launch description for dual MID-360 FAST-LIO mapping
    """
    # ==========================================================================
    # PATH CONFIGURATION
    # ==========================================================================
    package_path = get_package_share_directory('fast_lio_ros2')
    default_config_path = os.path.join(package_path, 'config')
    default_rviz_config_path = os.path.join(package_path, 'rviz', 'fastlio.rviz')

    # ==========================================================================
    # LAUNCH ARGUMENTS
    # ==========================================================================
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')
    config_file = LaunchConfiguration('config_file')
    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')

    # Argument: use_sim_time
    # Set to 'false' for real hardware, 'true' for Gazebo simulation
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',  # Real hardware by default
        description='Use simulation (Gazebo) clock if true, real-time if false'
    )

    # Argument: config_path
    # Directory containing YAML config files
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path',
        default_value=default_config_path,
        description='Directory path for YAML configuration files'
    )

    # Argument: config_file
    # YAML configuration filename (must exist in config_path)
    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file',
        default_value='dual_mid360_mine.yaml',  # Our custom config
        description='YAML config filename for dual MID-360 setup'
    )

    # Argument: rviz
    # Enable/disable RViz visualization
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz for visualization (true/false)'
    )

    # Argument: rviz_cfg
    # RViz configuration file path
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_cfg',
        default_value=default_rviz_config_path,
        description='RViz configuration file path'
    )

    # ==========================================================================
    # FAST-LIO NODE
    # ==========================================================================
    # Main SLAM node that performs LiDAR-inertial odometry and mapping
    # Subscribes to:
    #   - /livox/lidar_192_168_1_10 (Front MID-360)
    #   - /livox/lidar_192_168_1_18 (Rear MID-360)
    #   - /livox/imu_192_168_1_10 (IMU data)
    # Publishes:
    #   - /Odometry (nav_msgs/Odometry)
    #   - /path (nav_msgs/Path)
    #   - /cloud_registered (sensor_msgs/PointCloud2) - Global map
    #   - /cloud_registered_body (sensor_msgs/PointCloud2) - Current scan in body frame
    # ==========================================================================
    fast_lio_node = Node(
        package='fast_lio_ros2',
        executable='fastlio_mapping',
        name='fastlio_mapping_node',
        output='screen',
        parameters=[
            PathJoinSubstitution([config_path, config_file]),
            {'use_sim_time': use_sim_time}
        ],
        # Increase output buffer to prevent log truncation
        arguments=['--ros-args', '--log-level', 'info'],
        # Remap topics if needed (uncomment and modify if your topics differ)
        # remappings=[
        #     ('/livox/lidar_192_168_1_10', '/custom/lidar1'),
        #     ('/livox/lidar_192_168_1_18', '/custom/lidar2'),
        # ]
    )

    # ==========================================================================
    # RVIZ NODE
    # ==========================================================================
    # Visualization node for monitoring SLAM performance
    # Displays:
    #   - Point clouds (current scan + global map)
    #   - Odometry path/trajectory
    #   - TF frames (transformations)
    # ==========================================================================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz_use)  # Only launch if rviz:=true
    )

    # ==========================================================================
    # STATIC TRANSFORM PUBLISHERS (Optional)
    # ==========================================================================
    # If you need to publish static transforms between frames, add them here
    # Example: Publish transform from base_link to lidar1
    #
    # from launch_ros.actions import Node as StaticTFNode
    #
    # static_tf_lidar1 = StaticTFNode(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_tf_base_to_lidar1',
    #     arguments=['0.15', '-0.10', '0.05', '0', '0', '0', 'base_link', 'lidar1']
    #     # args: x y z yaw pitch roll parent_frame child_frame
    # )
    #
    # NOTE: FAST-LIO handles internal transforms, so this is usually not needed

    # ==========================================================================
    # DIAGNOSTICS NODE (Optional)
    # ==========================================================================
    # Add diagnostic monitoring for system health
    # Uncomment to enable:
    #
    # diagnostics_node = Node(
    #     package='diagnostic_aggregator',
    #     executable='aggregator_node',
    #     name='diagnostic_aggregator',
    #     parameters=[{
    #         'pub_rate': 1.0,
    #         'base_path': 'fastlio_diagnostics'
    #     }]
    # )

    # ==========================================================================
    # LAUNCH DESCRIPTION ASSEMBLY
    # ==========================================================================
    ld = LaunchDescription()

    # Add all argument declarations
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(declare_config_file_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)

    # Add informational log message
    ld.add_action(LogInfo(
        msg=[
            '\n',
            '═══════════════════════════════════════════════════════════════\n',
            ' FAST-LIO Dual MID-360 Underground Mine Mapping\n',
            '═══════════════════════════════════════════════════════════════\n',
            ' Config: ', config_file, '\n',
            ' RViz: ', rviz_use, '\n',
            ' Sim Time: ', use_sim_time, '\n',
            '───────────────────────────────────────────────────────────────\n',
            ' Expected Topics:\n',
            '   → /livox/lidar_192_168_1_10 (Front MID-360)\n',
            '   → /livox/lidar_192_168_1_18 (Rear MID-360)\n',
            '   → /livox/imu_192_168_1_10 (IMU)\n',
            '───────────────────────────────────────────────────────────────\n',
            ' Output Topics:\n',
            '   ← /Odometry (Pose + Velocity)\n',
            '   ← /path (Trajectory)\n',
            '   ← /cloud_registered (Global Map)\n',
            '   ← /cloud_registered_body (Current Scan)\n',
            '═══════════════════════════════════════════════════════════════\n'
        ]
    ))

    # Add nodes
    ld.add_action(fast_lio_node)
    ld.add_action(rviz_node)

    # Uncomment if using additional nodes:
    # ld.add_action(static_tf_lidar1)
    # ld.add_action(diagnostics_node)

    return ld


###############################################################################
# USAGE EXAMPLES
###############################################################################
#
# 1. Basic launch (with RViz):
#    $ ros2 launch fast_lio_ros2 dual_mid360_mine.launch.py
#
# 2. Launch without RViz (headless operation):
#    $ ros2 launch fast_lio_ros2 dual_mid360_mine.launch.py rviz:=false
#
# 3. Launch with custom config file:
#    $ ros2 launch fast_lio_ros2 dual_mid360_mine.launch.py \
#      config_file:=my_custom_config.yaml
#
# 4. Launch with simulation time (Gazebo):
#    $ ros2 launch fast_lio_ros2 dual_mid360_mine.launch.py \
#      use_sim_time:=true
#
# 5. Launch with all custom parameters:
#    $ ros2 launch fast_lio_ros2 dual_mid360_mine.launch.py \
#      config_file:=test.yaml \
#      rviz:=false \
#      use_sim_time:=false
#
###############################################################################

###############################################################################
# PRE-LAUNCH CHECKLIST
###############################################################################
#
# Before launching, ensure:
#
# ✓ Both Livox drivers are running:
#   $ ros2 topic list | grep livox
#   Should show:
#     /livox/imu_192_168_1_10
#     /livox/lidar_192_168_1_10
#     /livox/lidar_192_168_1_18
#
# ✓ Sensors are publishing at correct rates:
#   $ ros2 topic hz /livox/lidar_192_168_1_10
#   Should show ~10 Hz
#   $ ros2 topic hz /livox/lidar_192_168_1_18
#   Should show ~10 Hz
#
# ✓ Network configuration is correct:
#   - Host IP: 192.168.1.124
#   - LiDAR 1 IP: 192.168.1.10
#   - LiDAR 2 IP: 192.168.1.18
#
# ✓ Extrinsic calibration is measured and updated in config file:
#   - Check extrinsic_T and extrinsic_T2 in dual_mid360_mine.yaml
#   - Verify rotation matrices match sensor orientations
#
# ✓ Sufficient computational resources:
#   - Monitor with: tegrastats (on Jetson)
#   - Expected CPU: 30-60% on ORIN NX
#   - Expected RAM: 1-2 GB
#
###############################################################################

###############################################################################
# TROUBLESHOOTING
###############################################################################
#
# PROBLEM: "No point cloud received"
# SOLUTION:
#   - Check livox_ros_driver2 is running
#   - Verify multi_topic: 1 in livox_params.yaml
#   - Check topic names match config file
#   - Test: ros2 topic echo /livox/lidar_192_168_1_10 --field width
#
# PROBLEM: "Filter divergence" or "Lost tracking"
# SOLUTION:
#   - Increase adaptive_fov_threshold to force more bundle updates
#   - Increase acc_cov/gyr_cov if high vibration
#   - Check extrinsic calibration accuracy
#   - Reduce max speed if motion too aggressive
#
# PROBLEM: "High CPU usage" or "Slow performance"
# SOLUTION:
#   - Increase point_filter_num (more downsampling)
#   - Increase filter_size_surf and filter_size_map
#   - Disable dense_publish_en
#   - Reduce cube_side_length
#
# PROBLEM: "Point clouds misaligned in RViz"
# SOLUTION:
#   - Check extrinsic_T and extrinsic_T2 values
#   - Verify rotation matrices (extrinsic_R and extrinsic_R2)
#   - Recalibrate with offline method (GICP)
#   - Check TF tree: ros2 run tf2_tools view_frames
#
# PROBLEM: "IMU saturation warnings"
# SOLUTION:
#   - Increase acc_cov and gyr_cov parameters
#   - Check physical mounting (reduce vibration)
#   - Verify IMU calibration
#
###############################################################################
