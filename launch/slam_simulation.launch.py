import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition

from launch_ros.actions import Node


def generate_launch_description():
    """Launch FAST-LIO for Gazebo simulation with x500_lidar_360 drone."""

    package_path = get_package_share_directory('fast_lio_ros2')
    default_config_path = os.path.join(package_path, 'config')
    default_rviz_config_path = os.path.join(package_path, 'rviz', 'fastlio.rviz')

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')
    config_file = LaunchConfiguration('config_file')
    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='Yaml config file path'
    )
    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file', default_value='gazebosim.yaml',
        description='Config file for Gazebo simulation'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Use RViz to monitor results'
    )
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_cfg', default_value=default_rviz_config_path,
        description='RViz config file path'
    )

    # FAST-LIO mapping node
    fast_lio_node = Node(
        package='fast_lio_ros2',
        executable='fastlio_mapping',
        name='fast_lio_mapping',
        parameters=[
            PathJoinSubstitution([config_path, config_file]),
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # Static transform: base_link -> lidar_link
    # LiDAR is 0.052m above base_link (from x500_depth model SDF)
    static_tf_base_to_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_lidar',
        arguments=['0', '0', '0.052', '0', '0', '0', 'base_link', 'lidar_link'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Static transform: map -> camera_init (alias for FAST-LIO map frame)
    # This allows using "map" as the fixed frame in RViz
    static_tf_map_to_camera_init = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_to_camera_init',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'camera_init'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz_use)
    )

    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(declare_config_file_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)

    # Add nodes
    ld.add_action(static_tf_base_to_lidar)
    ld.add_action(static_tf_map_to_camera_init)
    ld.add_action(fast_lio_node)
    ld.add_action(rviz_node)

    return ld
