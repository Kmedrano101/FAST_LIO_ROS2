import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('fast_lio_ros2'),
        'config',
        'offline_colorizer.yaml'
    )

    return LaunchDescription([
        Node(
            package='fast_lio_ros2',
            executable='offline_colorizer_node',
            name='offline_colorizer',
            parameters=[config],
            output='screen',
        ),
    ])
