import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    lio_interface_node = Node(
        package='lio_interface',
        executable='lio_interface_node',
        namespace='',
        output='screen',
        emulate_tty=True,  # 开启提示颜色
        parameters=[{
            'use_sim_time': False,
            'odometry_sub': '/Odometry',
            'base_frame': 'base_footprint',
            'lidar_frame': 'livox_frame',
        }],
    )

    return LaunchDescription([lio_interface_node])
