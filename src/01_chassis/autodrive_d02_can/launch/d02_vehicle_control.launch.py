# Copyright 2026 KJUNCLE
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    interface = LaunchConfiguration('interface')
    to_can_bus_topic = LaunchConfiguration('to_can_bus_topic')

    socket_can_sender = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros2_socketcan'),
                'launch',
                'socket_can_sender.launch.py',
            ])
        ),
        launch_arguments={
            'interface': interface,
            'enable_can_fd': 'false',
            'enable_frame_loopback': LaunchConfiguration('enable_frame_loopback'),
            'timeout_sec': LaunchConfiguration('sender_timeout_sec'),
            'to_can_bus_topic': to_can_bus_topic,
        }.items(),
    )

    vehicle_control_sender = Node(
        package='autodrive_d02_can',
        executable='d02_vehicle_control_sender_node',
        name='d02_vehicle_control_sender',
        output='screen',
        parameters=[{
            'output_frame_id': interface,
            'send_period_sec': LaunchConfiguration('send_period_sec'),
            'command_timeout_sec': LaunchConfiguration('command_timeout_sec'),
        }],
        remappings=[
            ('vehicle_control_cmd', LaunchConfiguration('vehicle_control_cmd_topic')),
            ('to_can_bus', to_can_bus_topic),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('interface', default_value='can2'),
        DeclareLaunchArgument('sender_timeout_sec', default_value='0.01'),
        DeclareLaunchArgument('enable_frame_loopback', default_value='false'),
        DeclareLaunchArgument('send_period_sec', default_value='0.01'),
        DeclareLaunchArgument('command_timeout_sec', default_value='0.1'),
        DeclareLaunchArgument('vehicle_control_cmd_topic', default_value='vehicle_control_cmd'),
        DeclareLaunchArgument('to_can_bus_topic', default_value='to_can_bus'),
        socket_can_sender,
        vehicle_control_sender,
    ])
