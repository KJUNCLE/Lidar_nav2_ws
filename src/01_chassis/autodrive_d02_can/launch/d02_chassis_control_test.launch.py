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

    chassis_control_test = Node(
        package='autodrive_d02_can',
        executable='d02_chassis_control_test_node',
        name='d02_chassis_control_test',
        output='screen',
        parameters=[{
            'output_frame_id': interface,
            'test_case': LaunchConfiguration('test_case'),
            'send_period_sec': LaunchConfiguration('send_period_sec'),
            'brake_duration_sec': LaunchConfiguration('brake_duration_sec'),
            'motion_duration_sec': LaunchConfiguration('motion_duration_sec'),
        }],
        remappings=[
            ('to_can_bus', to_can_bus_topic),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('interface', default_value='can2'),
        DeclareLaunchArgument('sender_timeout_sec', default_value='0.01'),
        DeclareLaunchArgument('enable_frame_loopback', default_value='false'),
        DeclareLaunchArgument('to_can_bus_topic', default_value='to_can_bus'),
        DeclareLaunchArgument('test_case', default_value='brake'),
        DeclareLaunchArgument('send_period_sec', default_value='0.01'),
        DeclareLaunchArgument('brake_duration_sec', default_value='1.0'),
        DeclareLaunchArgument('motion_duration_sec', default_value='1.0'),
        socket_can_sender,
        chassis_control_test,
    ])
