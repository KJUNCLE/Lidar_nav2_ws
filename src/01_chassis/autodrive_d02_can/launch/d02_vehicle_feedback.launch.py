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
    from_can_bus_topic = LaunchConfiguration('from_can_bus_topic')

    socket_can_receiver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros2_socketcan'),
                'launch',
                'socket_can_receiver.launch.py',
            ])
        ),
        launch_arguments={
            'interface': interface,
            'enable_can_fd': 'false',
            'interval_sec': LaunchConfiguration('receiver_interval_sec'),
            'filters': LaunchConfiguration('filters'),
            'use_bus_time': LaunchConfiguration('use_bus_time'),
            'from_can_bus_topic': from_can_bus_topic,
        }.items(),
    )

    vehicle_feedback_decoder = Node(
        package='autodrive_d02_can',
        executable='d02_vehicle_feedback_decoder_node',
        name='d02_vehicle_feedback_decoder',
        output='screen',
        parameters=[{
            'output_frame_id': interface,
        }],
        remappings=[
            ('from_can_bus', from_can_bus_topic),
            ('target_speed_feedback', LaunchConfiguration('target_speed_feedback_topic')),
            ('wheel_speed_feedback', LaunchConfiguration('wheel_speed_feedback_topic')),
            ('battery_feedback', LaunchConfiguration('battery_feedback_topic')),
            ('vehicle_status_feedback', LaunchConfiguration('vehicle_status_feedback_topic')),
            ('warning_level_feedback', LaunchConfiguration('warning_level_feedback_topic')),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('interface', default_value='can2'),
        DeclareLaunchArgument('receiver_interval_sec', default_value='0.01'),
        DeclareLaunchArgument(
            'filters',
            default_value='7F1:7FF,168:7FF,100:7FF,51:7FF,77:7FF',
        ),
        DeclareLaunchArgument('use_bus_time', default_value='false'),
        DeclareLaunchArgument('from_can_bus_topic', default_value='from_can_bus'),
        DeclareLaunchArgument(
            'target_speed_feedback_topic',
            default_value='target_speed_feedback',
        ),
        DeclareLaunchArgument('wheel_speed_feedback_topic', default_value='wheel_speed_feedback'),
        DeclareLaunchArgument('battery_feedback_topic', default_value='battery_feedback'),
        DeclareLaunchArgument(
            'vehicle_status_feedback_topic',
            default_value='vehicle_status_feedback',
        ),
        DeclareLaunchArgument(
            'warning_level_feedback_topic',
            default_value='warning_level_feedback',
        ),
        socket_can_receiver,
        vehicle_feedback_decoder,
    ])
