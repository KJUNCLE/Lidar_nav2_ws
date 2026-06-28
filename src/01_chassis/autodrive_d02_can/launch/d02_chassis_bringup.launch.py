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
    feedback_interface = LaunchConfiguration('feedback_interface')
    control_interface = LaunchConfiguration('control_interface')
    from_can_bus_topic = LaunchConfiguration('from_can_bus_topic')
    to_can_bus_topic = LaunchConfiguration('to_can_bus_topic')
    vehicle_control_cmd_topic = LaunchConfiguration('vehicle_control_cmd_topic')

    socket_can_receiver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros2_socketcan'),
                'launch',
                'socket_can_receiver.launch.py',
            ])
        ),
        launch_arguments={
            'interface': feedback_interface,
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
            'output_frame_id': feedback_interface,
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

    cmd_vel_converter = Node(
        package='autodrive_d02_can',
        executable='d02_cmd_vel_to_vehicle_control_node',
        name='d02_cmd_vel_to_vehicle_control',
        output='screen',
        parameters=[{
            'track_width_m': LaunchConfiguration('track_width_m'),
            'rpm_per_mps': LaunchConfiguration('rpm_per_mps'),
            'max_linear_mps': LaunchConfiguration('max_linear_mps'),
            'max_angular_radps': LaunchConfiguration('max_angular_radps'),
            'stop_epsilon': LaunchConfiguration('stop_epsilon'),
            'shift_policy': LaunchConfiguration('shift_policy'),
        }],
        remappings=[
            ('cmd_vel', LaunchConfiguration('cmd_vel_topic')),
            ('vehicle_control_cmd', vehicle_control_cmd_topic),
        ],
    )

    socket_can_sender = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros2_socketcan'),
                'launch',
                'socket_can_sender.launch.py',
            ])
        ),
        launch_arguments={
            'interface': control_interface,
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
            'output_frame_id': control_interface,
            'send_period_sec': LaunchConfiguration('send_period_sec'),
            'command_timeout_sec': LaunchConfiguration('command_timeout_sec'),
        }],
        remappings=[
            ('vehicle_control_cmd', vehicle_control_cmd_topic),
            ('to_can_bus', to_can_bus_topic),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('feedback_interface', default_value='can1'),
        DeclareLaunchArgument('control_interface', default_value='can1'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument(
            'vehicle_control_cmd_topic',
            default_value='/d02_chassis/vehicle_control_cmd',
        ),
        DeclareLaunchArgument('from_can_bus_topic', default_value='/d02_chassis/from_can_bus'),
        DeclareLaunchArgument('to_can_bus_topic', default_value='/d02_chassis/to_can_bus'),
        DeclareLaunchArgument('receiver_interval_sec', default_value='0.01'),
        DeclareLaunchArgument(
            'filters',
            default_value='7F1:7FF,168:7FF,100:7FF,51:7FF,77:7FF',
        ),
        DeclareLaunchArgument('use_bus_time', default_value='false'),
        DeclareLaunchArgument('sender_timeout_sec', default_value='0.01'),
        DeclareLaunchArgument('enable_frame_loopback', default_value='false'),
        DeclareLaunchArgument('send_period_sec', default_value='0.01'),
        DeclareLaunchArgument('command_timeout_sec', default_value='0.3'),
        DeclareLaunchArgument('track_width_m', default_value='0.39'),
        DeclareLaunchArgument('rpm_per_mps', default_value='72.0'),
        DeclareLaunchArgument('max_linear_mps', default_value='0.26'),
        DeclareLaunchArgument('max_angular_radps', default_value='1.0'),
        DeclareLaunchArgument('stop_epsilon', default_value='0.0001'),
        DeclareLaunchArgument('shift_policy', default_value='auto'),
        DeclareLaunchArgument(
            'target_speed_feedback_topic',
            default_value='/d02_chassis/target_speed_feedback',
        ),
        DeclareLaunchArgument(
            'wheel_speed_feedback_topic',
            default_value='/d02_chassis/wheel_speed_feedback',
        ),
        DeclareLaunchArgument(
            'battery_feedback_topic',
            default_value='/d02_chassis/battery_feedback',
        ),
        DeclareLaunchArgument(
            'vehicle_status_feedback_topic',
            default_value='/d02_chassis/vehicle_status_feedback',
        ),
        DeclareLaunchArgument(
            'warning_level_feedback_topic',
            default_value='/d02_chassis/warning_level_feedback',
        ),
        socket_can_receiver,
        vehicle_feedback_decoder,
        cmd_vel_converter,
        socket_can_sender,
        vehicle_control_sender,
    ])
