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
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
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

    cmd_vel_to_control = Node(
        package='autodrive_d02_can',
        executable='d02_cmd_vel_to_vehicle_control_node',
        name='d02_cmd_vel_to_vehicle_control',
        output='screen',
        parameters=[{
            'track_width_m': ParameterValue(
                LaunchConfiguration('track_width_m'), value_type=float),
            'rpm_per_mps': ParameterValue(
                LaunchConfiguration('rpm_per_mps'), value_type=float),
            'max_linear_mps': ParameterValue(
                LaunchConfiguration('max_linear_mps'), value_type=float),
            'max_angular_radps': ParameterValue(
                LaunchConfiguration('max_angular_radps'), value_type=float),
            'stop_epsilon': ParameterValue(
                LaunchConfiguration('stop_epsilon'), value_type=float),
            'shift_policy': LaunchConfiguration('shift_policy'),
        }],
        remappings=[
            ('cmd_vel', LaunchConfiguration('cmd_vel_topic')),
            ('vehicle_control_cmd', vehicle_control_cmd_topic),
        ],
    )

    vehicle_control_sender = Node(
        package='autodrive_d02_can',
        executable='d02_vehicle_control_sender_node',
        name='d02_vehicle_control_sender',
        output='screen',
        parameters=[{
            'output_frame_id': control_interface,
            'send_period_sec': ParameterValue(
                LaunchConfiguration('send_period_sec'), value_type=float),
            'command_timeout_sec': ParameterValue(
                LaunchConfiguration('control_command_timeout_sec'), value_type=float),
        }],
        remappings=[
            ('vehicle_control_cmd', vehicle_control_cmd_topic),
            ('to_can_bus', to_can_bus_topic),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('feedback_interface', default_value='can0'),
        DeclareLaunchArgument('control_interface', default_value='can1'),
        DeclareLaunchArgument('receiver_interval_sec', default_value='0.01'),
        DeclareLaunchArgument(
            'filters',
            default_value='7F1:7FF,168:7FF,100:7FF,51:7FF,77:7FF',
        ),
        DeclareLaunchArgument('use_bus_time', default_value='false'),
        DeclareLaunchArgument('sender_timeout_sec', default_value='0.01'),
        DeclareLaunchArgument('enable_frame_loopback', default_value='false'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('vehicle_control_cmd_topic', default_value='/vehicle_control_cmd'),
        DeclareLaunchArgument('from_can_bus_topic', default_value='/from_can_bus'),
        DeclareLaunchArgument('to_can_bus_topic', default_value='/to_can_bus'),
        DeclareLaunchArgument(
            'target_speed_feedback_topic',
            default_value='/target_speed_feedback',
        ),
        DeclareLaunchArgument('wheel_speed_feedback_topic', default_value='/wheel_speed_feedback'),
        DeclareLaunchArgument('battery_feedback_topic', default_value='/battery_feedback'),
        DeclareLaunchArgument(
            'vehicle_status_feedback_topic',
            default_value='/vehicle_status_feedback',
        ),
        DeclareLaunchArgument(
            'warning_level_feedback_topic',
            default_value='/warning_level_feedback',
        ),
        DeclareLaunchArgument('track_width_m', default_value='0.39'),
        DeclareLaunchArgument('rpm_per_mps', default_value='72.0'),
        DeclareLaunchArgument('max_linear_mps', default_value='0.3'),
        DeclareLaunchArgument('max_angular_radps', default_value='0.6'),
        DeclareLaunchArgument('stop_epsilon', default_value='0.0001'),
        DeclareLaunchArgument('shift_policy', default_value='auto'),
        DeclareLaunchArgument('send_period_sec', default_value='0.01'),
        DeclareLaunchArgument('control_command_timeout_sec', default_value='0.1'),
        GroupAction([
            PushRosNamespace('d02_feedback_can'),
            socket_can_receiver,
        ]),
        GroupAction([
            PushRosNamespace('d02_control_can'),
            socket_can_sender,
        ]),
        vehicle_feedback_decoder,
        cmd_vel_to_control,
        vehicle_control_sender,
    ])
