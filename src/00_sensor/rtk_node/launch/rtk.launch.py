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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("baud_rate", default_value="230400"),
        DeclareLaunchArgument("poll_period_ms", default_value="1"),
        DeclareLaunchArgument("reconnect_interval_ms", default_value="5000"),
        DeclareLaunchArgument("read_chunk_size", default_value="512"),
        DeclareLaunchArgument("imu_topic", default_value="/rtk/imu"),
        DeclareLaunchArgument("fix_topic", default_value="/rtk/fix"),
        DeclareLaunchArgument("data_topic", default_value="/rtk/data"),
        DeclareLaunchArgument("imu_frame_id", default_value="imu_link"),
        DeclareLaunchArgument("gps_frame_id", default_value="gps_link"),
        DeclareLaunchArgument("gnss_service_mask", default_value="15"),
        Node(
            package="rtk_node",
            executable="rtk_node",
            name="rtk_node",
            output="screen",
            parameters=[{
                "port": LaunchConfiguration("port"),
                "baud_rate": ParameterValue(LaunchConfiguration("baud_rate"), value_type=int),
                "poll_period_ms": ParameterValue(
                    LaunchConfiguration("poll_period_ms"), value_type=int),
                "reconnect_interval_ms": ParameterValue(
                    LaunchConfiguration("reconnect_interval_ms"), value_type=int),
                "read_chunk_size": ParameterValue(
                    LaunchConfiguration("read_chunk_size"), value_type=int),
                "imu_topic": LaunchConfiguration("imu_topic"),
                "fix_topic": LaunchConfiguration("fix_topic"),
                "data_topic": LaunchConfiguration("data_topic"),
                "imu_frame_id": LaunchConfiguration("imu_frame_id"),
                "gps_frame_id": LaunchConfiguration("gps_frame_id"),
                "gnss_service_mask": ParameterValue(
                    LaunchConfiguration("gnss_service_mask"), value_type=int),
            }],
        ),
    ])
