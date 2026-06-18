import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(__file__))
from cpu_real_common import build_robot_description, load_vehicle_config


def _launch_setup(context, *args, **kwargs):
    me_share = get_package_share_directory("me_nav2_bringup")
    livox_share = get_package_share_directory("livox_ros_driver2")

    vehicle_config_file = LaunchConfiguration("vehicle_config_file").perform(context)
    fast_lio_config_file = LaunchConfiguration("fast_lio_config_file").perform(context)
    slam_params_file = LaunchConfiguration("slam_params_file").perform(context)
    pointcloud_to_laserscan_params_file = LaunchConfiguration("pointcloud_to_laserscan_params_file").perform(context)
    rviz_config_file = LaunchConfiguration("rviz_config_file").perform(context)

    vehicle = load_vehicle_config(vehicle_config_file)
    map_name = LaunchConfiguration("map_name").perform(context)
    pcd_file = LaunchConfiguration("prior_pcd_file").perform(context)
    if not pcd_file:
        pcd_file = os.path.join(me_share, "pcd", f"{map_name}.pcd")

    robot_description = build_robot_description(vehicle)

    fast_lio_params = [
        fast_lio_config_file,
        {
            "use_sim_time": False,
            "common.lid_topic": vehicle["lidar_topic"],
            "common.imu_topic": vehicle["imu_topic"],
            "preprocess.lidar_type": int(vehicle["fast_lio"]["lidar_type"]),
            "preprocess.scan_line": int(vehicle["fast_lio"]["scan_line"]),
            "preprocess.blind": float(vehicle["fast_lio"]["blind"]),
            "preprocess.timestamp_unit": int(vehicle["fast_lio"]["timestamp_unit"]),
            "mapping.extrinsic_T": vehicle["fast_lio"]["extrinsic_T"],
            "mapping.extrinsic_R": vehicle["fast_lio"]["extrinsic_R"],
            "pcd_save.pcd_save_en": True,
            "pcd_save.interval": -1,
            "map_file_path": pcd_file,
        },
    ]

    livox_node = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_lidar_publisher",
        output="screen",
        parameters=[
            {
                "xfer_format": 1,
                "multi_topic": 0,
                "data_src": 0,
                "publish_freq": 10.0,
                "output_data_type": 0,
                "frame_id": vehicle["lidar_frame"],
                "lvx_file_path": "",
                "user_config_path": os.path.join(livox_share, "config", "MID360_config.json"),
                "cmdline_input_bd_code": "livox0000000001",
            }
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    fast_lio_node = Node(
        package="fast_lio",
        executable="fastlio_mapping",
        output="screen",
        parameters=fast_lio_params,
    )

    lio_interface_node = Node(
        package="lio_interface",
        executable="lio_interface_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "use_sim_time": False,
                "odometry_sub": "/Odometry",
                "base_frame": vehicle["base_frame"],
                "lidar_frame": vehicle["lidar_frame"],
            }
        ],
    )

    sensor_scan_generation_node = Node(
        package="sensor_scan_generation",
        executable="sensor_scan_generation_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "use_sim_time": False,
                "lidar_frame": vehicle["lidar_frame"],
                "base_footprint_frame": vehicle["base_frame"],
                "chassis_frame": vehicle["chassis_frame"],
            }
        ],
    )

    pointcloud_to_laserscan_node = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="Pointcloud2d_3d",
        output="screen",
        parameters=[
            pointcloud_to_laserscan_params_file,
            {
                "use_sim_time": False,
                "target_frame": vehicle["pointcloud_to_laserscan"]["target_frame"],
                "min_height": float(vehicle["pointcloud_to_laserscan"]["min_height"]),
                "max_height": float(vehicle["pointcloud_to_laserscan"]["max_height"]),
            },
        ],
        remappings=[("cloud_in", "/registered_scan"), ("scan", "/scan")],
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("slam_toolbox"), "launch", "online_async_launch.py")
        ),
        launch_arguments={
            "slam_params_file": slam_params_file,
            "use_sim_time": "false",
        }.items(),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return [
        livox_node,
        robot_state_publisher,
        fast_lio_node,
        lio_interface_node,
        sensor_scan_generation_node,
        pointcloud_to_laserscan_node,
        slam_launch,
        rviz_node,
    ]


def generate_launch_description():
    me_share = get_package_share_directory("me_nav2_bringup")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "vehicle_config_file",
                default_value=os.path.join(me_share, "config", "vehicle.yaml"),
            ),
            DeclareLaunchArgument("map_name", default_value="nav_test_4_27"),
            DeclareLaunchArgument(
                "fast_lio_config_file",
                default_value=os.path.join(get_package_share_directory("fast_lio"), "config", "mid360.yaml"),
            ),
            DeclareLaunchArgument(
                "slam_params_file",
                default_value=os.path.join(me_share, "config", "slam_toolbox_params.yaml"),
            ),
            DeclareLaunchArgument(
                "pointcloud_to_laserscan_params_file",
                default_value=os.path.join(me_share, "config", "Pointcloud2d_3d.yaml"),
            ),
            DeclareLaunchArgument(
                "rviz_config_file",
                default_value=os.path.join(me_share, "rviz", "nav2.rviz"),
            ),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument(
                "prior_pcd_file",
                default_value=PythonExpression(
                    [
                        "'",
                        os.path.join(me_share, "pcd"),
                        "/' + '",
                        LaunchConfiguration("map_name"),
                        "' + '.pcd'",
                    ]
                ),
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
