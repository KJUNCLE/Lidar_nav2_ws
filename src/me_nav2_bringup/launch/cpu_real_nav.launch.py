import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

sys.path.insert(0, os.path.dirname(__file__))
from cpu_real_common import build_humanoid_open3d_params, build_robot_description, footprint_to_string, load_vehicle_config


def _make_nav2_params(nav2_params_file, vehicle):
    footprint = footprint_to_string(vehicle["footprint"])
    substitutions = {
        "bt_navigator.ros__parameters.global_frame": vehicle["map_frame"],
        "bt_navigator.ros__parameters.robot_base_frame": vehicle["base_frame"],
        "local_costmap.local_costmap.ros__parameters.global_frame": vehicle["odom_frame"],
        "local_costmap.local_costmap.ros__parameters.robot_base_frame": vehicle["base_frame"],
        "local_costmap.local_costmap.ros__parameters.footprint": footprint,
        "global_costmap.global_costmap.ros__parameters.global_frame": vehicle["map_frame"],
        "global_costmap.global_costmap.ros__parameters.robot_base_frame": vehicle["base_frame"],
        "global_costmap.global_costmap.ros__parameters.footprint": footprint,
    }
    return RewrittenYaml(
        source_file=nav2_params_file,
        root_key="",
        param_rewrites=substitutions,
        convert_types=True,
    )


def _make_navigation_nodes(configured_nav2_params):
    tf_remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]
    lifecycle_nodes = [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
        "velocity_smoother",
    ]

    return [
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            emulate_tty=True,
            parameters=[configured_nav2_params],
            remappings=tf_remappings + [("cmd_vel", "cmd_vel_nav")],
        ),
        Node(
            package="nav2_smoother",
            executable="smoother_server",
            name="smoother_server",
            output="screen",
            emulate_tty=True,
            parameters=[configured_nav2_params],
            remappings=tf_remappings,
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            emulate_tty=True,
            parameters=[configured_nav2_params],
            remappings=tf_remappings,
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            emulate_tty=True,
            parameters=[configured_nav2_params],
            remappings=tf_remappings,
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            emulate_tty=True,
            parameters=[configured_nav2_params],
            remappings=tf_remappings,
        ),
        Node(
            package="nav2_waypoint_follower",
            executable="waypoint_follower",
            name="waypoint_follower",
            output="screen",
            emulate_tty=True,
            parameters=[configured_nav2_params],
            remappings=tf_remappings,
        ),
        Node(
            package="nav2_velocity_smoother",
            executable="velocity_smoother",
            name="velocity_smoother",
            output="screen",
            emulate_tty=True,
            parameters=[configured_nav2_params],
            remappings=tf_remappings + [("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "cmd_vel")],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            emulate_tty=True,
            parameters=[{"use_sim_time": False}, {"autostart": True}, {"node_names": lifecycle_nodes}],
        ),
    ]


def _launch_setup(context, *args, **kwargs):
    me_share = get_package_share_directory("me_nav2_bringup")

    vehicle_config_file = LaunchConfiguration("vehicle_config_file").perform(context)
    nav2_params_file = LaunchConfiguration("nav2_params_file").perform(context)
    localization_backend = LaunchConfiguration("localization_backend").perform(context).lower()
    fast_lio_config_file = LaunchConfiguration("fast_lio_config_file").perform(context)
    humanoid_fast_lio_config_file = LaunchConfiguration("humanoid_fast_lio_config_file").perform(context)
    pointcloud_to_laserscan_params_file = LaunchConfiguration("pointcloud_to_laserscan_params_file").perform(context)
    rviz_config_file = LaunchConfiguration("rviz_config_file").perform(context)
    relocalizer = LaunchConfiguration("relocalizer").perform(context).lower()
    map_yaml_file = LaunchConfiguration("map_yaml_file").perform(context)
    prior_pcd_file = LaunchConfiguration("prior_pcd_file").perform(context)
    livox_config_file = LaunchConfiguration("livox_config_file").perform(context)

    if livox_config_file and not os.path.exists(livox_config_file):
        fallback_livox_config_file = os.path.join(
            get_package_share_directory("livox_ros_driver2"),
            "config",
            os.path.basename(livox_config_file),
        )
        if os.path.exists(fallback_livox_config_file):
            print(
                "[cpu_real_nav] livox_config_file not found: "
                f"{livox_config_file}. Falling back to {fallback_livox_config_file}"
            )
            livox_config_file = fallback_livox_config_file
        else:
            raise RuntimeError(f"livox_config_file does not exist: {livox_config_file}")

    vehicle = load_vehicle_config(vehicle_config_file)
    if localization_backend not in ("standard", "humanoid"):
        raise RuntimeError("localization_backend must be 'standard' or 'humanoid'")

    use_humanoid_backend = localization_backend == "humanoid"
    robot_description = build_robot_description(vehicle)
    configured_nav2_params = _make_nav2_params(nav2_params_file, vehicle)

    if not map_yaml_file:
        map_name = LaunchConfiguration("map_name").perform(context)
        map_yaml_file = os.path.join(me_share, "map", f"{map_name}.yaml")

    if not prior_pcd_file:
        map_name = LaunchConfiguration("map_name").perform(context)
        prior_pcd_file = os.path.join(me_share, "pcd", f"{map_name}.pcd")

    fast_lio_package = "fast_lio_humanoid" if use_humanoid_backend else "fast_lio"
    if use_humanoid_backend and not humanoid_fast_lio_config_file:
        humanoid_fast_lio_config_file = os.path.join(
            get_package_share_directory("fast_lio_humanoid"), "config", "mid360.yaml"
        )
    selected_fast_lio_config_file = humanoid_fast_lio_config_file if use_humanoid_backend else fast_lio_config_file

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
                "user_config_path": livox_config_file,
                "cmdline_input_bd_code": "livox0000000001",
            }
        ],
    )

    map_server_cmd = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{"yaml_filename": map_yaml_file, "use_sim_time": False}],
    )

    lifecycle_manager_map_cmd = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[{"use_sim_time": False}, {"autostart": True}, {"node_names": ["map_server"]}],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    fast_lio_node = Node(
        package=fast_lio_package,
        executable="fastlio_mapping",
        output="screen",
        parameters=[
            selected_fast_lio_config_file,
            {
                "use_sim_time": False,
                "common.lid_topic": vehicle["lidar_topic"],
                "common.imu_topic": vehicle["imu_topic"],
                "preprocess.lidar_type": int(vehicle["fast_lio"]["lidar_type"]),
                "preprocess.scan_line": int(vehicle["fast_lio"]["scan_line"]),
                "preprocess.blind": float(vehicle["fast_lio"]["blind"]),
                "preprocess.timestamp_unit": int(vehicle["fast_lio"]["timestamp_unit"]),
                "common.time_sync_en": bool(vehicle["fast_lio"]["time_sync_en"]),
                "common.time_offset_lidar_to_imu": float(vehicle["fast_lio"]["time_offset_lidar_to_imu"]),
                "mapping.extrinsic_est_en": bool(vehicle["fast_lio"]["extrinsic_est_en"]),
                "mapping.extrinsic_T": vehicle["fast_lio"]["extrinsic_T"],
                "mapping.extrinsic_R": vehicle["fast_lio"]["extrinsic_R"],
                "pcd_save.pcd_save_en": False,
                "pcd_save.interval": -1,
                "map_file_path": prior_pcd_file,
            },
        ],
    )

    lio_interface_node = Node(
        package="lio_interface",
        executable="lio_interface_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "use_sim_time": False,
                "odometry_sub": "/Odometry_loc" if use_humanoid_backend else "/Odometry",
                "cloud_sub": "/cloud_registered_1" if use_humanoid_backend else "/cloud_registered",
                "base_frame": vehicle["base_frame"],
                "lidar_frame": vehicle["lidar_frame"],
                "use_wall_time_for_nav": True,
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
                "use_wall_time_for_nav": True,
                "tf_future_tolerance": 0.5,
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
                "transform_tolerance": 0.5,
                "queue_size": 50,
            },
        ],
        remappings=[("cloud_in", "/registered_scan"), ("scan", "/scan")],
    )

    navigation_nodes = _make_navigation_nodes(configured_nav2_params)

    localization_node = None
    if use_humanoid_backend:
        localization_node = Node(
            package="open3d_loc_humanoid",
            executable="global_localization_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                os.path.join(get_package_share_directory("open3d_loc_humanoid"), "config", "loc_param_g1.yaml"),
                build_humanoid_open3d_params(vehicle, prior_pcd_file),
            ],
        )
    elif relocalizer == "kiss":
        localization_node = Node(
            package="global_relocalization_kiss_matcher",
            executable="global_kiss_matcher_relocalization_exec",
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    "num_threads": 4,
                    "num_neighbors": 10,
                    "global_leaf_size": 0.25,
                    "registered_leaf_size": 0.25,
                    "max_dist_sq": 1.0,
                    "voxel_resolution": 0.25,
                    "use_global_initialization": True,
                    "use_kiss_recovery": True,
                    "gicp_max_consecutive_failures": 2,
                    "recovery_min_points": 1000,
                    "recovery_cooldown_sec": 2.0,
                    "verify_kiss_with_gicp": True,
                    "loop.num_inliers_threshold": 3,
                    "loop.overlap_threshold": 80.0,
                    "map_frame": vehicle["map_frame"],
                    "odom_frame": vehicle["odom_frame"],
                    "base_frame": vehicle["base_frame"],
                    "lidar_frame": vehicle["lidar_frame"],
                    "robot_base_frame": vehicle["base_frame"],
                    "prior_pcd_file": prior_pcd_file,
                    "input_cloud_topic": "/registered_scan",
                    "tf_future_tolerance": 0.5,
                }
            ],
        )
    else:
        localization_node = Node(
            package="small_gicp_relocalization",
            executable="small_gicp_relocalization_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    "num_threads": 4,
                    "num_neighbors": 10,
                    "global_leaf_size": 0.25,
                    "registered_leaf_size": 0.25,
                    "max_dist_sq": 1.0,
                    "map_frame": vehicle["map_frame"],
                    "odom_frame": vehicle["odom_frame"],
                    "base_frame": vehicle["base_frame"],
                    "lidar_frame": vehicle["lidar_frame"],
                    "robot_base_frame": vehicle["base_frame"],
                    "prior_pcd_file": prior_pcd_file,
                    "input_cloud_topic": "/registered_scan",
                    "tf_future_tolerance": 0.5,
                }
            ],
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
        map_server_cmd,
        TimerAction(period=0.5, actions=[lifecycle_manager_map_cmd]),
        TimerAction(
            period=3.0,
            actions=[
                livox_node,
                robot_state_publisher,
                fast_lio_node,
                lio_interface_node,
                sensor_scan_generation_node,
                pointcloud_to_laserscan_node,
                *navigation_nodes,
                localization_node,
                rviz_node,
            ],
        ),
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
            DeclareLaunchArgument("localization_backend", default_value="standard"),
            DeclareLaunchArgument(
                "nav2_params_file",
                default_value=os.path.join(me_share, "config", "nav2_params.yaml"),
            ),
            DeclareLaunchArgument(
                "fast_lio_config_file",
                default_value=os.path.join(get_package_share_directory("fast_lio"), "config", "mid360.yaml"),
            ),
            DeclareLaunchArgument("humanoid_fast_lio_config_file", default_value=""),
            DeclareLaunchArgument(
                "pointcloud_to_laserscan_params_file",
                default_value=os.path.join(me_share, "config", "Pointcloud2d_3d.yaml"),
            ),
            DeclareLaunchArgument(
                "rviz_config_file",
                default_value=os.path.join(me_share, "rviz", "nav2.rviz"),
            ),
            DeclareLaunchArgument(
                "livox_config_file",
                default_value=os.path.join(
                    get_package_share_directory("livox_ros_driver2"), "config", "MID360s_config.json"
                ),
            ),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("relocalizer", default_value="small_gicp"),
            DeclareLaunchArgument(
                "map_yaml_file",
                default_value=PythonExpression(
                    [
                        "'",
                        os.path.join(me_share, "map"),
                        "/' + '",
                        LaunchConfiguration("map_name"),
                        "' + '.yaml'",
                    ]
                ),
            ),
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
