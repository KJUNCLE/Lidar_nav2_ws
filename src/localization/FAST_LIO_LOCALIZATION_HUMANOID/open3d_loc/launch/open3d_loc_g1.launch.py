from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    open3d_loc_share = FindPackageShare("open3d_loc_humanoid")

    use_sim_time = LaunchConfiguration("use_sim_time")
    publish_static_transforms = LaunchConfiguration("publish_static_transforms")
    map_frame = LaunchConfiguration("map_frame")
    odom_frame = LaunchConfiguration("odom_frame")
    base_frame = LaunchConfiguration("base_frame")
    imu_frame = LaunchConfiguration("imu_frame")
    motion_frame = LaunchConfiguration("motion_frame")

    config_file = PathJoinSubstitution(
        [
            open3d_loc_share,
            "config",
            "loc_param_g1.yaml",
        ]
    )

    static_tf_camera_init2odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_init2odom",
        arguments=["0", "0", "0", "0", "0", "0", "1", odom_frame, "camera_init"],
        condition=IfCondition(publish_static_transforms),
    )

    static_tf_imulink2baselink = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="imulink2baselink",
        arguments=["0", "0", "0", "0", "0", "0", "1", imu_frame, base_frame],
        condition=IfCondition(publish_static_transforms),
    )

    static_tf_base_center = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_center_broadcaster",
        arguments=["0", "0", "0", "0", "0", "0", "1", base_frame, motion_frame],
        condition=IfCondition(publish_static_transforms),
    )

    global_localization_node = Node(
        package="open3d_loc_humanoid",
        executable="global_localization_node",
        name="global_localization_node",
        output="screen",
        parameters=[
            config_file,
            {
                "path_map": LaunchConfiguration("path_map"),
                "map_frame": map_frame,
                "odom_frame": odom_frame,
                "base_frame": base_frame,
                "imu_frame": imu_frame,
                "motion_frame": motion_frame,
                "input_odometry_topic": LaunchConfiguration("input_odometry_topic"),
                "input_cloud_topic": LaunchConfiguration("input_cloud_topic"),
                "map_points_topic": LaunchConfiguration("map_points_topic"),
                "submap_topic": LaunchConfiguration("submap_topic"),
                "scan_points_topic": LaunchConfiguration("scan_points_topic"),
                "scan2map_topic": LaunchConfiguration("scan2map_topic"),
                "pcd_queue_maxsize": 10,
                "voxelsize_coarse": 0.01,
                "voxelsize_fine": 0.2,
                "threshold_fitness": 0.5,
                "threshold_fitness_init": 0.5,
                "loc_frequence": 2.5,
                "save_scan": False,
                "save_scan_path": LaunchConfiguration("save_scan_path"),
                "hidden_removal": False,
                "maxpoints_source": 80000,
                "maxpoints_target": 400000,
                "filter_odom2map": False,
                "kalman_processVar2": 0.001,
                "kalman_estimatedMeasVar2": 0.02,
                "confidence_loc_th": 0.7,
                "dis_updatemap": 3.5,
                "use_sim_time": use_sim_time,
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("path_map", default_value=""),
            DeclareLaunchArgument("map_frame", default_value="map"),
            DeclareLaunchArgument("odom_frame", default_value="odom"),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument("imu_frame", default_value="imu_link"),
            DeclareLaunchArgument("motion_frame", default_value="motion_link"),
            DeclareLaunchArgument("input_odometry_topic", default_value="/Odometry_loc"),
            DeclareLaunchArgument("input_cloud_topic", default_value="/cloud_registered_1"),
            DeclareLaunchArgument("map_points_topic", default_value="/humanoid_loc/map_points"),
            DeclareLaunchArgument("submap_topic", default_value="/humanoid_loc/submap"),
            DeclareLaunchArgument("scan_points_topic", default_value="/humanoid_loc/scan_points"),
            DeclareLaunchArgument("scan2map_topic", default_value="/humanoid_loc/scan2map"),
            DeclareLaunchArgument("save_scan_path", default_value="/tmp/open3d_loc_scan_submap/"),
            DeclareLaunchArgument("publish_static_transforms", default_value="true"),
            static_tf_camera_init2odom,
            static_tf_imulink2baselink,
            static_tf_base_center,
            global_localization_node,
        ]
    )
