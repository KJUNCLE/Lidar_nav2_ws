import os
from typing import Any, Dict, List

import yaml


def load_vehicle_config(path: str) -> Dict[str, Any]:
    defaults: Dict[str, Any] = {
        "robot_name": "cpu_mid360_vehicle",
        "map_frame": "map",
        "odom_frame": "odom",
        "base_frame": "base_footprint",
        "chassis_frame": "chassis",
        "lidar_frame": "livox_frame",
        "lidar_topic": "/livox/lidar",
        "imu_topic": "/livox/imu",
        "footprint": [[0.21, 0.195], [0.21, -0.195], [-0.21, -0.195], [-0.21, 0.195]],
        "base_to_chassis": {"xyz": [0.0, 0.0, 0.12], "rpy": [0.0, 0.0, 0.0]},
        "chassis_to_lidar": {"xyz": [0.35, 0.0, 0.18], "rpy": [0.0, 0.0, 0.0]},
        "fast_lio": {
            "lidar_type": 1,
            "scan_line": 4,
            "blind": 0.5,
            "timestamp_unit": 3,
            "extrinsic_T": [-0.011, -0.02329, 0.04412],
            "extrinsic_R": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        },
        "humanoid_localization": {
            "base_frame": "base_footprint",
            "imu_frame": "livox_frame",
            "motion_frame": "chassis",
            "input_odometry_topic": "/Odometry_loc",
            "input_cloud_topic": "/cloud_registered_1",
            "map_points_topic": "/humanoid_loc/map_points",
            "submap_topic": "/humanoid_loc/submap",
            "scan_points_topic": "/humanoid_loc/scan_points",
            "scan2map_topic": "/humanoid_loc/scan2map",
            "pcd_queue_maxsize": 10,
            "voxelsize_coarse": 0.01,
            "voxelsize_fine": 0.2,
            "threshold_fitness": 0.5,
            "threshold_fitness_init": 0.5,
            "loc_frequence": 2.5,
            "save_scan": False,
            "save_scan_path": "/tmp/open3d_loc_scan_submap/",
            "hidden_removal": False,
            "maxpoints_source": 80000,
            "maxpoints_target": 400000,
            "filter_odom2map": False,
            "kalman_processVar2": 0.001,
            "kalman_estimatedMeasVar2": 0.02,
            "confidence_loc_th": 0.7,
            "dis_updatemap": 3.5,
        },
        "pointcloud_to_laserscan": {"target_frame": "livox_frame", "min_height": 0.2, "max_height": 1.0},
        "slam_toolbox": {"min_laser_range": 0.45, "max_laser_range": 40.0},
    }

    if not os.path.exists(path):
        return defaults

    with open(path, "r", encoding="utf-8") as f:
        loaded = yaml.safe_load(f) or {}

    vehicle = loaded.get("vehicle", loaded)

    merged = dict(defaults)
    for key, value in vehicle.items():
        merged[key] = value

    for key in ("base_to_chassis", "chassis_to_lidar", "fast_lio", "pointcloud_to_laserscan", "slam_toolbox"):
        merged[key] = dict(defaults[key]) | dict(vehicle.get(key, {}))

    humanoid_defaults = dict(defaults["humanoid_localization"])
    humanoid_defaults["base_frame"] = merged["base_frame"]
    humanoid_defaults["imu_frame"] = merged["lidar_frame"]
    humanoid_defaults["motion_frame"] = merged["chassis_frame"]
    merged["humanoid_localization"] = humanoid_defaults | dict(vehicle.get("humanoid_localization", {}))

    return merged


def build_humanoid_open3d_params(vehicle: Dict[str, Any], prior_pcd_file: str) -> Dict[str, Any]:
    humanoid = vehicle["humanoid_localization"]
    return {
        "path_map": prior_pcd_file,
        "map_frame": vehicle["map_frame"],
        "odom_frame": vehicle["odom_frame"],
        "base_frame": humanoid["base_frame"],
        "imu_frame": humanoid["imu_frame"],
        "motion_frame": humanoid["motion_frame"],
        "input_odometry_topic": humanoid["input_odometry_topic"],
        "input_cloud_topic": humanoid["input_cloud_topic"],
        "map_points_topic": humanoid["map_points_topic"],
        "submap_topic": humanoid["submap_topic"],
        "scan_points_topic": humanoid["scan_points_topic"],
        "scan2map_topic": humanoid["scan2map_topic"],
        "pcd_queue_maxsize": int(humanoid["pcd_queue_maxsize"]),
        "voxelsize_coarse": float(humanoid["voxelsize_coarse"]),
        "voxelsize_fine": float(humanoid["voxelsize_fine"]),
        "threshold_fitness": float(humanoid["threshold_fitness"]),
        "threshold_fitness_init": float(humanoid["threshold_fitness_init"]),
        "loc_frequence": float(humanoid["loc_frequence"]),
        "save_scan": bool(humanoid["save_scan"]),
        "save_scan_path": humanoid["save_scan_path"],
        "hidden_removal": bool(humanoid["hidden_removal"]),
        "maxpoints_source": int(humanoid["maxpoints_source"]),
        "maxpoints_target": int(humanoid["maxpoints_target"]),
        "filter_odom2map": bool(humanoid["filter_odom2map"]),
        "kalman_processVar2": float(humanoid["kalman_processVar2"]),
        "kalman_estimatedMeasVar2": float(humanoid["kalman_estimatedMeasVar2"]),
        "confidence_loc_th": float(humanoid["confidence_loc_th"]),
        "dis_updatemap": float(humanoid["dis_updatemap"]),
        "use_sim_time": False,
    }


def footprint_to_string(footprint: List[List[float]]) -> str:
    points = ", ".join(f"[{float(x):.6f}, {float(y):.6f}]" for x, y in footprint)
    return f"[{points}]"


def format_xyz_rpy(block: Dict[str, List[float]]) -> List[str]:
    xyz = block.get("xyz", [0.0, 0.0, 0.0])
    rpy = block.get("rpy", [0.0, 0.0, 0.0])
    return [
        f"{float(xyz[0]):.6f} {float(xyz[1]):.6f} {float(xyz[2]):.6f}",
        f"{float(rpy[0]):.6f} {float(rpy[1]):.6f} {float(rpy[2]):.6f}",
    ]


def build_robot_description(vehicle: Dict[str, Any]) -> str:
    robot_name = vehicle["robot_name"]
    base_frame = vehicle["base_frame"]
    chassis_frame = vehicle["chassis_frame"]
    lidar_frame = vehicle["lidar_frame"]
    base_xyz, base_rpy = format_xyz_rpy(vehicle["base_to_chassis"])
    lidar_xyz, lidar_rpy = format_xyz_rpy(vehicle["chassis_to_lidar"])

    return f"""<?xml version="1.0"?>
<robot name="{robot_name}">
  <link name="{base_frame}" />
  <link name="{chassis_frame}" />
  <link name="{lidar_frame}" />
  <joint name="base_to_chassis" type="fixed">
    <parent link="{base_frame}" />
    <child link="{chassis_frame}" />
    <origin xyz="{base_xyz}" rpy="{base_rpy}" />
  </joint>
  <joint name="chassis_to_lidar" type="fixed">
    <parent link="{chassis_frame}" />
    <child link="{lidar_frame}" />
    <origin xyz="{lidar_xyz}" rpy="{lidar_rpy}" />
  </joint>
</robot>
"""
