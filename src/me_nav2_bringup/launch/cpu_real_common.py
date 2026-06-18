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
        "base_to_chassis": {"xyz": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0]},
        "chassis_to_lidar": {"xyz": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0]},
        "fast_lio": {
            "lidar_type": 1,
            "scan_line": 4,
            "blind": 0.5,
            "timestamp_unit": 3,
            "extrinsic_T": [-0.011, -0.02329, 0.04412],
            "extrinsic_R": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
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

    return merged


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
