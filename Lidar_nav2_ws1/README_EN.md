# Lidar_nav2_ws

A CPU-only real-vehicle ROS 2 Humble workspace for Livox MID-360 mapping, localization, and Nav2 navigation.

This branch is `deploy/cpu-mid360-nav2`. It is scoped for an onboard 11th-gen i5 PC without a GPU. The full deployment guide is currently maintained in Chinese: [CPU real-vehicle deployment](./docs/cpu_real_deployment.md).

## Scope

- Closed-loop sensors: MID360 LiDAR and IMU.
- Reserved sensor: Tongxing RTK-INSS5715, not used in the localization loop yet.
- Odometry: FAST-LIO using `/livox/lidar` CustomMsg and `/livox/imu`.
- Mapping: FAST-LIO 3D PCD plus `pointcloud_to_laserscan` and SLAM Toolbox 2D grid mapping.
- Localization: `small_gicp_relocalization` by default, `global_relocalization_kiss_matcher` for unknown startup pose or recovery.
- Navigation: Nav2 with chassis output fixed to `/cmd_vel` as `geometry_msgs/msg/Twist`.

Target TF tree:

```text
map -> odom -> base_footprint -> chassis -> livox_frame
```

## Quick Start

```bash
source /opt/ros/humble/setup.bash
cd ~/Lidar_nav2_ws
git submodule update --init --recursive
cd scripts
./build.sh
```

Mapping:

```bash
cd ~/Lidar_nav2_ws/scripts
./mapping_real.sh map_name:=site_a use_rviz:=true
```

Save the 2D map:

```bash
./save_map.sh site_a
```

Navigation:

```bash
./nav2_real.sh map_name:=site_a relocalizer:=small_gicp
```

Unknown startup pose:

```bash
./nav2_real.sh map_name:=site_a relocalizer:=kiss
```

## Kept Packages

The CPU branch should expose only these workspace packages by default:

```text
fast_lio
gld_robot_description
global_relocalization_kiss_matcher
lio_interface
livox_ros_driver2
me_nav2_bringup
pcd2pgm
sensor_scan_generation
small_gicp
small_gicp_relocalization
tsl-robin-map
```

KISS-Matcher C++ sources are kept under `third_party/KISS-Matcher`. `third_party/COLCON_IGNORE` prevents colcon from treating third-party demo code as workspace packages.

## Key Docs

- [CPU real-vehicle deployment](./docs/cpu_real_deployment.md)
- [Environment setup](./Environment_Setup_EN.md)

## License

This project is open source under the [MIT License](./LICENSE).
