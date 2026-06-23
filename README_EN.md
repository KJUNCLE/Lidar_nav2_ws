# Lidar_nav2_ws

A CPU-only real-vehicle ROS 2 Humble workspace for Livox MID-360 mapping, localization, and Nav2 navigation.

This branch is `deploy/cpu-mid360-nav2`. It is scoped for an onboard 11th-gen i5 PC without a GPU. The full deployment guide is currently maintained in Chinese: [CPU real-vehicle deployment](./docs/cpu_real_deployment.md).

## Scope

- Closed-loop sensors: MID360 LiDAR and IMU.
- Reserved sensor: Tongxing RTK-INSS5715, not used in the localization loop yet.
- Odometry: FAST-LIO using `/livox/lidar` CustomMsg and `/livox/imu`.
- Mapping: FAST-LIO 3D PCD plus `pointcloud_to_laserscan` and SLAM Toolbox 2D grid mapping.
- Localization: `small_gicp_relocalization` by default, `global_relocalization_kiss_matcher` for unknown startup pose or recovery, and an optional humanoid backend via `localization_backend:=humanoid`.
- Navigation: Nav2 with chassis output fixed to `/cmd_vel` as `geometry_msgs/msg/Twist`.

Target TF tree:

```text
map -> odom -> base_footprint -> chassis -> livox_frame
```

Default fixed TF offsets live in `src/me_nav2_bringup/config/vehicle.yaml`: `base_to_chassis.xyz` is `[0.0, 0.0, 0.12]`, and `chassis_to_lidar.xyz` is `[0.35, 0.0, 0.18]`.

## Quick Start

```bash
source /opt/ros/humble/setup.bash
cd ~/Lidar_nav2_ws
cd scripts
./build.sh
```

Build the optional humanoid localization backend separately:

```bash
Open3D_DIR=/abs/path/to/Open3DConfig.cmake-directory ./build_humanoid_localization.sh
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

Save the FAST-LIO 3D PCD:

```bash
./save_pcd.sh
```

Generated files:

```text
src/me_nav2_bringup/map/site_a.yaml
src/me_nav2_bringup/map/site_a.pgm
install/me_nav2_bringup/share/me_nav2_bringup/pcd/site_a.pcd
```

`save_pcd.sh` does not accept a map-name argument. The PCD path is decided when mapping starts through `map_name` or `prior_pcd_file`; pass `prior_pcd_file:=/abs/site_a.pcd` if the PCD must be written to a fixed source-tree path.

Navigation:

```bash
./nav2_real.sh map_name:=site_a relocalizer:=small_gicp
```

`nav2_real.sh` guards `source install/setup.bash` against unset-variable failures and `cpu_real_nav.launch.py` starts the map server before the delayed sensor/Nav2/relocalization chain.

Unknown startup pose:

```bash
./nav2_real.sh map_name:=site_a relocalizer:=kiss
```

## Kept Packages

The default CPU build selects these packages:

```text
fast_lio
gld_robot_description
global_relocalization_kiss_matcher
lio_interface
livox_ros_driver2
livox_sdk2
me_nav2_bringup
pcd2pgm
sensor_scan_generation
small_gicp
small_gicp_relocalization
tsl-robin-map
```

Optional humanoid backend packages:

```text
fast_lio_humanoid
open3d_loc_humanoid
```

KISS-Matcher C++ sources are kept under `third_party/KISS-Matcher`. `third_party/COLCON_IGNORE` prevents colcon from treating third-party demo code as workspace packages.

## Key Docs

- [CPU real-vehicle deployment](./docs/cpu_real_deployment.md)
- [Environment setup](./Environment_Setup_EN.md)

## License

This project is open source under the [MIT License](./LICENSE).
