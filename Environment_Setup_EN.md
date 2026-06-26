# Lidar_nav2_ws CPU Real-Vehicle Environment Setup

This document only covers the `deploy/cpu-mid360-nav2` CPU-only MID360 real-vehicle branch. The full launch and parameter guide is maintained in Chinese: [CPU real-vehicle deployment](./docs/cpu_real_deployment.md).

## 1. Target Environment

| Item | Requirement |
| --- | --- |
| OS | Ubuntu 22.04 |
| ROS 2 | Humble Hawksbill |
| Onboard PC | CPU-only, scoped for an 11th-gen i5 without GPU |
| LiDAR | Livox MID360 |
| Chassis interface | `/cmd_vel`, `geometry_msgs/msg/Twist` |

Load ROS 2:

```bash
source /opt/ros/humble/setup.bash
printenv ROS_DISTRO
```

Expected output: `humble`.

## 2. Get the Code

```bash
cd ~
git clone <repo-url> Lidar_nav2_ws
cd Lidar_nav2_ws
git switch deploy/cpu-mid360-nav2
git submodule update --init --recursive
```

Kept root-level third-party dependencies:

- `third_party/robin-map`: KISS-Matcher dependency.
- `third_party/KISS-Matcher`: C++ dependency only, excluded from colcon package discovery.

The colcon-built third-party package `small_gicp` lives under `src/99_vendor/small_gicp`.

## 3. Install Dependencies

```bash
sudo apt update
sudo apt install -y \
  git wget curl vim gnupg lsb-release software-properties-common \
  build-essential cmake ninja-build pkg-config \
  python3-pip python3-colcon-common-extensions python3-rosdep \
  libtbb-dev libboost-all-dev libomp-dev
```

Initialize `rosdep`:

```bash
sudo rosdep init
rosdep update
```

Install package dependencies:

```bash
cd ~/Lidar_nav2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
```

Common runtime dependencies:

```bash
sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-pointcloud-to-laserscan \
  ros-humble-robot-state-publisher \
  ros-humble-tf2-tools
```

## 4. MID360 Network Check

Edit the Livox config:

```bash
vim src/livox_ros_driver2-master/config/MID360s_config.json
```

Check that:

- `host_net_info` matches the wired host NIC IP.
- `lidar_configs` matches the real MID360s IP.
- Host and MID360s are on the same subnet.

Connectivity check:

```bash
ping <MID360_IP>
```

## 5. Build

```bash
cd ~/Lidar_nav2_ws
source /opt/ros/humble/setup.bash
cd scripts
./build.sh
```

`scripts/build.sh` calls `scripts/build_cpu_real.sh` and builds only the CPU real-vehicle package whitelist.

Source the workspace after build:

```bash
source ~/Lidar_nav2_ws/install/setup.bash
```

## 6. Quick Verification

```bash
cd ~/Lidar_nav2_ws
colcon list --names-only
source install/setup.bash
ros2 launch me_nav2_bringup cpu_real_mapping.launch.py --show-args
ros2 launch me_nav2_bringup cpu_real_nav.launch.py --show-args
```

## 7. Real-Vehicle Run

Mapping:

```bash
cd ~/Lidar_nav2_ws/scripts
./mapping_real.sh map_name:=site_a use_rviz:=true
```

Save 2D map:

```bash
./save_map.sh site_a
```

Save the FAST-LIO 3D PCD:

```bash
./save_pcd.sh
```

`save_pcd.sh` does not accept a map-name argument. The PCD path is decided when mapping starts through `map_name` or `prior_pcd_file`; by default it is usually under `install/me_nav2_bringup/share/me_nav2_bringup/pcd/<map_name>.pcd`.

Navigation:

```bash
./nav2_real.sh map_name:=site_a relocalizer:=small_gicp
```

`nav2_real.sh` guards `source install/setup.bash` against unset-variable failures. The navigation launch starts and activates `map_server` first, then delays the Livox, FAST-LIO, Nav2, and relocalization chain.

Unknown startup pose:

```bash
./nav2_real.sh map_name:=site_a relocalizer:=kiss
```

Runtime checks:

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic hz /cloud_registered
ros2 topic hz /registered_scan
ros2 topic hz /scan
ros2 run tf2_ros tf2_echo map odom
ros2 topic echo /cmd_vel
```

## 8. Common Commands

```bash
cd ~/Lidar_nav2_ws/scripts
./build.sh
./mapping_real.sh map_name:=site_a
./save_map.sh site_a
./save_pcd.sh
./nav2_real.sh map_name:=site_a relocalizer:=small_gicp
./nav2_real.sh map_name:=site_a relocalizer:=kiss
./show_tf_tree.sh
```
