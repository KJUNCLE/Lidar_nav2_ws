# Lidar_nav2_ws

[English Documentation](./README_EN.md)

面向 Livox MID-360 + ROS 2 Humble + CPU-only 车载 PC 的实车建图、定位与 Nav2 导航工作空间。

本分支是 `deploy/cpu-mid360-nav2`，目标是十一代 i5、无 GPU 的车载 PC 可直接部署。详细部署、脚本参数和逐步调试流程见：[CPU 实车部署说明](./docs/cpu_real_deployment.md)。

## 运行边界

- 传感器闭环：MID360 LiDAR/IMU。
- 预留传感器：同星 RTK-INSS5715，本阶段不进入定位闭环。
- 里程计：FAST-LIO，订阅 `/livox/lidar` CustomMsg 和 `/livox/imu`。
- 建图：FAST-LIO 3D PCD + `pointcloud_to_laserscan` + SLAM Toolbox 2D 栅格。
- 定位：默认 `small_gicp_relocalization`，未知起点可切换 `global_relocalization_kiss_matcher`，也可显式启用 `localization_backend:=humanoid` 的 Open3D humanoid 后端。
- 导航：Nav2，底盘接口固定为 `/cmd_vel`，消息类型 `geometry_msgs/msg/Twist`。

目标 TF：

```text
map -> odom -> base_footprint -> chassis -> livox_frame
```

## 快速使用

```bash
source /opt/ros/humble/setup.bash
cd ~/Lidar_nav2_ws
cd scripts
./build.sh
```

`scripts/build.sh` 等价于 `scripts/build_cpu_real.sh`，只构建实车白名单包。

Humanoid 可选后端单独构建：

```bash
Open3D_DIR=/abs/path/to/Open3DConfig.cmake-directory ./build_humanoid_localization.sh
```

建图：

```bash
cd ~/Lidar_nav2_ws/scripts
./mapping_real.sh map_name:=site_a use_rviz:=true
```

保存 2D 地图：

```bash
./save_map.sh site_a
```

保存 FAST-LIO 3D PCD：

```bash
./save_pcd.sh
```

地图文件：

```text
src/me_nav2_bringup/map/site_a.yaml
src/me_nav2_bringup/map/site_a.pgm
install/me_nav2_bringup/share/me_nav2_bringup/pcd/site_a.pcd
```

`save_pcd.sh` 不接收地图名参数；PCD 路径由启动建图时的 `map_name` 或 `prior_pcd_file` 决定。需要固定写到源码树时，启动建图时传入 `prior_pcd_file:=/abs/site_a.pcd`。

导航：

```bash
./nav2_real.sh map_name:=site_a relocalizer:=small_gicp
```

未知起点或丢定位恢复：

```bash
./nav2_real.sh map_name:=site_a relocalizer:=kiss
```

## 保留包

CPU 分支默认构建包列表为：

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

可选 humanoid 后端包：

```text
fast_lio_humanoid
open3d_loc_humanoid
```

第三方 KISS-Matcher C++ 源码保留在 `third_party/KISS-Matcher`，`robin-map` 保留在 `third_party/robin-map`，通过 `third_party/COLCON_IGNORE` 退出 colcon 包扫描，仅作为 `global_relocalization_kiss_matcher` 的 CMake 依赖使用。需要由 colcon 构建的第三方包 `small_gicp` 放在 `src/99_vendor/small_gicp`。

## 关键配置

- `src/me_nav2_bringup/config/vehicle.yaml`：frame 名称、footprint、MID360 外参、FAST-LIO 参数、点云切片高度；默认 `base_to_chassis.xyz` 为 `[0.0, 0.0, 0.12]`，`chassis_to_lidar.xyz` 为 `[0.35, 0.0, 0.18]`。
- `src/livox_ros_driver2-master/config/MID360s_config.json`：MID360s IP 和主机网卡配置。
- `src/me_nav2_bringup/config/nav2_params.yaml`：Nav2 控制器、规划器、代价地图参数。
- `src/me_nav2_bringup/config/slam_toolbox_params.yaml`：2D SLAM 参数。
- `src/me_nav2_bringup/config/Pointcloud2d_3d.yaml`：3D 点云转 2D LaserScan 参数。

## 核心话题

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` | MID360 点云输入 |
| `/livox/imu` | `sensor_msgs/msg/Imu` | MID360 内置 IMU |
| `/cloud_registered` | `sensor_msgs/msg/PointCloud2` | FAST-LIO 全局点云 |
| `/registered_scan` | `sensor_msgs/msg/PointCloud2` | 重定位和切片输入 |
| `/scan` | `sensor_msgs/msg/LaserScan` | Nav2 障碍物输入 |
| `/odom` | `nav_msgs/msg/Odometry` | 底盘里程计语义输出 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 输出给底盘 |

## 常用检查

构建阶段如果出现 `bash\r` 或 `std::clamp` 错误，先看 [环境搭建 - 常见问题](./环境搭建.md#8-常见问题) 和 [CPU 实车部署说明](./docs/cpu_real_deployment.md#21-编译-cpu-实车包)。

```bash
colcon list --names-only
ros2 launch me_nav2_bringup cpu_real_mapping.launch.py --show-args
ros2 launch me_nav2_bringup cpu_real_nav.launch.py --show-args
ros2 run tf2_ros tf2_echo map odom
ros2 topic hz /registered_scan
ros2 topic echo /cmd_vel
```

## 文档

- [CPU 实车部署说明](./docs/cpu_real_deployment.md)：一键脚本、逐个 launch、参数表、实车验收。
- [环境搭建](./环境搭建.md)：Ubuntu 22.04 + ROS 2 Humble 的依赖安装和构建检查。

## 许可证

本项目依据 [MIT License](./LICENSE) 开源。
