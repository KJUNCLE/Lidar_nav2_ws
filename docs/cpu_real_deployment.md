# CPU-only MID360 实车部署说明

本文档面向 `deploy/cpu-mid360-nav2` 分支，说明如何在无 GPU 的车载 PC 上部署 MID360 + FAST-LIO + Nav2 实车闭环。当前版本不把同星 RTK-INSS5715 接入定位闭环，仅保留后续扩展接口。

## 1. 前置检查

### 1.1 分支和系统

推荐环境：

| 项目 | 要求 |
| --- | --- |
| 操作系统 | Ubuntu 22.04 |
| ROS 2 | Humble |
| CPU | 十一代 i5 可用 |
| GPU | 不需要 |
| LiDAR | Livox MID360 |
| 底盘接口 | 订阅 `/cmd_vel`，消息类型 `geometry_msgs/msg/Twist` |

确认当前分支：

```bash
git branch --show-current
```

期望输出：

```text
deploy/cpu-mid360-nav2
```

加载 ROS 2 环境：

```bash
source /opt/ros/humble/setup.bash
```

### 1.2 MID360 网络

实车启动前检查：

```bash
ping <MID360_IP>
```

确认 MID360 配置文件：

```bash
vim src/livox_ros_driver2/config/MID360_config.json
```

重点检查：

- `host_net_info` 中的主机 IP 是否是连接 MID360 的网卡 IP。
- `lidar_configs` 中的 MID360 IP 是否与实际设备一致。
- 主机和 MID360 是否在同一网段。

### 1.3 地图命名规则

本分支按 `map_name` 统一管理 2D 地图和 3D PCD：

```text
2D map: src/me_nav2_bringup/map/<map_name>.yaml
2D pgm: src/me_nav2_bringup/map/<map_name>.pgm
3D pcd: src/me_nav2_bringup/pcd/<map_name>.pcd
```

例如 `map_name:=site_a` 对应：

```text
src/me_nav2_bringup/map/site_a.yaml
src/me_nav2_bringup/map/site_a.pgm
src/me_nav2_bringup/pcd/site_a.pcd
```

导航时必须保证 2D map 和 3D PCD 来自同一次或同一坐标系下的建图结果。

### 1.4 车体配置

车体尺寸、坐标系、MID360 外参和点云切片参数集中在：

```text
src/me_nav2_bringup/config/vehicle.yaml
```

默认值是占位配置，上车前建议实测后修改：

| 字段 | 默认值 | 用处 | 何时修改 |
| --- | --- | --- | --- |
| `map_frame` | `map` | 全局地图坐标系 | 一般不改 |
| `odom_frame` | `odom` | 里程计坐标系 | 一般不改 |
| `base_frame` | `base_footprint` | Nav2 使用的车体基准坐标系 | 车体 TF 命名不同时修改 |
| `chassis_frame` | `chassis` | 底盘坐标系 | URDF/机械坐标系不同时修改 |
| `lidar_frame` | `livox_frame` | MID360 坐标系 | LiDAR frame 名称不同时修改 |
| `lidar_topic` | `/livox/lidar` | FAST-LIO 点云输入 | 驱动输出话题变更时修改 |
| `imu_topic` | `/livox/imu` | FAST-LIO IMU 输入 | 驱动输出话题变更时修改 |
| `footprint` | 0.42 m x 0.39 m 矩形 | Nav2 代价地图碰撞轮廓 | 车体尺寸变化时必须修改 |
| `base_to_chassis.xyz/rpy` | 全 0 | `base_footprint -> chassis` 固定 TF | 实测坐标偏置后修改 |
| `chassis_to_lidar.xyz/rpy` | 全 0 | `chassis -> livox_frame` 固定 TF | MID360 安装位置/姿态变化时必须修改 |
| `fast_lio.lidar_type` | `1` | Livox CustomMsg 输入 | MID360 CPU 实车默认不改 |
| `fast_lio.scan_line` | `4` | MID360 扫描线数配置 | 一般不改 |
| `fast_lio.blind` | `0.5` | 近距离盲区过滤 | 近距离噪声或车体点过多时调整 |
| `fast_lio.timestamp_unit` | `3` | Livox 时间戳单位 | 使用当前 MID360 驱动时不改 |
| `fast_lio.extrinsic_T/R` | 当前占位/默认值 | FAST-LIO 内部 IMU-LiDAR 外参 | 标定后替换 |
| `pointcloud_to_laserscan.min_height` | `0.2` | 3D 点云切片下界 | 地面点或车体点进入 `/scan` 时调整 |
| `pointcloud_to_laserscan.max_height` | `1.0` | 3D 点云切片上界 | 低矮障碍或高障碍识别不理想时调整 |
| `slam_toolbox.*` | 预留字段 | 集中配置占位 | 当前 launch 不直接覆盖这些值 |

目标 TF 树：

```text
map -> odom -> base_footprint -> chassis -> livox_frame
```

## 2. 一键流程

以下命令都从工作空间根目录进入 `scripts` 后执行：

```bash
cd ~/Lidar_nav2_ws/scripts
```

### 2.1 编译 CPU 实车包

```bash
source /opt/ros/humble/setup.bash
./build.sh
```

`build.sh` 只是入口脚本，实际执行：

```bash
./build_cpu_real.sh
```

`build_cpu_real.sh` 只编译实车 CPU 白名单包：

```text
small_gicp
livox_ros_driver2
fast_lio
lio_interface
sensor_scan_generation
gld_robot_description
me_nav2_bringup
pcd2pgm
small_gicp_relocalization
global_relocalization_kiss_matcher
```

它不会全量编译 CUDA、Open3D、RTAB-Map、Gazebo 仿真、Point-LIO 等非本阶段部署包。

脚本固定使用：

| 构建参数 | 用处 |
| --- | --- |
| `--symlink-install` | 安装区使用符号链接，便于修改 launch/config 后快速生效 |
| `--packages-select ...` | 只构建 CPU 实车白名单包 |
| `--cmake-args -DCMAKE_BUILD_TYPE=Release` | Release 编译，降低 CPU 运行压力 |
| `--parallel-workers 2` | 限制并发，避免车载 PC 编译时内存/温度压力过大 |
| `"$@"` | 透传额外 colcon 参数 |

额外参数透传示例：

```bash
./build_cpu_real.sh --event-handlers console_direct+
```

构建阶段常见错误：

| 现象 | 根因 | 处理 |
| --- | --- | --- |
| `/usr/bin/env: 'bash\r': No such file or directory` | 脚本被 Windows/CRLF 行尾污染，Linux 把解释器读成 `bash\r` | 执行 `sed -i 's/\r$//' scripts/*.sh && chmod +x scripts/*.sh`；仓库已用 `.gitattributes` 固定脚本 LF 行尾 |
| `error: 'clamp' is not a member of 'std'` | `livox_ros_driver2` 默认 C++14，但源码使用了 C++17 的 `std::clamp` | CPU 分支已改为 C++14 兼容边界判断；更新分支后重新执行 `./build_cpu_real.sh` |

编译完成后加载工作空间：

```bash
source ../install/setup.bash
```

### 2.2 实车建图

```bash
./mapping_real.sh map_name:=site_a use_rviz:=true
```

这个脚本等价于：

```bash
cd ~/Lidar_nav2_ws
source install/setup.bash
ros2 launch me_nav2_bringup cpu_real_mapping.launch.py map_name:=site_a use_rviz:=true
```

启动内容：

| 节点/组件 | 作用 | 关键输入 | 关键输出 |
| --- | --- | --- | --- |
| `livox_ros_driver2_node` | MID360 驱动 | MID360 以太网数据 | `/livox/lidar`, `/livox/imu` |
| `robot_state_publisher` | 发布车体静态 TF | `vehicle.yaml` 生成的 URDF | `/tf_static` |
| `fastlio_mapping` | FAST-LIO 建图/里程计 | `/livox/lidar`, `/livox/imu` | `/Odometry`, `/cloud_registered` |
| `lio_interface_node` | 转换 LIO 里程计坐标 | `/Odometry`, TF | 标准化 odom 数据 |
| `sensor_scan_generation_node` | 发布 `/odom` 和 `/registered_scan` | FAST-LIO 点云/里程计 | `/odom`, `/registered_scan`, `odom -> base_footprint` |
| `pointcloud_to_laserscan_node` | 3D 点云切片成 2D scan | `/registered_scan` | `/scan` |
| `slam_toolbox` | 2D 建图 | `/scan`, `/tf` | `/map` |
| `rviz2` | 可视化 | 由 `use_rviz` 控制 | RViz 界面 |

建图时建议检查：

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic hz /cloud_registered
ros2 topic hz /registered_scan
ros2 topic hz /scan
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo odom base_footprint
```

### 2.3 保存 2D 地图

建图完成后，在另一个已加载工作空间的终端执行：

```bash
cd ~/Lidar_nav2_ws/scripts
source ../install/setup.bash
./save_map.sh site_a
```

`save_map.sh` 的位置参数是地图名：

| 参数 | 默认值 | 用处 |
| --- | --- | --- |
| `<map_name>` | `nav_test_4_27` | 保存到 `src/me_nav2_bringup/map/<map_name>.yaml/.pgm` |

例如：

```bash
./save_map.sh site_a
```

输出：

```text
src/me_nav2_bringup/map/site_a.yaml
src/me_nav2_bringup/map/site_a.pgm
```

FAST-LIO 的 PCD 保存由 `/map_save` 服务触发，默认路径由 `prior_pcd_file` 或 `map_name` 推导。项目里的 `scripts/save_pcd.sh` 会调用该服务；保存后确认目标文件最终存在：

```bash
ls src/me_nav2_bringup/pcd/site_a.pcd
```

### 2.4 实车导航

已知或近似开机位姿时，使用默认 small_gicp：

```bash
./nav2_real.sh map_name:=site_a relocalizer:=small_gicp
```

未知起点或需要全局恢复时，使用 KISS：

```bash
./nav2_real.sh map_name:=site_a relocalizer:=kiss
```

显式指定地图和 PCD：

```bash
./nav2_real.sh \
  map_name:=site_a \
  map_yaml_file:=/home/khp/Projects/AutoDrive/Lidar_nav2_ws/src/me_nav2_bringup/map/site_a.yaml \
  prior_pcd_file:=/home/khp/Projects/AutoDrive/Lidar_nav2_ws/src/me_nav2_bringup/pcd/site_a.pcd
```

`nav2_real.sh` 等价于：

```bash
cd ~/Lidar_nav2_ws
source install/setup.bash
ros2 launch me_nav2_bringup cpu_real_nav.launch.py ...
```

启动内容是在建图链路基础上增加：

| 节点/组件 | 作用 |
| --- | --- |
| `nav2_map_server` | 加载 2D `.yaml/.pgm` 地图 |
| `nav2_lifecycle_manager` | 自动激活 map server |
| `nav2_bringup/navigation_launch.py` | 启动 Nav2 planner/controller/BT navigator/behavior/velocity smoother |
| `small_gicp_relocalization_node` | 默认 3D 连续重定位，发布 `map -> odom` |
| `global_kiss_matcher_relocalization_exec` | KISS 全局初始化 + GICP 连续跟踪，发布 `map -> odom` |

导航时检查：

```bash
ros2 run tf2_ros tf2_echo map odom
ros2 topic hz /registered_scan
ros2 topic hz /scan
ros2 topic echo /cmd_vel
```

同一时间只能有一个重定位节点发布 `map -> odom`。不要同时启动 `small_gicp` 和 `kiss`。

## 3. Launch 参数参考

### 3.1 `cpu_real_mapping.launch.py`

查看真实参数：

```bash
ros2 launch me_nav2_bringup cpu_real_mapping.launch.py --show-args
```

参数表：

| 参数 | 默认值 | 用处 | 何时修改 | 示例 |
| --- | --- | --- | --- | --- |
| `vehicle_config_file` | `me_nav2_bringup/config/vehicle.yaml` | 车体尺寸、TF、话题、外参集中配置 | 使用另一套车体配置时 | `vehicle_config_file:=/abs/vehicle.yaml` |
| `map_name` | `nav_test_4_27` | 推导默认 PCD 文件名 | 每个场地/路线建图时修改 | `map_name:=site_a` |
| `fast_lio_config_file` | `fast_lio/config/mid360.yaml` | FAST-LIO 基础参数文件 | 需要独立 FAST-LIO 配置时 | `fast_lio_config_file:=/abs/mid360.yaml` |
| `slam_params_file` | `me_nav2_bringup/config/slam_toolbox_params.yaml` | SLAM Toolbox 建图参数 | 修改 2D SLAM 行为时 | `slam_params_file:=/abs/slam.yaml` |
| `pointcloud_to_laserscan_params_file` | `me_nav2_bringup/config/Pointcloud2d_3d.yaml` | 3D 点云切片基础参数 | 切片角度/距离配置变化时 | `pointcloud_to_laserscan_params_file:=/abs/scan.yaml` |
| `rviz_config_file` | `me_nav2_bringup/rviz/nav2.rviz` | RViz 配置 | 使用自定义可视化布局时 | `rviz_config_file:=/abs/view.rviz` |
| `use_rviz` | `false` | 是否启动 RViz | 调试时设为 `true` | `use_rviz:=true` |
| `prior_pcd_file` | `me_nav2_bringup/pcd/<map_name>.pcd` | FAST-LIO PCD 保存目标路径 | 需要保存到指定路径时 | `prior_pcd_file:=/abs/site_a.pcd` |

注意：`cpu_real_mapping.launch.py` 会用 `vehicle.yaml` 覆盖 FAST-LIO 的关键输入话题、`lidar_type`、外参和 PCD 保存路径，因此上车调外参优先改 `vehicle.yaml`。

### 3.2 `cpu_real_nav.launch.py`

查看真实参数：

```bash
ros2 launch me_nav2_bringup cpu_real_nav.launch.py --show-args
```

参数表：

| 参数 | 默认值 | 用处 | 何时修改 | 示例 |
| --- | --- | --- | --- | --- |
| `vehicle_config_file` | `me_nav2_bringup/config/vehicle.yaml` | 车体尺寸、TF、话题、外参集中配置 | 使用另一套车体配置时 | `vehicle_config_file:=/abs/vehicle.yaml` |
| `map_name` | `nav_test_4_27` | 同时推导 2D map 和 3D PCD | 换场地/地图时 | `map_name:=site_a` |
| `nav2_params_file` | `me_nav2_bringup/config/nav2_params.yaml` | Nav2 行为树、规划、控制、代价地图参数 | 调速度、代价地图、规划器时 | `nav2_params_file:=/abs/nav2.yaml` |
| `fast_lio_config_file` | `fast_lio/config/mid360.yaml` | FAST-LIO 基础参数文件 | 需要独立 FAST-LIO 配置时 | `fast_lio_config_file:=/abs/mid360.yaml` |
| `pointcloud_to_laserscan_params_file` | `me_nav2_bringup/config/Pointcloud2d_3d.yaml` | 3D 点云切片基础参数 | 调整 `/scan` 分辨率/距离时 | `pointcloud_to_laserscan_params_file:=/abs/scan.yaml` |
| `rviz_config_file` | `me_nav2_bringup/rviz/nav2.rviz` | RViz 配置 | 调试导航时 | `rviz_config_file:=/abs/view.rviz` |
| `use_rviz` | `false` | 是否启动 RViz | 调试时设为 `true` | `use_rviz:=true` |
| `relocalizer` | `small_gicp` | 选择 3D 重定位后端 | 未知起点设为 `kiss` | `relocalizer:=kiss` |
| `map_yaml_file` | `me_nav2_bringup/map/<map_name>.yaml` | Nav2 2D 地图路径 | 地图不按 `map_name` 存放时 | `map_yaml_file:=/abs/site_a.yaml` |
| `prior_pcd_file` | `me_nav2_bringup/pcd/<map_name>.pcd` | 3D 重定位先验 PCD | PCD 不按 `map_name` 存放时 | `prior_pcd_file:=/abs/site_a.pcd` |

`cpu_real_nav.launch.py` 会把 `vehicle.yaml` 中的 `footprint` 和 frame 名称重写到 Nav2 参数中，避免 `nav2_params.yaml` 和车体配置不一致。

## 4. 固定节点参数说明

### 4.1 MID360 驱动

`cpu_real_mapping.launch.py` 和 `cpu_real_nav.launch.py` 内部固定启动 `livox_ros_driver2_node`：

| 参数 | 固定值 | 用处 |
| --- | --- | --- |
| `xfer_format` | `1` | 输出 Livox CustomMsg，供 FAST-LIO `lidar_type=1` 使用 |
| `multi_topic` | `0` | 多雷达共用话题；单 MID360 保持默认 |
| `data_src` | `0` | 从真实 LiDAR 取数 |
| `publish_freq` | `10.0` | 驱动发布频率 |
| `output_data_type` | `0` | 驱动输出类型 |
| `frame_id` | 来自 `vehicle.yaml` 的 `lidar_frame` | LiDAR 坐标系 |
| `user_config_path` | `livox_ros_driver2/config/MID360_config.json` | MID360 网络配置 |
| `cmdline_input_bd_code` | `livox0000000001` | 广播码占位，实际连接主要看 MID360 配置 |

### 4.2 FAST-LIO 覆盖参数

launch 会在 `mid360.yaml` 基础上覆盖：

| 参数 | 来源 | 用处 |
| --- | --- | --- |
| `common.lid_topic` | `vehicle.lidar_topic` | 订阅 `/livox/lidar` |
| `common.imu_topic` | `vehicle.imu_topic` | 订阅 `/livox/imu` |
| `preprocess.lidar_type` | `vehicle.fast_lio.lidar_type` | MID360 CustomMsg 使用 `1` |
| `preprocess.scan_line` | `vehicle.fast_lio.scan_line` | MID360 默认 `4` |
| `preprocess.blind` | `vehicle.fast_lio.blind` | 近距离盲区 |
| `preprocess.timestamp_unit` | `vehicle.fast_lio.timestamp_unit` | Livox 时间戳单位 |
| `mapping.extrinsic_T/R` | `vehicle.fast_lio.extrinsic_T/R` | FAST-LIO IMU-LiDAR 外参 |
| `pcd_save.pcd_save_en` | `true` | 允许保存 PCD |
| `pcd_save.interval` | `-1` | 全部帧保存到一个 PCD |
| `map_file_path` | `prior_pcd_file` | PCD 输出路径 |

### 4.3 点云切片

`pointcloud_to_laserscan` 输入 `/registered_scan`，输出 `/scan`。关键参数来自 `Pointcloud2d_3d.yaml` 和 `vehicle.yaml`：

| 参数 | 默认值 | 用处 |
| --- | --- | --- |
| `target_frame` | `livox_frame` | 切片参考坐标系 |
| `min_height` | `0.2` | 丢弃低于该高度的点 |
| `max_height` | `1.0` | 丢弃高于该高度的点 |
| `angle_min/max` | `-pi` / `pi` | 生成 360 度 LaserScan |
| `angle_increment` | `0.0087` | 约 0.5 度角分辨率 |
| `range_min/max` | `0.05` / `70.0` | LaserScan 有效距离范围 |

如果代价地图里有车体残留或地面噪声，优先调 `min_height`；如果低矮障碍识别不到，降低 `min_height` 或提高点云质量。

### 4.4 重定位固定参数

`small_gicp` 和 `kiss` 都订阅 `/registered_scan`，加载 `prior_pcd_file`，发布 `map -> odom`。

`small_gicp` 固定参数：

| 参数 | 固定值 | 用处 |
| --- | --- | --- |
| `num_threads` | `4` | CPU 并行线程数 |
| `num_neighbors` | `10` | 协方差估计邻域数 |
| `global_leaf_size` | `0.25` | 先验地图降采样体素大小 |
| `registered_leaf_size` | `0.25` | 当前点云降采样体素大小 |
| `max_dist_sq` | `1.0` | 匹配最大距离平方 |
| `input_cloud_topic` | `/registered_scan` | 当前帧 3D 点云 |

`kiss` 在上述 GICP 参数基础上增加：

| 参数 | 固定值 | 用处 |
| --- | --- | --- |
| `voxel_resolution` | `0.25` | KISS 特征/体素分辨率 |
| `use_global_initialization` | `true` | 开启全局初始化 |
| `use_kiss_recovery` | `true` | 跟踪失败后允许 KISS 恢复 |
| `gicp_max_consecutive_failures` | `2` | 连续失败后进入恢复 |
| `recovery_min_points` | `1000` | 恢复所需最小点数 |
| `recovery_cooldown_sec` | `2.0` | 恢复冷却时间 |
| `verify_kiss_with_gicp` | `true` | 用 GICP 验证 KISS 结果 |
| `loop.num_inliers_threshold` | `3` | KISS 内点阈值 |
| `loop.overlap_threshold` | `80.0` | 重叠阈值 |

规则：

- `small_gicp` 适合已知或近似开机位姿。
- `kiss` 适合未知起点或丢定位恢复。
- 两者不能同时运行，否则会有两个 `map -> odom` 发布源。

## 5. 逐步启动/调试命令

### 5.1 建图链路逐步调试

优先使用一体化 launch：

```bash
ros2 launch me_nav2_bringup cpu_real_mapping.launch.py map_name:=site_a use_rviz:=true
```

如果需要拆节点定位问题，可按链路理解和分段检查：

```bash
ros2 launch livox_ros_driver2 fast_lio_msg_MID360_launch.py
ros2 launch fast_lio mapping.launch.py rviz:=false
ros2 launch lio_interface lio_interface_launch.py
ros2 launch sensor_scan_generation sensor_scan_generation_launch.py
ros2 launch me_nav2_bringup pointcloud_to_laserscan_launch.py
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=src/me_nav2_bringup/config/slam_toolbox_params.yaml
```

注意：分段启动使用的是各包默认 launch，不会自动读取 `vehicle.yaml` 的全部覆盖参数。实车部署建议使用 `cpu_real_mapping.launch.py`，分段启动只用于定位问题。

### 5.2 导航链路逐步调试

优先使用一体化 launch：

```bash
ros2 launch me_nav2_bringup cpu_real_nav.launch.py map_name:=site_a relocalizer:=small_gicp use_rviz:=true
```

单独验证重定位参数：

```bash
ros2 launch small_gicp_relocalization small_gicp_relocalization_launch.py \
  prior_pcd_file:=src/me_nav2_bringup/pcd/site_a.pcd
```

或：

```bash
ros2 launch global_relocalization_kiss_matcher global_kiss_matcher_relocalization_launch.py \
  prior_pcd_file:=src/me_nav2_bringup/pcd/site_a.pcd
```

注意：单独启动重定位时不要再启动 `cpu_real_nav.launch.py` 中的另一个重定位后端。

## 6. 验收检查

建图验收：

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic hz /cloud_registered
ros2 topic hz /registered_scan
ros2 topic hz /scan
ros2 topic hz /odom
```

导航验收：

```bash
ros2 run tf2_ros tf2_echo map odom
ros2 lifecycle get /controller_server
ros2 topic echo /cmd_vel
```

检查原则：

- `/cmd_vel` 输出平滑，底盘能正常订阅。
- `map -> odom` 只有一个发布源。
- RViz 中 `/map`、`/scan`、局部/全局 costmap 正常。
- 如果重定位失败，优先检查 `map_name`、`prior_pcd_file`、外参、初始位姿和 `/registered_scan` 点数。

实车启动和验收必须在实际 MID360、底盘、电源、急停和人工接管正常的环境中完成。
