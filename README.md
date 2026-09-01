# M20Pro SLAM 建图与原厂导航合同适配

本仓库提供一个面向 M20Pro 的 ROS 2 LiDAR-IMU 三维建图实现，并将
`m20_orignal` 的导航数据链适配到原厂可观测的全局/局部规划合同。建图部分对原厂
`slam_ddsnode` 做行为重建；导航部分接收原厂地图、里程计、目标和参数接口，使用
Hybrid A* + cubic spline、DWA + LinePlanner 的对应实现，保持原厂参数名和话题名。

> 重要边界：这是对闭源原厂算法的工程重建，不是原厂私有源码的 1:1 复制。建图
> 真机默认使用隔离的 `/m20_slam/*` 输出并关闭 TF，可与原厂定位链并联做静止
> 验收；只有显式使用 `--takeover-vendor-outputs` 才切换到原厂同名话题和 TF，且
> 启动前必须停掉原厂同名节点。导航适配器默认
> `navigation.enable_motion_output=false`，不会
> 因为启动节点而向机器人发运动指令，也不自动接管导航系统的 TF。

## 当前验证结果

官方 bag：`/home/lee/light_slam/data/m20_office_bag`

| 指标 | 原厂 NOS 基线 | 本实现最新结果 |
|---|---:|---:|
| LiDAR / IMU | 452 / 8301 | 452 / 8301 |
| 成功 LIO 更新 | 440 | 441 |
| 关键帧 | 49 | 52 |
| 关键帧轨迹长度 | 35.5007 m | 37.1787 m |
| 首尾误差 | 0.2689 m | 0.0463 m |

本次发布前复测产物位于本机忽略目录 `maps/review_release_20260831/`。轨迹长度
差异约 4.7%，首尾误差约 0.05 m。原厂基线来自 NOS 实机上的 ARM64/Foxy 闭源
`slam_ddsnode`；本仓库不能在 x86 主机上直接执行该二进制。

2026-09-01 对 M20 PRO V1.1.8.7 三板实机做了只读核验：`/IMU` 为
RELIABLE/volatile；NOS 的配置与历史 DDS 发现结果均指向 `/LIDAR/POINTS`，但本轮
ROS 2 图检查时该话题没有实时 publisher/sample；原生 DrDDS 预检能匹配 2 个
LiDAR publisher，但 5 秒内 `updated=no`，而 `/IMU` 为 `matched=1 updated=yes`。
两台雷达 `10.21.33.201/.202` 可 ping，`hsLidar` 也在监听 UDP 2361/2362，因此
当前验收边界是“网络、端点与进程存在，点云帧未到达”，不能据此宣称真机建图
已通过。详见
`docs/M20PRO_LIVE_ADAPTATION_20260901.md`。

## 算法组成

### 1. M20Pro 数据适配层

`src/m20_slam_navigation/src/ros/` 将 ROS 2 消息、原厂 DrDDS 数据和内部数据包
统一起来：

- `PointCloud2` 解析 `x/y/z/intensity/ring/timestamp` 字段；
- 绝对点时间用于扫描内运动补偿；
- LiDAR 扫描起止时间和 IMU 时间统一为单调时间戳；
- 实时 DrDDS 通过 root-only 网关转发到 Unix socket，避免原厂 Fast DDS 与 ROS 2
  进程在同一进程内冲突；
- bag 回放使用 ROS 2 订阅，LiDAR/IMU QoS 深度分别为 512/4096，避免注册线程较慢
 造成 DDS 覆盖未读样本。

### 2. ESKF / LIO 前端

主要实现位于 `src/m20_slam_navigation/src/lio/`：

- 23 维状态：姿态、位置、速度、陀螺仪偏置、加速度偏置和重力方向；
- S2 重力表示和 IMU 扫描端点传播；
- 基于每点时间戳的扫描内 deskew；
- 分层体素地图，使用局部平面法向和偏移查询；
- 迭代点到平面观测更新，残差由 `lio.lidar_cov` 加权；
- 有限值、法向、残差和对应数量门控，拒绝异常更新；
- 成功更新后回写姿态、速度、偏置和协方差信息。

这部分对应原厂可观测的 ESKF + point-to-plane 结构，但内部线性化顺序、协方差
传播细节和私有阈值仍无法从闭源二进制证明完全一致。

### 3. 关键帧与局部地图

关键帧由平移和旋转阈值触发：

- `lio.keyframe_distance: 0.8` m；
- `lio.keyframe_angle: 0.4` rad。

每个关键帧保存局部点云、时间戳和 LIO 姿态，并将点云插入增量体素地图。保存时
`trajectory.csv` 和 `.sessions/session_0/poses.txt` 使用关键帧语义；高频诊断轨迹
单独保存在 `lio_trajectory.csv`。

### 4. PGO 因子模型

后端使用 GTSAM/iSAM2 重建可观测的 Pose3 图：

- 首关键帧 prior factor；
- 相邻关键帧 odometry `BetweenFactor<Pose3>`；
- 可选 IMU gravity factor；
- 几何验证通过后的 loop `BetweenFactor<Pose3>`；
- 保留原厂参数的各向异性六维噪声顺序 `[rx, ry, rz, tx, ty, tz]`；
- 每轮优化结果回写到已保存关键帧，因此最终地图使用优化后的姿态。

原厂 session/segment 图的私有加入时机和权重未知，因此这里是行为模型而不是源码
复刻。

### 5. GHT / 几何闭环

`loop_closure.cpp` 中的几何约束分为三层：

1. ScanContext 描述子检索历史候选；
2. 使用 `pgo.segment_num` 做分段质心/半径哈希，枚举 sector shift，估计 yaw 和
   平移初值；
3. 使用有界多 yaw ICP，必要时使用 NDT 兜底，并按 fitness 与点云重叠率验收。

这不是原厂私有 GHT 哈希公式，而是可测试的重建约束。默认每 5 个关键帧进行一次
几何验证，以控制离线和实时计算量。

### 6. 终端闭环约束

保存地图前会检查首、末关键帧的几何重叠。对于确认回到起点的终端闭环：

- 保留 ICP 估计的相对旋转；
- 对相对平移使用首帧零平移先验；
- 再执行 iSAM2 优化并保存优化后的轨迹和地图。

该策略用于消除末端漂移，能把官方 bag 的首尾误差从约 1.3 m 降到约 0.20 m；它
是针对当前 M20Pro 回环场景的合理重建，并不代表已证明的原厂内部实现。

## 原厂导航合同适配

导航入口由 `navigation_system.launch.py` 加载
`config/native_navigation.yaml`。适配链路为：

```text
/GRID_MAP + /ODOM + /goal_pose 或 /GOAL_GLOBAL
    -> Hybrid A* + cubic spline
    -> /path_Astar、/global_path、/local_goal

/path_Astar + /NAV_POINTS + /ODOM 或 /MOTION_INFO
    -> DWA / LinePlanner
    -> /NAV_CMD、/PLANNER_STATUS
```

原厂参数快照和接口快照位于：

- `config/native_navigation.yaml`：可直接加载的导航节点配置；
- `config/native_global_planner.yaml`、`config/native_local_planner.yaml`：原厂全局/局部参数；
- `config/native_passable_area.yaml`、`config/native_pcl_pass_grid.yaml`：地形和点云过滤参数；
- `config/native_global_topics.yaml`、`config/native_lidar_params.yaml`、`config/native_body_params.yaml`：原厂接口和机体标定快照。

安装了原厂 DrDDS SDK 的 M20Pro 构建会启用原生 `drdds/msg/*`、`drdds/srv/*` 桥接；
x86 工作站没有该 SDK 时使用标准 ROS 消息回退，仅用于离线合同测试。原生 `/NAV_CMD`
只有在显式打开 `navigation.enable_motion_output` 后才会写出。

完整字段、类型、参数映射和验收边界见
`docs/M20PRO_NATIVE_NAVIGATION_ADAPTATION.md`。

## M20Pro 输入契约

| 接口 | 类型 | 典型频率 | 要求 |
|---|---|---:|---|
| `/LIDAR/POINTS` | `sensor_msgs/msg/PointCloud2` | 10 Hz | 前后雷达融合点云，通常 96 线 |
| `/IMU` | `sensor_msgs/msg/Imu` | 200 Hz | 原厂 IMU；当前固件可能为空 `frame_id` |

LiDAR 必须提供以下字段：

```text
x,y,z        float32
intensity    float32
ring         uint16
timestamp    float64   # 每点绝对时间，单位秒
```

如果没有 `timestamp`，只能退化为整帧时间，deskew 和点到平面精度都会下降；如果
没有 `ring`，仍可运行，但无法完整匹配原厂点云语义。

实时 M20Pro 默认契约：

```text
ROS_DOMAIN_ID=0
DDS prefix=rt
network=eth0/eth1
shared memory=true
```

## 输出接口

真机默认输出采用隔离命名且不广播 TF，不会接管原厂定位/导航数据链：

| 默认输出 | 类型 |
|---|---|
| `/m20_slam/odom` | `nav_msgs/msg/Odometry` |
| `/m20_slam/aligned_points` | `sensor_msgs/msg/PointCloud2` |
| `/m20_slam/cloud_registered_body` | `sensor_msgs/msg/PointCloud2` |
| `/m20_slam/depth_points` | `sensor_msgs/msg/PointCloud2` |
| `/m20_slam/depth_image` | `sensor_msgs/msg/Image` |
| `/m20_slam/accumulated_points_map` | `sensor_msgs/msg/PointCloud2` |
| `/m20_slam/path` | `nav_msgs/msg/Path` |
| `/m20_slam/save_map` | `std_srvs/srv/Trigger` |

显式加入 `--takeover-vendor-outputs` 后才启用以下原厂合同；此模式下本节点和原厂
`slam_ddsnode` 不能同时运行：

| 输出 | 类型 |
|---|---|
| `/SLAM_ODOM` | `nav_msgs/msg/Odometry` |
| `/SLAM_ALIGNED_POINTS` | `sensor_msgs/msg/PointCloud2` |
| `/SLAM_CLOUD_REGISTERED_BODY` | `sensor_msgs/msg/PointCloud2` |
| `/DEPTH_POINTS` | `sensor_msgs/msg/PointCloud2` |
| `/DEPTH_IMAGE` | `sensor_msgs/msg/Image` |
| `/SLAM_ACCUMULATED_POINTS_MAP` | `sensor_msgs/msg/PointCloud2` |
| `/path` | `nav_msgs/msg/Path` |
| `/m20_slam/save_map` | `std_srvs/srv/Trigger` |
| `camera_init -> base_link` | 动态 TF |

地图目录通常包含：

```text
full_cloud.pcd
mapping_summary.txt
trajectory.csv              # 优化后的关键帧轨迹
lio_trajectory.csv          # 高频 LIO 诊断轨迹
.sessions/session_0/poses.txt
.optimizers/optimizer_0/loops.txt
```

## 一键运行

### M20Pro 实时建图

在 NOS 验证工作区运行（DrDDS 网关需要 root 权限）：

```bash
cd ~/m20_orignal
./src/m20_slam_navigation/scripts/start_mapping.sh --map-name site_a
```

脚本会完成 ROS 环境加载、重复进程保护、生命周期节点启动、RViz（可选）和 Ctrl+C
自动保存。实时输入 helper 直接链接机器人已有的 `libdrdds.so.1`，不创建第二个 DDS
实现，也不重发 DDS 话题；它只用本地 socket 将消息交给算法进程。这个进程隔离是
必需的：实机已证明把 ROS 2 Foxy 的 FastDDS 与厂商 FastDDS 2.14 加载到同一进程会
在 Participant 公告阶段崩溃。ROS 2 订阅仅用于官方 bag 回放。

默认命令仅发布隔离话题且不发布 TF。原厂同名输出仅在完成单独验收并停止原厂
`slam_ddsnode` 后显式启用：

```bash
./src/m20_slam_navigation/scripts/start_mapping.sh \
  --map-name site_a --takeover-vendor-outputs
```

### 官方 bag 回放

```bash
cd ~/m20_orignal
./src/m20_slam_navigation/scripts/start_mapping.sh \
  --bag /path/to/m20_office_bag \
  --map-name office_test \
  --no-rviz \
  --skip-build
```

### 复用原厂 `drmap` 建图结果

实机原始 `/LIDAR/POINTS` 由原厂私有数据链交给 `slam_ddsnode`，外部 ROS 2 和独立
DrDDS subscriber 即使端点匹配也收不到样本。推荐先按原厂流程完成建图，再让本仓库
直接读取原厂 `poses.txt` 与 `lidar_cloud/*.pcd`，仅运行 ScanContext/GHT/ICP 和
GTSAM/iSAM2 后端：

```bash
cd ~/m20_orignal
./src/m20_slam_navigation/scripts/start_mapping.sh \
  --vendor-map /var/opt/robot/data/maps/<原厂地图目录> \
  --map-name site_a_backend \
  --skip-build
```

输出目录包含 `optimized_poses.txt`、`optimized_full_cloud.pcd`、`loops.txt` 和
`adapter_summary.txt`。该模式不启动 DDS、ROS 节点、TF、导航或运动控制，也不修改
原厂地图和 `active` 软链接。

### 手工构建

```bash
source /opt/ros/humble/setup.bash
cd ~/m20_orignal
colcon build --symlink-install --packages-select m20_slam_navigation \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_EXPERIMENTAL_NAVIGATION=OFF
source install/setup.bash
```

验证：

```bash
ctest --test-dir build/m20_slam_navigation --output-on-failure
```

当前测试为 7/7 测试目标、37 个测试通过，覆盖原厂导航话题/参数、运动原语、
Hybrid A*、LinePlanner、地形回退和定位 ESKF 合同。

### 已有地图的定位与导航联调

```bash
ros2 launch m20_slam_navigation m20_full_system.launch.py \
  map_path:=/absolute/path/to/full_cloud.pcd \
  use_rviz:=true \
  enable_motion_output:=false
```

该完整启动文件运行“定位 + 导航”，不同时启动建图。建图和定位不能共同拥有
`camera_init` 位姿链；真实运动输出必须经过单独实机验收后显式开启。

## 关键参数

配置文件：`src/m20_slam_navigation/config/m20_mapping.yaml`

常用参数包括：

```text
lio.voxel_size                  0.16
lio.leaf_size                   0.15
lio.leaf_size_body              0.05
lio.skip_num                    5
lio.max_iteration               3
lio.esti_plane_threshold        0.1
lio.lidar_cov                   0.001
lio.keyframe_distance           0.8
lio.keyframe_angle              0.4
pgo.enable_imu_gravity          true
backend.enable_loop_closure     true
backend.loop_detection_stride   5
backend.loop_min_submap_overlap 0.65
```

## 部署边界与安全

- 默认隔离输出允许并联做 mapping-only 数据链验收；只有启用
  `--takeover-vendor-outputs` 时才要求先停止原厂 `slam_ddsnode`。
- `start_mapping.sh` 仍只做 mapping-only 验证，不自动启动导航、Nav2、`cmd_vel`、
  motion bridge，也不重启原厂 AOS/NOS 服务；导航需要单独运行
  `navigation_system.launch.py`。
- 导航适配器默认只计算和发布规划/诊断结果；必须单独审核后才能打开运动输出，并
  在同一台 M20Pro 上确认原厂运动控制器的消费话题与 DDS QoS。
- 官方 bag 通过不等于真实设备已验收；真实部署还必须检查时间同步、外参、TF、物理
  尺度、长时间运行、地图质量和下游消费接口。
- 实时替换原厂前，建议先完成同一 M20Pro 设备上的重复 mapping-only 回放和现场建图
  验证。

更多对齐记录见：

- `docs/M20PRO_DRIFT_LOOP_ALIGNMENT_20260828.md`
- `docs/REVIEW_AND_M20PRO_ADAPTATION.md`
- `docs/M20PRO_SLAM_ADAPTATION_HANDOFF_20260827.md`
