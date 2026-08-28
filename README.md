# M20Pro SLAM 建图重建算法

本仓库提供一个面向 M20Pro 的 ROS 2 LiDAR-IMU 三维建图实现。它对原厂
`slam_ddsnode` 的可观测输入、输出和参数进行行为重建，目标是让官方 ROS bag
和 M20Pro 实时数据能够进入一条可复现、可测试、与原厂轨迹量纲接近的建图链路。

> 重要边界：这是对闭源原厂算法的工程重建，不是原厂私有源码的 1:1 复制。当前
> 支持范围是隔离三维建图；不会发布原厂 `/ODOM`、`/map`、`/cmd_vel`，也不接管
> 导航系统的 `map -> odom -> base_link` TF。

## 当前验证结果

官方 bag：`/home/lee/light_slam/data/m20_office_bag`

| 指标 | 原厂 NOS 基线 | 本实现最新结果 |
|---|---:|---:|
| LiDAR / IMU | 452 / 8301 | 452 / 8301 |
| 成功 LIO 更新 | 440 | 441 |
| 关键帧 | 49 | 52 |
| 关键帧轨迹长度 | 35.5007 m | 36.8157 m |
| 首尾误差 | 0.2689 m | 0.1986 m |

本实现产物位于 `maps/ab_repro_terminal_zero_20260828-20260828-155443/`。轨迹长度
差异约 3.7%，首尾误差约 0.20 m。原厂基线来自 NOS 实机上的 ARM64/Foxy 闭源
`slam_ddsnode`；本仓库不能在 x86 主机上直接执行该二进制。

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

为避免与原厂节点争抢名称，默认使用 `/m20_slam/*` 隔离命名空间：

| 输出 | 类型 |
|---|---|
| `/m20_slam/SLAM_ODOM` | `nav_msgs/msg/Odometry` |
| `/m20_slam/SLAM_ALIGNED_POINTS` | `sensor_msgs/msg/PointCloud2` |
| `/m20_slam/SLAM_CLOUD_REGISTERED_BODY` | `sensor_msgs/msg/PointCloud2` |
| `/m20_slam/DEPTH_POINTS` | `sensor_msgs/msg/PointCloud2` |
| `/m20_slam/SLAM_ACCUMULATED_POINTS_MAP` | `sensor_msgs/msg/PointCloud2` |
| `/m20_slam/path` | `nav_msgs/msg/Path` |
| `/m20_slam/save_map` | `std_srvs/srv/Trigger` |
| `m20_slam_map -> m20_slam_lidar` | isolated TF |

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

脚本会完成 ROS 环境加载、重复进程保护、DrDDS socket 网关、生命周期节点启动、
RViz（可选）和 Ctrl+C 自动保存。

### 官方 bag 回放

```bash
cd ~/m20_orignal
./src/m20_slam_navigation/scripts/start_mapping.sh \
  --bag /path/to/m20_office_bag \
  --map-name office_test \
  --no-rviz \
  --skip-build
```

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

当前测试为 5/5 通过，覆盖消息适配、传感器线协议、ESKF、输出契约以及 GHT/PGO
几何模型。

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

- 原厂 `slam_ddsnode` 和本实现不能同时占用同一套输入/输出资源；A/B 测试前先停止
  原厂建图服务。
- 当前流程只做 mapping-only 验证，不启动 Nav2、`cmd_vel`、motion bridge，也不重启
  原厂 AOS/NOS 服务。
- 官方 bag 通过不等于真实设备已验收；真实部署还必须检查时间同步、外参、TF、物理
  尺度、长时间运行、地图质量和下游消费接口。
- 实时替换原厂前，建议先完成同一 M20Pro 设备上的重复 mapping-only 回放和现场建图
  验证。

更多对齐记录见：

- `docs/M20PRO_DRIFT_LOOP_ALIGNMENT_20260828.md`
- `docs/REVIEW_AND_M20PRO_ADAPTATION.md`
- `docs/M20PRO_SLAM_ADAPTATION_HANDOFF_20260827.md`
