# M20Pro SLAM 迁移适配与实机验收交接文档

更新时间：2026-08-27
适用工作区：`/home/lee/m20_orignal`
当前分支：`codex/m20pro-direct-mapping`
当前基础提交：`0341fc9`（工作区仍有未提交修改，不能仅凭该 SHA 还原现状）

## 1. 文档目的

本文记录目前将 M20 原生算法及后续其他 LiDAR-Inertial SLAM 算法迁移到 M20Pro 真机时，已经确认的网络、主机、传感器、DrDDS、ROS 2、构建部署、代码改动和实机验收结果。

本文特别区分以下三层结论：

1. **输入链路通过**：能够收到原厂融合双雷达点云和 IMU。
2. **静态建图通过**：算法能初始化、产生关键帧并保存 PCD。
3. **动态建图通过**：机器狗真实运动时，轨迹、尺度、外参、漂移和地图几何正确。

截至 2026-08-27，第 1、2 层已经通过；第 3 层没有通过。最近一次覆盖约 300 平方米的实机运行中，算法估计的平移范围只有约 0.37 m，说明当前算法没有正确恢复机器狗平移。

## 2. 保护边界

本次适配遵循以下边界：

- 不修改原厂 `/opt/robot` 下的驱动、SLAM 二进制、配置和服务。
- 不替换或重新实现原厂前后雷达融合。
- 不覆盖原厂话题和 TF，不抢占 `/ODOM`、`map -> odom` 等原厂定位链路。
- 第三方算法使用独立输出：`/m20_slam/*` 和 `m20_slam_map`。
- 原厂算法只用于短时 A/B 对照；测试完成必须执行 `sudo drmap stop_mapping`。
- 动态测试由人工遥控，低速、空旷、有人随时急停；当前适配不包含自主运动验收。

## 3. 网络与主机

### 3.1 Wi-Fi 和网络拓扑

开发电脑连接：

```text
SSID: ZKYD-GUEST
开发电脑示例地址: 192.168.100.106/23
```

当前可用拓扑：

```text
开发电脑
  -> SSH 跳板机 192.168.101.36
      -> NOS 10.21.31.106
      -> GOS 10.21.31.104
```

历史上本机配置过：

```text
10.21.31.0/24 via 192.168.101.36
```

机器狗断电时的典型现象：

```text
ssh: connect to host 192.168.101.36 port 22: No route to host
```

跳板机恢复后，NOS/GOS 即使不响应 ICMP，也可能仍可通过 SSH ProxyJump 访问，因此应以 SSH 登录结果为准。

### 3.2 主机信息

| 主机 | 地址 | 用户 | 用途 | 已确认环境 |
|---|---|---|---|---|
| NOS | `10.21.31.106` | `user` | 原厂导航/建图主机，第三方实机验证 | Ubuntu 20.04、ROS 2 Foxy、ARM64 |
| GOS | `10.21.31.104` | `user` | ARM64/Foxy 构建主机 | Ubuntu 20.04、ROS 2 Foxy、ARM64 |
| 跳板机 | `192.168.101.36` | `user` | 从 ZKYD-GUEST 访问机器狗内网 | SSH ProxyJump |

当前测试密码为英文单引号：`'`。该密码只用于现场设备，不应提交到公共仓库或公开文档。

### 3.3 SSH 命令

登录 NOS：

```bash
ssh -J user@192.168.101.36 user@10.21.31.106
```

登录 GOS：

```bash
ssh -J user@192.168.101.36 user@10.21.31.104
```

基础检查：

```bash
systemctl is-active \
  localization.service \
  planner.service \
  global_planner.service \
  mapping.service
```

非建图状态的预期结果：

```text
active
active
active
inactive
```

## 4. 原厂建图基线

### 4.1 原厂命令和文件

原厂启动命令：

```bash
sudo drmap mapping
```

用于不激活新地图、不启动 RViz 的短时 A/B 对照：

```bash
sudo drmap mapping -b -s -n codex_align_probe
```

停止原厂建图：

```bash
sudo drmap stop_mapping
```

主要原厂文件：

```text
/opt/robot/share/slam/conf/params.yaml
/opt/robot/share/slam/scripts/mapping.sh
/opt/robot/share/slam/scripts/mapping_stop.sh
/opt/robot/share/slam/scripts/start_dds.sh
/opt/robot/share/slam/bin/slam_ddsnode
/opt/robot/share/slam/include/dr_lio/
```

原厂日志已经确认：

```text
Lidar Imu Odometry
LIO version: 3.4.0
LIO command server started.
Save frame: 0: ...
```

### 4.2 原厂实际输入

原厂 `params.yaml` 和二进制模板符号共同确认：

| 数据 | 话题 | 类型 | 传输 |
|---|---|---|---|
| 前后双雷达融合点云 | `/LIDAR/POINTS` | `sensor_msgs::msg::PointCloud2` | 原厂 DrDDS |
| IMU | `/IMU` | `sensor_msgs::msg::Imu` | 原厂 DrDDS |

参数文件注释中曾提到 `/IMU_DATA`，但当前生效配置值是 `/IMU`，二进制也实例化了 `DrDDSChannel<sensor_msgs::msg::ImuPubSubType>`。后续必须以运行时当前固件为准，不要仅依据旧注释。

原厂 DrDDS 初始化和点云通道已通过反汇编与实机探针确认：

```cpp
DrDDSManager::Init(0, "eth0/eth1");

DrDDSChannel<sensor_msgs::msg::PointCloud2PubSubType>(
    callback,
    "/LIDAR/POINTS",
    0,
    true,
    "rt");
```

对应接口合同：

```text
domain_id:    0
network_name: eth0/eth1
topic_prefix: rt
use_shm:      true
```

### 4.3 root/共享内存要求

同一个 DrDDS 点云探针的实机结果：

普通用户：

```text
matched=2 updated=no received=0
```

root：

```text
matched=2 updated=yes received=119
```

因此已确认 `/LIDAR/POINTS` 的原厂 DrDDS 数据位于 root 可访问的共享内存链路。原厂 `slam_ddsnode` 也由 root/systemd 运行。

工程约束：需要直接读取原厂 DrDDS 的接收进程必须以 root 运行；第三方 SLAM 主体应保持普通用户运行。

### 4.4 双雷达结论

`/LIDAR/POINTS` 是原厂已经完成前后双雷达外参转换和融合后的点云。本项目不再分别订阅前、后雷达，也不重复融合。

DrDDS 接收器实机显示：

```text
matched=2
updated=yes
约 10 Hz
```

`matched=2` 与原厂双雷达数据链路一致，但它表示 DDS 匹配数量，不应单独作为几何融合正确性的证明。融合结果本身由原厂发布者负责。

### 4.5 点云数据合同

当前适配实际接受的字段：

```text
x          float32
y          float32
z          float32
intensity  float32
ring       uint16
timestamp  float64
```

重要语义：

- `timestamp` 是逐点绝对秒时间，不是强度，也不是简单的相对毫秒。
- 点云约 10 Hz。
- 适配器保留完整原始 `PointCloud2` header、frame、fields、point_step、row_step 和 data。
- M20 算法内部将 `timestamp` 转成每点相对扫描起始时间用于去畸变。
- 小于等于 20 ms 的小幅点云时间回退会被夹紧为单调递增；更大的回退应拒绝。
- 实机发现过 0.001～0.110 ms 的微小回退，当前属于可控范围。

### 4.6 原厂可见 LIO 参数

来源：`/opt/robot/share/slam/conf/params.yaml`。

```yaml
node:
  input_lidar_topic: /LIDAR/POINTS
  input_imu_topic: /IMU
  output_odom_topic: /SLAM_ODOM
  output_aligned_cloud_topic: /SLAM_ALIGNED_POINTS
  output_voxel_cloud_topic: /DEPTH_POINTS
  output_accumulated_map_cloud_topic: /SLAM_ACCUMULATED_POINTS_MAP
  lidar_type: 1
  lidar_use_system_time: false
  imu_use_system_time: false

lio:
  acc_cov: 0.5
  gyr_cov: 0.5
  b_acc_cov: 0.001
  b_gyr_cov: 0.001
  extrinsic_est_en: false
  extrinsic_B_I: identity
  extrinsic_B_L: identity
  max_iteration: 3
  enable_downsample: true
  leaf_size: 0.15
  leaf_size_body: 0.05
  skip_num: 5
  esti_plane_threshold: 0.1
  init_time: 0.1
  lidar_cov: 0.001
  deepest_level: 2
  plane_level: 2
  top_level: 1
```

原厂占据栅格参数：

```yaml
occ_grid_2d:
  min_height: -0.2
  max_height: 0.4
  resolution: 0.1
  min_range: 0.2
  max_range: 30.0
```

原厂 PGO 中可见：

```yaml
pgo:
  distance_threshold_factor: 0.03
  segment_num: 15
  matching_error_threshold: 0.16
  inlier_fraction_threshold: 0.95
  max_search_distance: 8.0
  keyframe_time: 60.0
```

必须注意：原厂 LIO 3.4.0 是 ESKF + IMU 去畸变 + 点到平面观测模型，并非当前 M20 复现代码的 VGICP。只复制数值参数不能让两种不同算法获得相同行为。

原厂公开头文件还明确提示：

```text
传入点云时确认是否已转换到 imu 坐标系下
```

因此融合点云 frame、IMU frame 和 identity 外参为何成立，是下一阶段必须真机确认的重点。

## 5. 当前 M20 适配实现

### 5.1 工作区和构建状态

```text
本地工作区: /home/lee/m20_orignal
分支: codex/m20pro-direct-mapping
GOS 构建区: /home/user/codex_m20_drdds_ws
NOS 验证区: /home/user/codex_m20_drdds_validation
```

已完成：

- 本地 Humble Release 构建通过。
- GOS ARM64/Foxy Release 构建通过。
- focused tests 曾通过：6 tests、0 errors、0 failures。
- 修复 ARM64/PCL 1.10 智能指针兼容问题。
- 绕过 GOS 损坏的 GTSAM CMake metadata。

### 5.2 DrDDS/Fast DDS ABI 隔离

ROS 2 Foxy 自带 Fast DDS 与原厂 DrDDS 使用的 Fast DDS 2.14 不能安全加载进同一个进程，否则存在 ABI/符号冲突。

当前架构：

```text
原厂前后双雷达融合点云 /LIDAR/POINTS
  -> root + libdrdds + Fast DDS 2.14 + SHM
  -> m20_drdds_receiver（独立进程）
  -> Unix Domain Socket（完整 PointCloud2 wire format）
  -> 普通用户 slam_node
  -> M20 LIO/VGICP
```

依赖验收：

```text
m20_drdds_receiver:
  libdrdds.so.1
  libfastrtps.so.2.14
  libfastcdr.so.2

slam_node:
  不加载 libdrdds.so.1
  不加载 libfastrtps.so.2.14
```

DrDDS CMake 必须优先使用：

```text
/usr/local/lib/cmake/drdds
```

不要使用：

```text
/opt/ros/foxy/share/drdds
```

后者是另一个同名 ROS 包，没有项目需要的 `drdds::drdds` target。

### 5.3 当前话题和数据对齐程度

| 输入 | 原厂算法 | 当前 M20 | 状态 |
|---|---|---|---|
| 双雷达融合点云 | DrDDS `/LIDAR/POINTS` | DrDDS `/LIDAR/POINTS` 经 root receiver + socket | 已同源对齐 |
| 点云类型 | `sensor_msgs::msg::PointCloud2` | 保留完整 `PointCloud2` 后适配为 `PointXYZI + point_time_offsets` | 已对齐 |
| IMU | DrDDS `/IMU` | DrDDS `/IMU` 经 root gateway + 独立 socket | 已同源对齐 |
| IMU 类型 | `sensor_msgs::msg::Imu` | 完整 Imu wire contract 后适配为算法 accel/gyro | 已对齐 |
| 点云频率 | 约 10 Hz | 约 10 Hz | 已对齐 |
| IMU 频率 | 约 200 Hz | 约 200 Hz | 频率对齐 |
| 外参 | 原厂配置 identity，且提示点云应已在 IMU系 | 当前默认 identity | 数值相同，但语义未验收 |

2026-08-27 已完成同源输入扩展。真机静态探针确认：cloud `matched=2`、约 10 Hz；IMU `matched=1`、约 200 Hz；点云 `frame_id=lidar_link`，IMU `frame_id` 为空，加速度模长约 9.78～9.81 m/s²。下一优先级转为外参/轴向动态验收和 VGICP 实时性、动态平移恢复。

### 5.4 主要新增文件

```text
src/m20_slam_navigation/include/m20_slam_navigation/ros/
  drdds_pointcloud_source.hpp
  drdds_imu_source.hpp
  m20_cloud_adapter.hpp
  pointcloud_wire.hpp
  socket_imu_source.hpp
  socket_pointcloud_source.hpp

src/m20_slam_navigation/src/ros/
  drdds_pointcloud_source.cpp（点云和 IMU 共用同一个 DrDDSManager）
  drdds_receiver_main.cpp
  pointcloud_wire.cpp
  socket_pointcloud_source.cpp

src/m20_slam_navigation/config/m20_mapping.yaml
src/m20_slam_navigation/launch/slam_system.launch.py
src/m20_slam_navigation/scripts/start_mapping.sh
src/m20_slam_navigation/rviz/slam_view.rviz
src/m20_slam_navigation/test/test_m20_cloud_adapter.cpp
src/m20_slam_navigation/test/test_sensor_wire.cpp
```

主要修改还包括：

- `slam_node.cpp`
- LIO、IMU 预积分和去畸变
- 体素地图和 VGICP
- 后端图优化
- CMake/package 配置
- 生命周期自动 configure/activate
- 输入统计、重复实例保护和地图保存

### 5.5 当前输出

为避免与原厂导航冲突，使用：

```text
/m20_slam/odom
/m20_slam/path
/m20_slam/map_cloud
TF: m20_slam_map -> m20_slam_lidar
```

地图文件：

```text
full_cloud.pcd
mapping_summary.txt
trajectory.csv
```

当前 `full_cloud.pcd` 保存的是体素地图质心，不是每一帧原始点云的完整累积。因此文件点数会明显少于输入原始点数。地图稀疏问题一部分来自保存表示，另一部分来自动态位姿没有正确展开场景。

### 5.6 地图保存修复

旧实现保存时会暂停接收并等待 LiDAR 队列完全排空。真机算力下输入约 10 Hz，处理可能追不上输入，导致保存最长等待 30 秒甚至在 launch 退出期限内被 SIGTERM。

当前修复：

- 暂停接收新点云；
- 利用 `VoxelMap::getAllVoxels()` 已有共享锁生成线程安全快照；
- 不再等待整个历史输入队列排空；
- 每 10 秒 checkpoint 保存；
- 正常退出时再次保存。

该修改只改变地图快照时机，没有改变点云、IMU、外参或 VGICP计算。

## 6. 实机验收证据

### 6.1 root DrDDS 接收

典型日志：

```text
DrDDS fused-cloud receiver started: topic=/LIDAR/POINTS domain=0 prefix=rt
DrDDS cloud status: matched=2 updated=yes received=...
```

`forwarded=0` 表示还没有 socket 客户端连接；`slam_node` 连接后 forwarded 持续增长属于正常现象。

### 6.2 静态完整验收

已完成一次约 22 秒静态运行：

```text
accepted_clouds=185
accepted_imus=3378
keyframes=1
map_points=3141
trajectory.csv=26 行
```

文件：

```text
/home/user/codex_m20_drdds_validation/maps/acceptance/full_cloud.pcd
/home/user/codex_m20_drdds_validation/maps/acceptance/mapping_summary.txt
/home/user/codex_m20_drdds_validation/maps/acceptance/trajectory.csv
```

PCD 头：

```text
FIELDS x y z intensity
WIDTH 3141
POINTS 3141
DATA binary
```

结论：原厂双雷达融合点云和机器狗 IMU 能进入当前 M20 算法，算法能够初始化、生成地图并落盘。

### 6.3 约 300 平方米动态实测

NOS 地图目录：

```text
/home/user/codex_m20_drdds_validation/maps/round_20260827_091047/
```

已下载到开发电脑：

```text
/home/lee/m20_maps/round_20260827_091047/
```

最终统计：

```text
accepted_clouds=3294
accepted_imus=65925
keyframes=5
map_points=5916
trajectory poses=444
```

估计轨迹范围：

```text
x_range=0.366 m
y_range=0.097 m
z_range=0.024 m
```

PCD：

```text
文件大小: 93 KB
点数: 5916
SHA256: a7c24c9bdaefc27d63e15ae42f6dadfdebeb93e8fe17b1d1b54731384c7eacee
```

结论：输入和保存链路正常，但动态建图失败。机器狗真实覆盖接近 300 平方米，而算法只估计出约 0.37 m 平移，导致局部点云不断叠加、地图稀疏并出现漂移。不能将此 PCD 用于后续定位或导航验收。

### 6.4 同源输入对齐后的静态验收

2026-08-27 在 NOS 隔离目录完成最终约 10 秒静态验收：

```text
地图目录: /home/user/codex_m20_drdds_validation/maps/alignment_static_final_20260827/
cloud matched=2，约 10 Hz
imu matched=1，约 200 Hz
lidar_frame=lidar_link
imu_frame=<empty>
accepted_clouds=98
accepted_imus=1894
processed_clouds=31
dropped_clouds=67
dropped_imus=0
keyframes=1
map_points=2658
point_stride=5
downsample_leaf_size=0.15
init_time=0.1
accel_bias=0.000490874,-0.000068754,-0.0257251
gyro_bias=0.00121235,0.000253126,0.00745822
```

本轮没有出现 VGICP 拒绝日志，地图正常保存为 42 KB PCD。处理速率由上一轮约 1 Hz 提升到约 2.8 Hz，有界队列保持在 2～3 帧，避免继续处理十几秒前的旧点云；但 10 Hz 输入中仍主动丢弃了约 68% 的点云。因此“完全同源输入、初始化和低延迟队列”已通过静态验收，“NOS 上满帧实时 VGICP”和“动态平移正确”仍未通过。

## 7. 当前问题分析

### 7.1 已排除

- 不是没有收到双雷达点云：Cloud 约 10 Hz 持续增长。
- 不是没有收到 IMU：IMU 约 200 Hz 持续增长。
- 不是 DrDDS 没有匹配：`matched=2 updated=yes`。
- 不是 PCD 文件损坏：PCL 成功读取 5916 个点和 `x y z intensity` 字段。
- 不是保存服务完全失效：每 10 秒均看到 `Saved map:`。

### 7.2 高优先级疑点

以下仍需实验确认，不能写成已证实根因：

1. 同源探针显示点云 `frame_id=lidar_link`、IMU `frame_id` 为空；原厂公开代码要求输入点云已转换到 IMU 坐标系，因此 identity 外参的语义仍未验收。
2. 当前 VGICP 与原厂 ESKF 点到平面算法不同。原厂参数不能直接映射为 VGICP 的 `correspondence_radius`、收敛阈值和 Hessian 判据。
3. 当前算法保存体素质心，视觉上天然比原始累计点云稀疏；但这不能解释 300 平方米被压缩到约 0.37 m，位姿估计仍是主要问题。
4. 首次同源静态验收中，154 帧点云只处理出 26 个轨迹姿态，队列曾积压约 130 帧。已加入 `skip_num=5` 点抽样和有界低延迟队列，但必须重新量化实时性。
5. 需要持续记录每次 VGICP 的平移、旋转、correspondences、fitness、迭代数、更新量和处理耗时，并与原厂相同路线对比。

## 8. 当前构建和部署流程

### 8.1 本地检查

```bash
cd /home/lee/m20_orignal
git status -sb
git diff --check
```

### 8.2 同步到 GOS

```bash
rsync -a --exclude __pycache__ \
  -e "ssh -J user@192.168.101.36" \
  /home/lee/m20_orignal/src/m20_slam_navigation/ \
  user@10.21.31.104:/home/user/codex_m20_drdds_ws/src/m20_slam_navigation/
```

### 8.3 GOS ARM64/Foxy Release 构建

```bash
ssh -J user@192.168.101.36 user@10.21.31.104 \
  "cd /home/user/codex_m20_drdds_ws && \
   source /opt/ros/foxy/setup.bash && \
   colcon build --packages-select m20_slam_navigation \
     --cmake-force-configure \
     --cmake-args -DCMAKE_BUILD_TYPE=Release"
```

### 8.4 部署到 NOS 隔离目录

先从 GOS 下载到本机临时目录：

```bash
mkdir -p /tmp/codex_m20_drdds_install
rsync -a -e "ssh -J user@192.168.101.36" \
  user@10.21.31.104:/home/user/codex_m20_drdds_ws/install/ \
  /tmp/codex_m20_drdds_install/
```

再上传到 NOS：

```bash
ssh -J user@192.168.101.36 user@10.21.31.106 \
  "mkdir -p /home/user/codex_m20_drdds_validation/install \
            /home/user/codex_m20_drdds_validation/maps"

rsync -a -e "ssh -J user@192.168.101.36" \
  /tmp/codex_m20_drdds_install/ \
  user@10.21.31.106:/home/user/codex_m20_drdds_validation/install/
```

## 9. 当前手动运行流程

以下是已经实机验证的“两终端”同源输入运行方式。

### 9.1 终端一：root DrDDS 传感器网关

```bash
cd /home/user/codex_m20_drdds_validation
source /opt/ros/foxy/setup.bash
source install/setup.bash

sudo env LD_LIBRARY_PATH="$LD_LIBRARY_PATH" \
  install/m20_slam_navigation/lib/m20_slam_navigation/m20_drdds_receiver \
  --lidar-topic /LIDAR/POINTS \
  --imu-topic /IMU \
  --domain 0 \
  --prefix rt \
  --network eth0/eth1 \
  --lidar-socket /tmp/m20_drdds_lidar.sock \
  --imu-socket /tmp/m20_drdds_imu.sock \
  --shm
```

### 9.2 终端二：M20 建图

```bash
cd /home/user/codex_m20_drdds_validation
source /opt/ros/foxy/setup.bash
unset FASTRTPS_DEFAULT_PROFILES_FILE
source install/setup.bash

MAP_DIR="/home/user/codex_m20_drdds_validation/maps/round_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$MAP_DIR"

ros2 launch m20_slam_navigation slam_system.launch.py \
  use_rviz:=false \
  lidar_transport:=drdds \
  drdds_socket_path:=/tmp/m20_drdds_lidar.sock \
  imu_transport:=drdds \
  drdds_imu_socket_path:=/tmp/m20_drdds_imu.sock \
  map_save_path:="$MAP_DIR/full_cloud.pcd"
```

正常输入日志：

```text
M20 mapping is active
M20 input: Cloud=... IMU=... keyframes=...
```

结束顺序：

1. 停止机器狗运动并保持安全姿态。
2. 在建图终端按 `Ctrl+C`，等待 `Saved map:` 和节点退出。
3. 再在 receiver 终端按 `Ctrl+C`。
4. 不要使用 `sudo drmap stop_mapping` 停止第三方 M20 节点；该命令只属于原厂算法。

如终端显示暂停，先按 `Ctrl+Q`，再按 `Ctrl+C`。

### 9.3 PCD 下载和查看

```bash
rsync -av -e "ssh -J user@192.168.101.36" \
  user@10.21.31.106:/home/user/codex_m20_drdds_validation/maps/<地图目录>/ \
  /home/lee/m20_maps/<地图目录>/
```

当前电脑 RTX 3060 使用 `nouveau` 驱动，普通 `pcl_viewer` 会触发 OpenGL pushbuf 崩溃。使用软件渲染：

```bash
LIBGL_ALWAYS_SOFTWARE=1 \
GALLIUM_DRIVER=llvmpipe \
pcl_viewer /home/lee/m20_maps/<地图目录>/full_cloud.pcd
```

## 10. 后续对齐原厂的实施顺序

### 阶段 A：完全同源输入

状态：**2026-08-27 已完成并完成静态真机输入验收。**

1. 已为 DrDDS receiver 增加 `sensor_msgs::msg::ImuPubSubType`。
2. root receiver 同时订阅：

   ```text
   /LIDAR/POINTS
   /IMU
   domain=0
   prefix=rt
   network=eth0/eth1
   use_shm=true
   ```

3. IMU 使用独立 Unix socket，完整保留 Imu header、姿态、协方差、角速度和加速度。
4. `slam_node` 在 `lidar_transport=drdds`、`imu_transport=drdds` 时同时使用 socket LiDAR 和 socket IMU，不再混用 ROS 2 `/IMU`。
5. 已记录消息时间戳差、频率、回退、接受/丢弃数量和 LiDAR 队列深度；gateway 记录 DrDDS 匹配、更新、接收和转发数量。

### 阶段 B：坐标系和外参

1. 已记录 `/LIDAR/POINTS` 的真实 `frame_id=lidar_link`。
2. 已确认 DrDDS `/IMU` 当前 `frame_id` 为空；轴方向和重力方向仍需动态验证。
3. 机器狗静止时检查：
   - 加速度模长是否约 9.81 m/s²；
   - 静止角速度均值和噪声；
   - 点云地面法向与重力是否一致；
   - 前后雷达区域是否在同一机体坐标系连续拼接。
4. 小角度原地旋转，检查点云和 IMU yaw 符号是否一致。
5. 直行已知距离，检查估计尺度和轴方向。
6. identity 外参只能在上述证据成立后保留，否则必须使用真机标定值。

### 阶段 C：算法参数映射

可以直接对齐的参数：

```text
输入降采样 leaf_size = 0.15 m
初始化静止时间至少覆盖原厂 init_time = 0.1 s，并建议实机 10～20 s
LiDAR/IMU 使用消息时间戳，不使用系统接收时间
原厂 IMU 噪声和 bias 随机游走作为起始标定参考
```

不能直接一一对应的参数：

```text
原厂 max_iteration=3       vs VGICP max_iterations
原厂 esti_plane_threshold  vs VGICP correspondence radius/fitness
原厂 lidar_cov             vs VGICP 体素协方差
原厂 ESKF观测更新          vs 当前 scan-to-map VGICP + 后端图
原厂 keyframe_time         vs 当前距离/角度关键帧
```

应先增加诊断，再基于真机数据调参，不应只把 YAML 数字机械复制过去。

### 阶段 D：动态 A/B 验收

使用完全相同的安全路线分别运行：

```text
A: sudo drmap mapping -b -s
B: 第三方算法
```

记录并对比：

- 总运行时间；
- 点云/IMU 接收数量和丢帧；
- 处理时延和队列积压；
- 轨迹总长度和 XY/Z 范围；
- 起终点闭合误差；
- 墙面厚度、重影、倾斜和拼接断层；
- 地图点数、体素分辨率和文件大小；
- CPU、内存、温度；
- 原厂和第三方算法的坐标轴、尺度和旋转方向。

动态验收最低要求：

```text
Cloud 约 10 Hz 连续
IMU 约 200 Hz 连续
运动时关键帧持续增长
估计路程与真实路程同量级
闭环回到起点时点云基本重合
无持续墙面双层、地图撕裂或整体倾斜
正常退出并生成完整 PCD/轨迹/摘要
```

## 11. 适配其他算法的通用接口合同

后续 FAST-LIO2、LIO-SAM、Point-LIO、FAST-LIO-MULTI 或其他算法接入 M20Pro 时，建议统一经过同一个“原厂输入适配层”，不要每个算法重复直接链接 DrDDS。

推荐架构：

```text
root m20_vendor_sensor_gateway
  ├─ DrDDS /LIDAR/POINTS -> PointCloud2 socket/IPC
  └─ DrDDS /IMU          -> Imu socket/IPC

普通用户算法适配器
  ├─ 保留原始时间戳和 frame
  ├─ 按目标算法转换 point time/ring
  ├─ 应用经过验收的 LiDAR-IMU 外参
  └─ 输出到独立命名空间和 TF
```

算法接入前必须回答：

1. 算法要求哪种点类型，是否要求 `ring`、`time`、`timestamp`？
2. 逐点时间单位是秒、毫秒、微秒还是纳秒？是绝对时间还是相对扫描时间？
3. 算法期望点云位于 LiDAR frame、IMU frame 还是 body frame？
4. IMU轴约定和重力方向是什么？
5. 外参定义是 `T_lidar_imu`、`T_imu_lidar`、`T_body_lidar` 还是相反方向？
6. 算法是否能承受 10 Hz 双雷达融合点云的点数和 NOS/GOS 算力？
7. 谁拥有 `map -> odom`、`odom -> base_link`，是否会与原厂定位冲突？
8. 地图输出是原始累计点、关键帧点、体素质心还是表面地图？

## 12. 当前结论

已经完成并真机确认：

- M20Pro NOS/GOS 网络和 SSH 跳板链路。
- 原厂 `/LIDAR/POINTS` 是前后双雷达融合点云。
- 原厂点云通过 DrDDS domain 0、`rt` prefix、`eth0/eth1`、共享内存发布。
- root receiver 可稳定接收约 10 Hz 点云并通过 Unix socket 转发。
- root gateway 可同时接收 DrDDS `/LIDAR/POINTS` 和 `/IMU`，并通过两个独立 Unix socket 转发。
- 当前 M20 可接收完全同源的约 10 Hz 融合点云和约 200 Hz IMU。
- 点云 frame 为 `lidar_link`；当前固件 IMU frame 为空，静止加速度模长约 9.78～9.81 m/s²。
- IMU 已改为跨扫描连续传播；修复了位置积分重复计加速度和静止姿态重力符号问题。
- 原厂 `leaf_size=0.15`、`skip_num=5`、`init_time=0.1` 和 identity 外参已映射到当前可对应的输入处理参数。
- ARM64/Foxy 构建、静态算法初始化、关键帧和 PCD保存通过。
- 地图保存卡顿已修复。
- 原厂 LIO 参数、输入话题、消息类型和算法框架已经提取。

尚未完成：

- IMU 空 frame 下的真实轴向、融合点云到 IMU 的 identity 外参语义验收。
- 300 平方米动态平移和地图质量修复。
- 与原厂算法相同路线的量化 A/B 地图对比。
- 长时间资源、温度、丢帧和安全验收。

当前最重要的工程判断是：**同源输入已经完成，但这不等于动态建图通过。下一次测试先做已知距离直行和原地小角度旋转，验证 `lidar_link`、空 IMU frame、重力和 yaw 符号下 identity 外参是否成立；同时量化丢弃旧帧后的处理时延。**

## 13. 2026-08-27 原厂 LIO 前端替换进展

本节晚于前文的 VGICP 历史记录，描述当前工作区最新状态。

已从 NOS 只读取得原厂公开头文件，并对 `slam_ddsnode` 动态符号进行核对：

```text
dr_lio::LidarOdometry
dr_lio::ImuProcess
state_ikfom = pos, rot, offset_R_L_I, offset_T_L_I, vel, bg, ba, grav
esekfom::update_iterated_dyn_share_modified
local_map::VoxelMap<..., 7>::getCorrespondPlane
local_map::VoxelMap<..., 7>::addPoints
```

原厂 LIO 被静态编入 `slam_ddsnode`，设备上不存在可直接链接的独立
`libdr_lio.so`。因此当前实现采用行为等价前端，而不是继续调整 VGICP：

- VGICP 已从 `M20_CORE_SOURCES` 和建图主路径移除；旧源码暂时保留但不参与构建。
- 新增活动误差状态 `[p, theta, v, bg, ba, g]` 的 ESKF；原厂固定 identity
  外参仍保存在名义变换中，`extrinsic_est_en=false`。
- IMU 协方差传播使用原厂 `acc_cov/gyr_cov/b_acc_cov/b_gyr_cov`。
- LiDAR 更新使用迭代点到平面残差和 `lidar_cov=0.001`，最多 3 次更新。
- 地图改为按 `top_level=1`、`plane_level=2`、`deepest_level=2` 查询的分层体素平面图。
- 修正去畸变时间语义：点云补偿到扫描终点，ESKF 状态也关联扫描终点；旧实现的扫描起点/终点混用已移除。
- `leaf_size=0.15` 用于 LIO 输入降采样；`leaf_size_body=0.05` 按原厂注释仅用于对外机体系点云；`skip_num=5` 为跳点。
- 输出语义已补齐，并保留隔离前缀：

  ```text
  /m20_slam/SLAM_ODOM
  /m20_slam/SLAM_ALIGNED_POINTS
  /m20_slam/DEPTH_POINTS
  /m20_slam/SLAM_ACCUMULATED_POINTS_MAP
  ```

  停止原厂 mapping 后，可以通过参数覆盖为原厂无前缀名称；不得让两套算法同时发布同名通道或 TF。

本机 Humble Release 构建和 3 个测试目标已通过；新增测试覆盖分层体素平面查询、已知高度误差的迭代点面收敛和静止 IMU 协方差传播。GOS ARM64/Foxy Release 构建也已通过；由于 GOS 镜像缺少 Python 模块 `ament_cmake_test`，`ctest` 包装器不能运行，但 3 个 gtest 二进制直接执行共 10 个测试用例全部通过。

NOS 最终静止缓存版结果：

```text
地图目录: /home/user/codex_m20_drdds_validation/maps/vendor_lio_static_cached_20260827/
accepted_clouds=2519
accepted_imus=49440
processed_clouds=1631
dropped_clouds=884
dropped_imus=0
keyframes=1
map_points=12714
trajectory poses=1628
x_range=0.016122 m
y_range=0.023158 m
z_range=0.015214 m
point-plane rejection=0
PCD SHA256=d075b4ef05fc64bdb52ec397c72b4114a227fb7eb13c5ba6045c02b750e903f5
```

最终缓存版平均处理约 6.5 Hz，相比首版约 1.9 Hz 和批量查询版约 5.8 Hz 已明显改善，但仍低于约 10 Hz 输入；当前有界队列丢弃约 35% 点云。静止约 4 分钟的 XYZ 范围均小于 2.4 cm，说明静止稳定性通过本轮检查，但这不能替代动态尺度和地图几何验收。

尚未完成、不得写成 1:1 已验收：

- NOS 满 10 Hz 实时性优化（当前约 6.5 Hz）；
- 静止、小角度 yaw、已知距离直行；
- 同一人工遥控路线下原厂/复现算法的轨迹尺度、姿态、地图几何和资源占用 A/B；
- 原厂私有 `voxel_block_map` 内部节点分裂、平面判定和淘汰策略无法仅凭公开头文件做到源码逐行相同，必须通过 A/B 行为指标约束。

## 14. 2026-08-27 原厂输出合同与七候选体素查询对齐

在原厂 LIO 3.4.0 二进制 SHA256
`1b94abf5533c9098630055ec511682fba670f43776ca41aa0b58f5ae97e286cd`
上继续完成只读反汇编和短时静态输出采样，确认：

- 原厂层级分辨率常量是 `0.64/0.32/0.16/0.08/0.04/0.02 m`；当前配置
  `top_level=1`、`plane_level=2`、`deepest_level=2` 对应 `0.32/0.16 m`，
  因此复现地图 finest resolution 已由 `0.15` 修正为 `0.16 m`。输入配准云
  `leaf_size=0.15 m` 保持不变，两者不是同一个参数。
- `getCorrespondPlane(..., std::array<VoxelCorre, 7>&, ...)` 的七个候选已由
  rodata 和循环边界共同确认：当前体素以及 `±X/±Y/±Z` 六个面邻居。复现查询
  已从每层 27 邻域改为同样的七候选结构。
- 原厂额外固定发布 `/SLAM_CLOUD_REGISTERED_BODY`。复现算法使用隔离话题
  `/m20_slam/SLAM_CLOUD_REGISTERED_BODY`，消息为 `PointXYZINormal`、
  `frame_id=base_link`、传感器时间戳、`leaf_size_body=0.05 m`，字段语义为
  `normal_z=ring`、`intensity=原始强度`、`curvature=扫描内相对时间毫秒`。
- `/SLAM_ALIGNED_POINTS` 对齐为 `PointXYZINormal`、配准云 `leaf_size=0.15 m`；
  世界系查询到平面时，`normal_*` 为平面法向，`intensity` 为平面偏置，
  `curvature` 保留降采样后的 ring。
- 原厂 `accumulated_points.enable=false` 时仍约 10 Hz 发布空 `PointXYZ`：
  `frame_id=base_link`、零时间戳、`point_step=16`、`1x0`、`dense=true`。
  复现算法已对齐该行为。
- 原厂 `voxel_map.ray_casting=false` 时 `/DEPTH_POINTS` 不产生数据；复现算法
  默认也不发布该通道的数据，不再把全局体素质心快照错误地复用为 DEPTH 输出。

代码将上述合同抽取为可测试接口，并新增输出字段、空累计点、禁用 ray casting
以及七候选体素模板测试。本机 Humble Release 构建通过，共 `18 tests`、
`0 errors`、`0 failures`；GOS ARM64/Foxy Release 构建通过，四个 gtest 二进制
共 14 个测试用例全部通过。

NOS 静止同源验收目录：

```text
/home/user/codex_m20_drdds_validation/maps/vendor_7cell_static_20260827_1320/
```

约 76 秒有效输入窗口的结果：

```text
accepted_clouds=764
processed_clouds=759
dropped_clouds=4
accepted_imus=15093
dropped_imus=0
keyframes=1
map_points=7305
```

即处理约 `9.94 Hz`，LiDAR 主动丢帧约 `0.52%`，队列通常为 0，已从上一版
约 `6.5 Hz` 提升到接近原厂约 10 Hz 输入。静止轨迹 755 个姿态的范围：

```text
x_range=0.017259 m
y_range=0.018811 m
z_range=0.015563 m
首尾三维距离=0.006709 m
```

在隔离 `ROS_DOMAIN_ID=77` 下取得复现输出运行时证据：

```text
/m20_slam/SLAM_ALIGNED_POINTS
  frame=m20_slam_map, PointXYZINormal, point_step=48
  width=3008, dense=true, sensor stamp

/m20_slam/SLAM_CLOUD_REGISTERED_BODY
  frame=base_link, PointXYZINormal, point_step=48
  width=5237, dense=false, sensor stamp

/m20_slam/SLAM_ACCUMULATED_POINTS_MAP
  frame=base_link, PointXYZ, point_step=16
  width=0, height=1, dense=true, stamp=0

/m20_slam/DEPTH_POINTS
  15 秒探针窗口内未收到消息
```

需要继续保持边界：上述结果证明输入、参数、输出消息合同、静止稳定性和接近满帧
处理通过，但仍不能证明动态 1:1。下一次必须由人工低速遥控完成小角度 yaw 和已知
距离直行，再做同路线原厂/复现 A/B；原厂私有体素节点分裂、平面判定和淘汰策略也
只能继续通过动态轨迹与地图几何约束，不能宣称源码级一致。

## 15. 2026-08-27 双外参、IMU 初始化与参数覆盖对齐

本轮再次从 NOS 只读取得当前
`/opt/robot/share/slam/conf/params.yaml`，并对 SHA256 为
`1b94abf5533c9098630055ec511682fba670f43776ca41aa0b58f5ae97e286cd`
的 ARM64 `slam_ddsnode` 反汇编。新增确认和实现如下：

### 15.1 双外参不是单一 `T_lidar_imu`

原厂依次读取 16 元素行主序 `extrinsic_B_I`、`extrinsic_B_L`。二进制在
`getParamsFromFile()` 中先对 `extrinsic_B_I` 求逆，再与 `extrinsic_B_L`
相乘，随后把结果传给 `ImuProcess::SetExtrinsic()`：

```text
T_I_L = inverse(T_B_I) * T_B_L
```

复现已分别保存 `T_B_I` 和 `T_B_L`，仅将上述组合结果用于去畸变、点面观测和
LiDAR 输出位姿。`m20_mapping.yaml` 已改用原厂参数名
`lio.extrinsic_B_I/B_L`；新增非 identity 单元测试防止乘法方向退化。

### 15.2 IMU 初始化门槛和初始协方差

原厂公开头文件定义 `MAX_INI_COUNT=200`。复现不再仅凭 `init_time=0.1 s`
启动地图，而是同时要求累计至少 200 个 IMU 样本和配置的最短时间窗口。以当前
约 200 Hz IMU 计算，初始化实际约需 1 秒；`init_time` 不能被误解成完整初始化
样本门槛。

原厂 `IMUInit()` 的 23 维协方差从单位阵开始，仅把固定外参、陀螺 bias、加速度
bias 和 S2 gravity 块收紧。复现现已从旧的 18 维欧氏重力状态改为同布局的 23 维
误差状态：

```text
[pos(3), rot(3), offset_R_L_I(3), offset_T_L_I(3),
 vel(3), bg(3), ba(3), grav(S2,2)]

P(pos)=1, P(rot)=1, P(vel)=1
P(offset_R_L_I)=1e-5, P(offset_T_L_I)=1e-5
P(bg)=1e-4, P(ba)=1e-3, P(grav tangent)=1e-5
```

重力名义量仍保存为固定模长三维向量，但误差更新和协方差传播使用二维切空间。
点面观测 Jacobian 也扩展为原厂 `h_x` 的 12 列布局：位置、姿态、雷达在 IMU 中的
旋转和平移；默认 `extrinsic_est_en=false` 时外参列保持关闭并由紧协方差固定。

### 15.3 观测有效点数量

原厂 `ObsModel()` 在 `effect_feat_num <= 0` 时才令观测无效；当前复现此前自定义的
`correspondences < 20` 拒绝条件没有原厂依据，现已改为仅在零有效平面时拒绝。
数值分解仍由先验信息正则化保护。

### 15.4 参数、话题和数据覆盖矩阵

| 合同组 | 当前状态 | 边界 |
|---|---|---|
| `/LIDAR/POINTS`、`/IMU`、DrDDS domain/prefix/network/SHM | 已运行时对齐 | root gateway 与普通用户算法进程分离 |
| 消息时间戳、逐点 `timestamp`、`ring`、约 10/200 Hz | 已运行时对齐 | 默认 `*_use_system_time=false`；系统接收时间启用路径未验收 |
| `extrinsic_B_I/B_L` | 已按原厂组合公式实现 | identity 是否符合真实轴向仍需动态 yaw/直行确认 |
| LIO 数值参数、200 样本初始化、23维状态、S2重力、12列点面 Jacobian、七候选体素 | 已实现同结构 | 私有体素分裂/平面生命周期和原厂迭代收敛细节仍只能行为等价 |
| ODOM、ALIGNED、BODY、禁用 ACCUMULATED/DEPTH 输出 | 已运行时对齐 | 默认保留 `/m20_slam` 隔离前缀和隔离 frame |
| `save_full_pcd/full_map_save_path` | 已声明并读取 | 原厂路径替换为工作区隔离路径；保存服务仍是复现扩展 |
| `depth_image.*`、`height_map.*`、`occ_grid_2d.*` | 已声明并校验，默认禁用 | depth/height 启用时会明确拒绝启动，避免静默空实现；栅格生成仍未实现 |
| 完整 `pgo.*` | 已声明并读取、启动时明确告警 | 当前后端不是原厂 PGO/GHT，不能据此宣称闭环行为对齐 |
| `accumulated_points`/ray casting 启用路径 | 禁用行为已对齐 | 启用路径仍未完成原厂射线和局部裁剪语义 |

本机 Humble Release 重新构建通过；四个 gtest 目标合计 `20 tests`、
`0 errors`、`0 failures`。同一源码已同步到 GOS 隔离构建区，ARM64/Foxy
Release 构建通过，四个 gtest 二进制共 16 个用例通过，并已将安装树部署到 NOS
隔离目录 `/home/user/codex_m20_drdds_validation/install`。

NOS 200 样本初始化版本静止运行目录：

```text
/home/user/codex_m20_drdds_validation/maps/vendor_init200_static_20260827_140649/
```

约 125.5 秒有效输入的保存结果：

```text
accepted_clouds=1255
accepted_imus=25059
processed_clouds=1255
dropped_clouds=0
dropped_imus=0
keyframes=1
map_points=13554
PCD SHA256=c00e08abeb7486ba847b36808fe33817ce66dea13939de96a897b85d58c81354
```

运行期间通常 `lidar_queue=0`，输入约 10 Hz/200 Hz，说明 200 IMU 样本门槛没有
引入持续积压，初始化地图首次出现在第 12 个 LIO frame 左右，即约 1.2 秒而不是
配置字面值 0.1 秒。地图周期保存点数从 7184 增长到 13554。停止后发现 launch
退出遗留一个复现 `slam_node`，已对明确 PID 发送 `SIGINT` 并确认 NOS 上原厂
`slam_ddsnode`、DrDDS gateway、复现 `slam_node` 和 `start_mapping.sh` 均无残留。

本轮也暴露出旧验收口径缺陷：该次 `trajectory.csv` 取自 PGO/iSAM2 缓存，只有
frame 12、13 两行，XYZ 范围分别为 `0.001291/0.003072/0.003599 m`，首尾三维
距离约 `0.004905 m`。它不能代表全部 1255 帧 LIO 轨迹，也不能据此证明 1254 次
点面观测全部成功。代码现已改为：

- `mapping_summary.txt` 分别保存初始化等待帧、bootstrap 帧、成功点面更新、拒绝
  更新和空配准云计数；
- 运行状态日志同步输出上述计数；
- `trajectory.csv` 改为记录每一个实际发布的 LIO 位姿，并追加 `stamp_ns`，不再依赖
  PGO 估计缓存。

该可观测性修正在本机 Humble Release 下重新构建并通过 `20 tests`、`0 errors`、
`0 failures`；GOS ARM64/Foxy Release 构建通过，四个 gtest 二进制共 16 个用例
通过。安装树和源码随后部署到 NOS，并完成第二轮短时静止复测：

```text
地图目录: /home/user/codex_m20_drdds_validation/maps/vendor_observability_static_20260827_142909/
accepted_clouds=543
accepted_imus=10847
processed_clouds=543
initialization_wait_clouds=11
bootstrap_clouds=1
successful_lio_updates=531
rejected_lio_updates=0
empty_registration_clouds=0
dropped_clouds=0
dropped_imus=0
keyframes=1
map_points=8230
```

计数满足 `11 + 1 + 531 = 543`，说明本轮每个进入处理函数的 LiDAR 帧都有明确
归类；初始化后 531 帧全部完成点面更新，没有把“进入处理循环”误报为“配准成功”。
完整 `trajectory.csv` 有 532 个 LIO 位姿（bootstrap 加 531 次更新），frame 12 到
543，传感器时间跨度 `53.199993 s`：

```text
x_range=0.009710 m
y_range=0.015701 m
z_range=0.011974 m
首尾三维距离=0.007742 m
yaw_range=0.000762 rad
首尾 yaw 差=-0.000010 rad
PCD SHA256=456ad728f16b794dfbf49447da7014357fdf5babf9b1106cc782a9ce53fc641a
```

这证明 200 样本初始化版本在 NOS 静止条件下能以约 10 Hz 满帧完成有效 LIO 更新，
且约 53 秒 XYZ 范围小于 1.6 cm；仍不能替代动态尺度、外参方向和地图几何验收。

两次 SSH 运行结束都曾发现 `ros2 launch` 子进程中的复现 `slam_node` 未随进程组
退出，均已对明确 PID 发送 `SIGINT` 清理。进一步确认 SSH 客户端转发 Ctrl+C 后
会话可能在保存响应后被 sshd 回收，使普通 shell trap 的后半段无法执行。启动脚本
现于退出信号到达时立即捕获当时存在的复现 PID，并启动脱离 SSH 会话的 fallback：

- fallback 先调用保存服务，再发送 `SIGINT`，超时才发送 `SIGTERM`；
- 每次信号前都核对 `/proc/<pid>/exe` 必须等于当前工作区安装的 `slam_node`；
- 不按进程名模糊匹配，因此不会触及后续新实例或 `/opt/robot` 原厂进程。

最终短启动/退出目录
`/home/user/codex_m20_drdds_validation/maps/cleanup_smoke4_20260827_1452/`
已保存地图；Ctrl+C 后等待 8 秒复查，原厂 `slam_ddsnode`、DrDDS gateway、复现
`slam_node` 和 `start_mapping.sh` 均无残留。脚本同时通过本机 `bash -n`。

当前仍不得宣称 1:1 完成。下一硬件步骤需由用户人工低速完成小角度 yaw、已知距离
直行和同路线原厂/复现动态 A/B。

## 16. 2026-08-27 最终静态复核后的增量实现

本轮根据原厂公开 `use-ikfom.h`、`ObsModel` 符号尺寸/矩阵维度和已保存地图产物，
完成以下增量：

- ESKF 改为 23×23 误差协方差和 12 维过程噪声输入，重力改用二维 S2 切空间；
- 点面观测使用 12 列位姿/外参 Jacobian，外参在线估计开关真正控制外参观测列；
- `full_cloud.pcd` 改为累计关键帧变换后的稠密地图，不再把 `DEPTH_POINTS` 体素质心
  当作完整地图；保存字段保持 `x y z intensity`，并按原厂产物把 intensity 写为 0；
- 保存目录新增 `.sessions/session_0/lio_odom.pose` 和 `poses.txt`，每行为
  `stamp tx ty tz qx qy qz qw`；
- YAML 中原先静默存在的 lidar time/type、depth image、height map、occupancy 和
  PGO 参数全部进入声明/读取/校验路径。尚未实现的启用模块会拒绝或明确告警。

原厂与旧复现 PCD 复核：二者 `full_cloud.pcd` 的 PCD v0.7 字段、类型和每点 16 字节
结构相同；原厂为 1,289,245 点，旧静止复现为 8,230 个体素质心。原厂 session
关键帧为 `x y z intensity normal_x normal_y normal_z curvature`、每点 32 字节。
当前尚未生成与原厂完全一致的 session 关键帧 PCD、`.blocks/*.chunk`、optimizer 和
occupancy 工程文件。

本机 Humble 重新构建通过，四个 gtest 目标共 `21 tests`、`0 errors`、
`0 failures`。该结果只证明源码和离线数学/接口测试通过；23 维版本尚未部署到
GOS/NOS，也尚未完成动态尺度、外参方向、闭环或地图几何验收。
