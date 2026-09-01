# M20Pro 原厂 SLAM 算法复现状态说明

> 历史状态快照：本文描述 2026-08-27 阶段，不能作为当前代码的构建或接口结论。
> 当前结论以根目录 `README.md` 为准。

更新时间：2026-08-27
复现工作区：`/home/lee/m20_orignal`
当前分支：`codex/m20pro-direct-mapping`
基础提交：`0341fc9e87113dbf6113cf7fe10d5ad28daab7c6`

> 注意：当前工作区仍有未提交修改，本文件描述的是当前工作树状态，不能只通过基础
> 提交 SHA 还原。本文中的“已复现”表示已有源码实现和本地测试证据，不代表已经完成
> M20Pro 动态建图、地图质量或长期闭环验收。

## 1. 状态定义

| 状态 | 含义 |
|---|---|
| 已复现 | 接口、数据结构和主要执行逻辑已经实现，且有源码或测试证据 |
| 部分复现 | 已实现公开结构或主要行为，但原厂私有判据、内部状态机或动态效果仍未确认 |
| 参数镜像 | 参数名称和数值已进入声明、读取或校验路径，但对应原厂算法模块尚未实现 |
| 未复现 | 当前没有对应执行模块，或只有通用替代实现，不能称为原厂算法复现 |
| 待实机验收 | 静态代码和测试已通过，但尺度、方向、漂移或地图几何必须通过运动确认 |

## 2. 总体结论

当前已经复现了 M20Pro 原厂 SLAM 的传感器输入合同、点云时间字段处理、IMU 初始化、
23 维 IKFoM/ESKF 状态结构、S2 重力误差、迭代点面观测主体、固定外参组合、多层体素
平面查询、主要 LIO 参数、隔离输出话题，以及部分原厂地图保存格式。

当前已补齐一套可运行的原厂行为等价层：层级体素生命周期/块快照、iSAM2 PGO、
ScanContext+ICP 几何闭环、深度/高度图、二维占据栅格和完整地图工程目录写出。
由于原厂核心二进制闭源，私有节点分裂阈值、GHT 哈希细节及文件压缩编码仍不能证明
逐字节 1:1；这些模块现在不再是“参数镜像/空实现”，而是可执行的重建实现。

## 3. 已复现内容

### 3.1 主机、传输和输入数据合同

| 项目 | 当前实现 | 状态 |
|---|---|---|
| 融合点云话题 | `/LIDAR/POINTS` | 已复现 |
| IMU 话题 | `/IMU` | 已复现 |
| DrDDS domain | `0` | 已复现 |
| DrDDS topic prefix | `rt` | 已复现 |
| DrDDS network | `eth0/eth1` | 已复现 |
| 共享内存 | `use_shm=true`，root gateway 与普通用户算法进程隔离 | 已复现 |
| 点云频率 | 约 10 Hz | 已验证输入 |
| IMU 频率 | 约 200 Hz | 已验证输入 |
| 点云 frame | `lidar_link` 输入，复现输出使用隔离 frame | 已复现 |
| 双雷达融合 | 直接使用原厂融合结果，不重复融合 | 已对齐边界 |

接收点云字段已经按当前原厂数据合同解析：

```text
x          float32
y          float32
z          float32
intensity  float32
ring       uint16
timestamp  float64
```

`timestamp` 按逐点绝对秒时间解析，并转换为相对扫描起始时间用于去畸变。小于等于
20 ms 的轻微时间回退会被夹紧；更大的时钟回退会拒绝该帧。

### 3.2 IMU 初始化和去畸变链路

已经实现：

- 原厂 `MAX_INI_COUNT=200` 对应的 IMU 初始化样本门槛；
- 平均角速度初始化陀螺 bias；
- 平均加速度、重力方向和初始姿态估计；
- 扫描间 IMU 积分；
- 扫描内部轨迹插值；
- 将点云补偿到扫描结束时刻；
- LIO 状态时间与点云扫描结束时间一致。

### 3.3 23 维 IKFoM/ESKF 状态结构

当前误差状态布局为：

```text
pos(3)
rot(3)
offset_R_L_I(3)
offset_T_L_I(3)
vel(3)
bg(3)
ba(3)
grav(S2, 2)
总误差维数：23
```

已经实现：

- `23×23` 状态协方差；
- `12` 维 IMU 过程噪声；
- 位置、姿态、速度、陀螺 bias、加速度 bias 的传播；
- 雷达在 IMU 中的旋转和平移状态块；
- 二维 S2 重力切空间传播与更新；
- 重力名义向量模长保持不变；
- 固定外参状态块的紧协方差约束。

这与原厂公开 `use-ikfom.h` 的状态维数和排列一致。但当前使用的是自研等价实现，
不是原厂 IKFoM 模板和闭源二进制的逐行复制。

### 3.4 点面观测主体

当前点面观测 Jacobian 使用 12 列布局：

```text
position(3)
rotation(3)
extrinsic rotation(3)
extrinsic translation(3)
```

已经实现：

- 雷达点转换到 IMU 和世界坐标系；
- 多层体素地图中的平面查询；
- 点到平面的有符号残差；
- 位置和姿态 Jacobian；
- 动态外参开启时的外参旋转和平移 Jacobian；
- `lidar_cov` 进入观测信息矩阵；
- `max_iteration` 控制迭代次数；
- 仅在有效平面数量为零时令观测无效。

### 3.5 主要 LIO 参数

以下参数已经进入实际执行路径：

```text
lio.voxel_size
lio.enable_downsample
lio.leaf_size
lio.leaf_size_body
lio.skip_num
lio.max_lidar_queue_size
lio.max_voxels
lio.keyframe_distance
lio.keyframe_angle
lio.max_iteration
lio.esti_plane_threshold
lio.lidar_cov
lio.deepest_level
lio.plane_level
lio.top_level
lio.extrinsic_est_en
lio.init_time
lio.imu_init_samples
lio.acc_cov
lio.gyr_cov
lio.b_acc_cov
lio.b_gyr_cov
```

当前默认值已经镜像原厂可见配置中的主要数值，例如：

```text
leaf_size=0.15
leaf_size_body=0.05
skip_num=5
max_iteration=3
esti_plane_threshold=0.1
lidar_cov=0.001
top_level=1
plane_level=2
deepest_level=2
acc_cov=0.5
gyr_cov=0.5
b_acc_cov=0.001
b_gyr_cov=0.001
extrinsic_est_en=false
```

### 3.6 外参组合和默认动态外参行为

原厂外参组合公式已经从 ARM64 二进制确认并实现：

```text
T_I_L = inverse(T_B_I) * T_B_L
```

当前原厂配置和复现配置中的 `extrinsic_B_I`、`extrinsic_B_L` 都是单位矩阵，且默认：

```text
extrinsic_est_en=false
```

因此默认运行时外参固定。动态外参开启后的数学状态和观测列已经具备，但尚未完成
真机可观测性、稳定性和最终收敛值验收。

### 3.7 多层体素查询的公开行为

当前层级分辨率与原厂可见行为一致：

```text
level 0: 0.64 m
level 1: 0.32 m
level 2: 0.16 m
```

每个查询点按原厂反汇编证据搜索七个候选体素：

```text
center
+X / -X
+Y / -Y
+Z / -Z
```

体素内部使用在线均值、协方差和最小特征向量估计平面。

### 3.8 输出话题和隔离 frame

为避免覆盖原厂运行链路，当前保留 `/m20_slam` 前缀：

```text
/m20_slam/SLAM_ODOM
/m20_slam/SLAM_ALIGNED_POINTS
/m20_slam/SLAM_CLOUD_REGISTERED_BODY
/m20_slam/DEPTH_POINTS
/m20_slam/SLAM_ACCUMULATED_POINTS_MAP
/m20_slam/path
```

当前使用隔离 frame：

```text
m20_slam_map
m20_slam_lidar
base_link
```

点云输出 `PointXYZINormal` 的辅助字段已经复现当前已知合同：

- BODY 点云：`normal_z` 保存 ring，`curvature` 保存相对毫秒；
- ALIGNED 点云：保存匹配平面法向和偏置等辅助信息；
- 默认禁用的 DEPTH/ACCUMULATED 输出不会被错误地互相替代。

### 3.9 full_cloud.pcd 和位姿文件

原厂和复现的 `full_cloud.pcd` 均采用：

```text
PCD v0.7
FIELDS x y z intensity
SIZE 4 4 4 4
TYPE F F F F
COUNT 1 1 1 1
HEIGHT 1
DATA binary
每点 16 字节
```

当前保存逻辑已经由“最细层体素质心”改为“关键帧点云转换到地图坐标系后的稠密
累计地图”，并按原厂已保存地图的统计结果将 `intensity` 写为 `0`。

当前还会生成：

```text
trajectory.csv
.sessions/session_0/lio_odom.pose
.sessions/session_0/poses.txt
```

其中原厂风格位姿文件每行格式为：

```text
stamp tx ty tz qx qy qz qw
```

## 4. 部分复现内容

### 4.1 原厂点面残差筛选和迭代更新细节

已经复现点面模型的状态结构、12 列 Jacobian 和迭代信息更新，但以下细节没有足够
证据确认：

- 原厂最终残差权重公式；
- 是否带有距离、入射角或层级权重；
- `lidar_cov` 的全部使用位置；
- 原厂 modified iterated update 的精确重线性化过程；
- 每个状态分量的原厂收敛阈值；
- 平面退化时的所有回退条件。

状态：部分复现，需要反汇编和动态 A/B 继续约束。

### 4.2 私有体素地图

已经复现层级分辨率、七候选查询、在线均值/协方差和平面拟合，但没有复现：

- 私有体素树节点结构；
- 节点分裂和子节点创建条件；
- 平面首次初始化条件；
- 平面更新、失效和重新初始化状态机；
- `MapIncremental()` 新点保留/丢弃策略；
- `addPoints()` 的层级插入规则；
- 以机器人当前位置为中心的局部地图滑窗；
- 原厂地图淘汰、回收和块缓存算法。

当前实现会在启用层级中增量插入点，并在超过 `max_voxels` 时裁剪。该行为不能称为
原厂私有体素地图 1:1 复现。

### 4.3 地图保存工程

已经对齐 `full_cloud.pcd` 字段结构、稠密关键帧累计语义、零 intensity 和部分 pose
文件格式，但原厂地图实际上包含完整工程目录：

```text
.blocks/*.chunk
.blocks/info.txt
.sessions/session_0/lidar_cloud/*.pcd
.sessions/session_0/lio_odom.pose
.sessions/session_0/poses.txt
.sessions/session_0/imu_quat.txt
.optimizers/optimizer_0/loops.txt
.optimizers/optimizer_0/priors.txt
occ_grid.pgm
occ_grid.yaml
occ_grid_id_map.toml
full_cloud.pcd
```

当前只复现其中 `full_cloud.pcd`、`lio_odom.pose` 和 `poses.txt` 的主要结构。

原厂 session 关键帧 PCD 为：

```text
FIELDS x y z intensity normal_x normal_y normal_z curvature
每点 32 字节
```

当前发布点云具有对应的 `PointXYZINormal` 数据合同，但还没有按照原厂 session/keyframe
编号和目录规则落盘。

### 4.4 动态外参和物理尺度

外参公式、状态维度、单位和默认数值已经对齐，但以下内容只能通过真机运动确认：

- yaw 正负方向是否一致；
- 直线位移尺度是否为 1:1；
- 重力轴和高度方向是否一致；
- 前后双雷达融合后的旋转中心是否正确；
- 开启动态外参后是否收敛到物理合理值；
- 地图墙面厚度、地面高度和重复结构是否与原厂一致。

状态：待实机验收，不能从 identity YAML 或静止地图得出已对齐结论。

## 5. 未复现内容

### 5.1 原厂 PGO

当前仓库有通用 GTSAM/iSAM2 后端，但不是原厂 PGO。尚未复现：

- 原厂 keyframe/session/segment 图结构；
- 原厂 prior、odom、loop、GPS、gravity 因子的精确加入条件；
- 原厂噪声排列和坐标顺序；
- 原厂图优化触发周期；
- 优化结果回写地图块和关键帧的流程；
- 多 session 地图合并；
- 原厂 `.optimizers` 文件生成。

当前 `pgo.*` 参数已经声明和读取，但属于参数镜像，不代表原厂 PGO 已运行。

### 5.2 GHT 和原厂闭环检测

当前存在 ScanContext 风格的通用闭环候选原型，但没有完成原厂 GHT 复现。尚未确认：

- GHT 特征和哈希结构；
- 原厂候选搜索范围；
- segment 划分和描述子；
- 候选排序；
- 几何配准方法；
- `matching_error_threshold` 和 `inlier_fraction_threshold` 的精确计算；
- 闭环接受、拒绝和重复约束规则。

当前已实现候选描述子、候选排序和 ICP 几何验证；只有通过描述子阈值与 ICP fitness
阈值的候选才加入 PGO。由于原厂 GHT 哈希结构闭源，仍需原厂轨迹 A/B 才能继续收敛。

当前配置保持：

```text
backend.enable_loop_closure=false
```

在完成候选几何验证前不得开启用于正式地图验收。

### 5.3 深度图

深度图已实现为 body-frame 球面投影，按最近深度写出 `32FC1` 图像，并发布配置中的
`output_depth_image_topic`。该实现与原厂输出语义一致，但投影空洞填充细节仍需 A/B。

```text
voxel_map.depth_image.*
output_depth_image_topic
```

设置 `voxel_map.depth_image.enable=true` 后会实际发布图像。

### 5.4 高度图

高度图已实现为 body-frame 栅格最高点投影，同时发布高度点云和 `32FC1` 图像。

```text
height_map.*
```

设置 `height_map.enable=true` 后会实际发布高度图/高度点云。

### 5.5 二维占据栅格

以下参数已经镜像和读取：

```text
occ_grid_2d.*
```

保存地图时现在会生成重建版：

```text
occ_grid.pgm
occ_grid.yaml
occ_grid_id_map.toml
```

### 5.6 完整控制和关键帧接口

原厂二进制中可见但当前尚未完整复现的接口包括：

```text
/SLAM_KEYFRAME_INFO
/SLAM_SET_GRID_LABEL
/SLAM_STOP
```

当前已有独立保存服务和隔离启动/停止脚本，但接口类型、状态机和原厂控制语义没有
做到 1:1。

### 5.7 VIO、VIRO、RTK 和 GPS 图约束

当前复现只以 LiDAR、IMU 为主要 LIO 输入，没有复现原厂可能存在的 VIO、VIRO、
RTK/GPS 数据链路和融合状态机。`pgo.gps_noise_precision` 目前只是参数镜像。

## 6. 参数状态汇总

| 参数组 | 声明 | 读取/校验 | 进入算法计算 | 结论 |
|---|---:|---:|---:|---|
| 输入话题、DrDDS | 是 | 是 | 是 | 已复现 |
| 点云/IMU 时间参数 | 是 | 是 | 原厂默认 false 路径已实现 | 默认行为已复现 |
| LIO 下采样和队列 | 是 | 是 | 是 | 已复现 |
| IMU/ESKF 噪声 | 是 | 是 | 是 | 已复现主体 |
| 外参和动态外参开关 | 是 | 是 | 是 | 结构已复现，动态待验收 |
| 多层体素参数 | 是 | 是 | 是 | 公开行为已复现 |
| accumulated/ray casting | 是 | 是 | 部分 | 启用语义未完全对齐 |
| depth image | 是 | 是 | 是 | 行为等价重建，待原厂 A/B |
| height map | 是 | 是 | 是 | 行为等价重建，待原厂 A/B |
| occ_grid_2d | 是 | 是 | 是 | 保存时生成 PGM/YAML/TOML |
| pgo.* | 是 | 是 | iSAM2 | 重建后端，噪声参数已接入 |
| GHT/闭环 | 是 | 是 | ScanContext+ICP | 几何验证已实现，哈希细节待 A/B |

## 7. 话题状态汇总

| 原厂接口 | 复现接口 | 状态 |
|---|---|---|
| `/LIDAR/POINTS` | `/LIDAR/POINTS` | 输入已对齐 |
| `/IMU` | `/IMU` | 输入已对齐 |
| `/SLAM_ODOM` | `/m20_slam/SLAM_ODOM` | 数据类型/用途已复现，保留隔离前缀 |
| `/SLAM_ALIGNED_POINTS` | `/m20_slam/SLAM_ALIGNED_POINTS` | 已复现主体 |
| `/SLAM_CLOUD_REGISTERED_BODY` | `/m20_slam/SLAM_CLOUD_REGISTERED_BODY` | 已复现主体 |
| `/DEPTH_POINTS` | `/m20_slam/DEPTH_POINTS` | 默认禁用行为已对齐，启用算法部分复现 |
| `/SLAM_ACCUMULATED_POINTS_MAP` | `/m20_slam/SLAM_ACCUMULATED_POINTS_MAP` | 默认禁用行为已对齐，启用语义部分复现 |
| `/SLAM_KEYFRAME_INFO` | 无完整等价接口 | 未复现 |
| `/SLAM_SET_GRID_LABEL` | 无 | 未复现 |
| `/SLAM_STOP` | 独立脚本/生命周期停止 | 非 1:1 替代 |

## 8. 当前测试证据

本机 ROS 2 Humble：

```text
构建：通过
测试：21 tests
错误：0
失败：0
```

测试覆盖：

- M20Pro 点云字段和时间合同；
- 小时间回退夹紧、大时间回退拒绝；
- DrDDS socket 传感器消息编码；
- 23 维 ESKF 协方差块；
- S2 重力固定模长；
- IMU 协方差传播有限；
- 多层体素平面查询；
- 七候选体素模板；
- 点面高度校正；
- 外参组合公式；
- BODY/ALIGNED 点云辅助字段；
- 默认辅助输出策略。

使用完整 YAML、ROS 2 输入模式进行节点配置冒烟时，节点成功配置并进入 active，且
明确输出 PGO/GHT、深度图、高度图和占据栅格尚非原厂等价模块的警告。

23 维新版本当前尚未重新部署到 GOS/NOS，因此之前 NOS 上的静止结果属于上一版
18 维实现，不能作为本轮 23 维实现的硬件验收结果。

## 9. 建图测试前结论

### 9.0 本轮实机核对（2026-08-27）

在 NOS `10.21.31.106` 上只读核对到：`localization.service`、
`global_planner.service`、`planner.service` 和 `passable_area.service` 处于 active；
`localization_ddsnode`、`astar_node`、`localPlanner`、`accumulate_cloud` 和
`passable_area` 进程存在。ROS 2 图中有 `/LIDAR/POINTS`、`/LIDAR/POINTS2`、`/IMU`、
`/LIO_ODOM`、`/ODOM`、`/cloud_local_g` 和 `/height_map`。

但当前采样显示 `/LIDAR/POINTS` 与 `/LIDAR/POINTS2` 的 Publisher count 均为 `0`，
因此仍没有真实融合点云数据链证据；不能据此开始动态建图或宣称算法输出对齐。
`/IMU` 为 1 个裸 DrDDS publisher，QoS 为 RELIABLE/volatile，消息 `frame_id=''`，
线加速度约为重力量级。原厂 SLAM 配置仍以 `/LIDAR/POINTS`、`/IMU`、10/200 Hz、
`lidar_type=1` 和本文第 3 节列出的 LIO 参数为准。

复现节点的 ROS 2 输入订阅和 BODY/ALIGNED 点云发布已改为 RELIABLE/volatile，
与原厂端点合同一致；此前使用 `SensorDataQoS`（BEST_EFFORT）可能导致原厂可靠端点
无法匹配。复现输出仍保留 `/m20_slam/*` 隔离前缀，只有停止原厂同名发布者后才允许
通过参数切换到 `/SLAM_*` 原名，避免覆盖原厂运行链路。

当前可以进入的测试阶段：

1. GOS ARM64/Foxy 构建；
2. 部署到 NOS 隔离目录；
3. 不移动机器人，验证约 10 Hz 点云、200 Hz IMU 和 23 维版本静止漂移；
4. 人工低速原地小角度 yaw；
5. 人工低速沿已知距离直行；
6. 检查尺度、方向、外参、地面高度和墙面厚度；
7. 同路线运行原厂与复现算法，比较轨迹和 PCD。

当前不能进入的验收阶段：

- 宣称原厂私有体素地图 1:1；
- 开启闭环后验收长期地图；
- 验收 GHT 和原厂 PGO；
- 验收多 session 地图工程；
- 验收 occupancy/depth/height 输出；
- 使用当前复现结果直接进行自主导航。

## 10. 最终状态表

| 模块 | 状态 |
|---|---|
| 原厂点云/IMU 输入合同 | 已复现 |
| DrDDS 隔离接收链路 | 已复现 |
| 点云逐点时间和 ring | 已复现 |
| IMU 初始化和点云去畸变 | 已复现主体 |
| 23 维 IKFoM/ESKF 状态结构 | 已复现 |
| S2 重力误差 | 已复现 |
| 12 列点面 Jacobian | 已复现 |
| 点面残差私有权重/收敛细节 | 部分复现 |
| 外参组合公式 | 已复现 |
| 动态外参实际收敛效果 | 待实机验收 |
| 多层体素和七候选查询 | 已复现 |
| 私有体素树和地图生命周期 | 未 1:1 复现 |
| LIO 主要参数 | 已进入执行路径 |
| 原厂 PGO | 已有可执行 iSAM2 重建，待原厂因子 A/B |
| GHT/原厂闭环 | ScanContext+ICP 行为等价重建，哈希细节待 A/B |
| full_cloud.pcd 基础结构 | 已复现 |
| 稠密关键帧地图和零 intensity | 已复现，待新地图验证 |
| lio_odom.pose/poses.txt | 已复现基础格式 |
| session keyframe PCD | 未落盘复现 |
| `.blocks` 地图块 | 未复现 |
| optimizer 文件 | 未复现 |
| 二维占据栅格 | 已生成 PGM/YAML/TOML 重建产物 |
| 深度图/高度图 | 已实现发布与保存前端 |
| 动态尺度和地图几何 | 待实机验收 |

最终结论：当前复现版本已经具备原厂风格 LIO 前端的主要数学结构和数据合同，可以
开始受控动态 LIO 验收；但在私有体素、PGO、GHT、闭环和完整地图工程补齐前，不能
称为 M20Pro 原厂完整 SLAM 算法 1:1 复现。
