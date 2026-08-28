# M20Pro 漂移/闭环对齐回归（2026-08-28）

## 结论

按“关键帧 → PGO → 几何闭环/GHT → 官方 bag”顺序完成复现侧回归。最新完整 bag 回放已接收并处理全部 452 帧 LiDAR、8301 条 IMU，内部无丢帧；终端闭环约束将首尾误差降到约 0.20 m，轨迹长度约 36.8 m，接近原厂 35.5007 m（约 3.7% 差异）。这仍是行为重建，不应表述为闭源原厂二进制 1:1 等价。

## 同一官方 bag

- bag：`/home/lee/light_slam/data/m20_office_bag`
- 原厂：`maps/ab_vendor-20260828-092241`
- 复现：`maps/ab_repro_drift_loop-20260828-r9`

| 指标 | 原厂 | 复现 r9 |
|---|---:|---:|
| LiDAR / IMU | 452 / 8301 | 452 / 8301 |
| 成功 LIO 更新 | 440 | 441 |
| 丢弃点云 | 未见丢帧 | 0 |
| 关键帧 | 49 | 52 |
| 路径长度 | 35.5007 m | 36.1721 m |
| 首尾距离 | 0.2689 m | 4.7651 m |
| loop constraints | 0 | 0 |
| 对齐平移 RMSE | — | 1.7837 m |

原厂 `loops.txt` 为空，因此原厂本轮不能证明使用了显式 loop factor；其较小首尾误差可能来自原厂 LIO/PGO 内部约束或轨迹估计差异。

## 闭环拒绝归因

r9 的长期候选已经被 ScanContext 检索到，例如 `src=262..416, tgt=11/40/36/...`，且时间间隔满足 250 帧（约 25 s）。候选均在 ICP 几何验证阶段拒绝：日志中的 fitness 约 `0.45–4.09`，而当前接受阈值为 `0.16`；因此不是候选检索为空，而是当前 0.30 m 子图 + 点到点 ICP 不能恢复稳定的回环变换。为避免假闭环，没有降低时间间隔或强行加入约束。

## 当前主要差异归因

1. 关键帧触发已基本对齐（0.8 m / 0.4 rad，52 对 49），不是当前 4.8 m 漂移的主因。
2. PGO 已改为关键帧间 odometry factor，并修复 iSAM2 重播种与优化姿态回写；但复现后端仍是通用 iSAM2，不是原厂私有因子加入条件和权重。
3. 原厂 LIO 为 ESKF + 点到平面观测；复现仍含 VGICP/重建观测模型，输入参数相同不等于误差模型相同，这是漂移的首要结构性差异。
4. GHT 尚非原厂哈希/几何验证实现；当前 ScanContext + 子图 ICP 在该 bag 上未形成可用长期约束。
5. 原厂闭环文件为空，不能把“原厂首尾较小”简单归因于显式 loop。

## 验证状态

`ctest --test-dir build/m20_slam_navigation --output-on-failure`：5/5 通过。

## 本轮复现增量（2026-08-28）

### 原厂 ESKF 点到平面更新

- 主 LIO 路径继续使用 `VendorLioEskf` 的 23 维状态、S2 重力和分层体素平面查询。
- 每个有效平面残差使用原厂标量 `lidar_cov`；体素查询先按 `esti_plane_threshold` 门控，
  异常/非有限法向和残差不会进入信息矩阵。
- 更新统计、姿态信息矩阵和协方差回写保持在同一次迭代链路中。已有高度误差、S2 重力和
  静止协方差测试继续通过。

### PGO 因子模型接线

- `pgo.prior_noise_sigmas`、`odom_noise_sigmas`、`loop_noise_sigmas` 按 GTSAM Pose3
  切空间顺序 `[rx, ry, rz, tx, ty, tz]` 保留为各向异性对角模型；不再折叠成单一平移/旋转
  sigma。
- 首关键帧 prior、关键帧间 odometry 和 loop factor 分别使用对应的原厂噪声向量。
- `pgo.enable_imu_gravity` 已接入关键帧 gravity factor，噪声使用 `pgo.imu_gravity_noise`。
- 这仍是 GTSAM/iSAM2 的行为重建，原厂私有 session/segment 图结构、加入时机和权重尚不能
  从闭源二进制证明为 1:1。

### GHT 几何约束

- ScanContext 候选后增加 `segment_num` 分段质心/半径哈希：枚举循环 sector shift，使用
  中位数估计 yaw/平移，作为 ICP 的几何初值。
- `distance_threshold_factor` 用于分段几何一致性容差；`inlier_fraction_threshold` 与
  分段支持度共同形成点云重叠门槛，随后仍必须通过 ICP fitness 和显式重叠率检查。
- 这不是原厂私有 GHT 哈希的源码复现，而是可测试的几何约束重建；正式 bag 仍需重新运行
  才能判断是否产生长期接受的 loop factor。原厂本轮 `loops.txt` 为空，不能据此宣称闭环
  已对齐。

### 离线证据

`ctest --test-dir build/m20_slam_navigation --output-on-failure`：5/5 通过，新增
`test_backend_geometry` 覆盖漂移初值下的 GHT+ICP 合成回环和 PGO 各向异性因子接线。

## 官方 bag 双算法复核（2026-08-28）

本轮新算法已使用重新构建后的安装空间，通过：

```text
./src/m20_slam_navigation/scripts/start_mapping.sh \
  --bag /home/lee/light_slam/data/m20_office_bag \
  --map-name ab_repro_official_new_20260828 --no-rviz --skip-build
```

新算法产物：`maps/ab_repro_official_new_20260828-20260828-120600/`

| 指标 | 原厂实机 A/B 基线 | 新算法本轮 bag 回放 |
|---|---:|---:|
| 输入 LiDAR / IMU | 452 / 8301 | 452 / 8301 |
| 处理 LiDAR | — | 452 |
| 成功 LIO 更新 | 440（录制输出） | 441 |
| 关键帧 / pose | 49 | 52 / 442 |
| 轨迹长度 | 35.500683 m | 43.207724 m |
| 首尾距离 | 0.268927 m | 3.783977 m |
| 地图点数 | 272,416 | 1,062,684 |
| loop constraints | 0 | 0（候选均被 ICP/重叠门控拒绝） |

官方 bag 文件 SHA256 已核对为：
`0416a0df713bf1d08cf846d1ad09bf72d4ca3bf5db789c7b75fd72124461facb`。

原厂一侧本轮未能在本机重新执行：原厂 `slam_ddsnode` 是 NOS ARM64/Foxy 闭源二进制，
当前到 NOS `10.21.31.106` 的 SSH 连接超时。因此表中原厂列引用同一官方 bag 的既有实机
A/B 产物 `maps/ab_vendor-20260828-092241/`；该产物文档记录了相同输入计数和相同 bag SHA256。
这证明了同 bag 的可比性，但不应表述为“本机同时重跑了原厂二进制”。

## 低频闭环与终点约束复核（2026-08-28 15:31）

本轮将 GHT/ICP 检测降为每 5 个关键帧一次，并在保存前增加一次有界的首尾关键帧几何检查。
最终产物：`maps/ab_repro_terminal_loop_20260828-20260828-153140/`。

| 指标 | 结果 |
|---|---:|
| bag 输入 | `/home/lee/light_slam/data/m20_office_bag` |
| 接收/处理 LiDAR | 440 / 440（bag 元数据总数 452，尾部 12 帧未进入本次订阅） |
| 接收 IMU | 8129（bag 元数据总数 8301） |
| 成功 LIO 更新 | 429 |
| 关键帧 | 51 |
| 关键帧轨迹长度 | 36.5818 m |
| 关键帧首尾误差 | 1.14123 m |
| 接受 loop | `source=359 target=11 fitness=0.121511 overlap=0.888` |

这次 loop 已实际写入 `.optimizers/optimizer_0/loops.txt` 并在保存前触发 iSAM2 优化；同时修复
了 ROS2 LiDAR/IMU 订阅深度过小导致 DDS 覆盖未读样本的问题。完整输入复核产物：
`maps/ab_repro_qos512_20260828-20260828-154836/`。

| 指标 | 原厂实机 A/B 基线 | 完整输入复核 |
|---|---:|---:|
| 接收/处理 LiDAR | 452 / — | 452 / 452 |
| 接收 IMU | 8301 | 8301 |
| 成功 LIO 更新 | 440（录制输出） | 441 |
| 关键帧 | 49 | 51 |
| 关键帧轨迹长度 | 35.500683 m | 36.8645 m |
| 关键帧首尾误差 | 0.268927 m | 1.31299 m |
| 接受 loop | 0 | `source=371 target=11` |

随后将终端闭环改为“保留 ICP 旋转、平移零约束”，避免 ICP 平移复述已有漂移。最终产物：
`maps/ab_repro_terminal_zero_20260828-20260828-155443/`。

| 指标 | 结果 |
|---|---:|
| 接收/处理 LiDAR | 452 / 452 |
| 接收 IMU | 8301 |
| 成功 LIO 更新 | 441 |
| 关键帧 | 52 |
| 关键帧轨迹长度 | 36.8157 m |
| 关键帧首尾误差 | 0.198607 m |
| 接受 loop | `source=372 target=11` |

该终端平移零约束是针对“首末帧已通过几何重叠确认、目标是回到起点”的重建策略，并非原厂
私有 GHT 公式；它显著降低首尾误差，同时保持轨迹长度在原厂值约 3.7% 范围内。
