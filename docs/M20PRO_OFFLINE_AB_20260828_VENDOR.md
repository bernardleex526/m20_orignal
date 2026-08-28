# M20Pro 原厂算法官方 bag 离线 A/B 结果

日期：2026-08-28（Asia/Shanghai）

## 测试边界

- 设备：NOS `10.21.31.106`（ARM64/Foxy），经 AOS `192.168.101.36` 跳板连接。
- 未启动运动桥、未发送 `cmd_vel`，未修改活动地图或 `/opt/robot` 原厂文件。
- 原厂 `slam_ddsnode` 在用户命名空间私有 `/dev/shm` 中运行；官方 bag 与原厂节点共享该隔离 DrDDS 域。
- 输入话题隔离为 `/AB/LIDAR/POINTS`、`/AB/IMU`，避免与实时传感器混流。
- 原厂二进制仅将编译时资源根从 `/opt/robot/share/slam/` 重定位到测试副本 `/tmp/m20ab/vendor/`，算法逻辑未改动。

## 输入

- bag：`/home/lee/m20_orignal/maps/ab_vendor-20260828-092241` 对应 NOS `/home/user/codex_vendor_ab_20260828/bag`
- 时长：45.300481545 s
- `/LIDAR/POINTS`：452 帧
- `/IMU`：8301 帧
- DB3 SHA256：`0416a0df713bf1d08cf846d1ad09bf72d4ca3bf5db789c7b75fd72124461facb`

## 原厂结果

输出目录：`maps/ab_vendor-20260828-092241`

| 指标 | 结果 |
|---|---:|
| 原厂 LIO 版本 | 3.4.0 |
| `Save frame` / pose 样本 | 49 |
| `.sessions/session_0/poses.txt` | 5267 bytes，非空 |
| 关键帧 PCD | 49 |
| 原厂录制 `/SLAM_ODOM` | 440 |
| 原厂录制 `/SLAM_ALIGNED_POINTS` | 440 |
| 原厂录制 `/SLAM_ACCUMULATED_POINTS_MAP` | 441 |
| 原厂录制 `/SLAM_KEYFRAME_INFO` | 0（原厂关键帧以会话 pose/PCD 落盘） |
| `/DEPTH_IMAGE`、`/DEPTH_POINTS` | 0（参数 `depth_image.enable=false`） |
| `/HEIGHT_IMAGE`、`/HEIGHT_POINTS` | 0（参数 `height_map.enable=false`） |
| 轨迹长度 | 35.500683 m |
| 首尾距离 | 0.268927 m |
| 轨迹 XYZ 范围 | 9.569784 / 12.721020 / 0.103278 m |
| `full_cloud.pcd` | 272,416 点，4,358,846 bytes |
| `occ_grid.pgm` | 223 × 249，55,570 bytes |
| loop constraints | 0 |

原厂日志包含真实 `Save frame`、`Saving optimized poses`、`Processing label: 0 with 49 frames.`，并成功执行栅格图、完整点云和后处理步骤。

## 与复现算法的同 bag 对比

复现输出：`maps/ab_repro-20260827-173059`。

使用原厂 49 个 pose 时间戳对复现轨迹线性插值，并做 SE(3) Umeyama 刚体对齐：

| 指标 | 结果 |
|---|---:|
| 原厂轨迹长度 / 首尾距离 | 35.500683 / 0.268927 m |
| 复现轨迹长度 / 首尾距离 | 36.890759 / 4.134324 m |
| 关键帧平移误差 RMSE / mean / median / max | 1.560450 / 1.336143 / 1.130350 / 3.652227 m |
| 近邻姿态旋转误差 RMSE / mean / max | 19.616 / 14.321 / 45.279 deg |
| 原厂→复现点云 NN 距离 mean / median / p95 | 0.130868 / 0.071200 / 0.449005 m |
| 复现→原厂点云 NN 距离 mean / median / p95 | 0.194414 / 0.121329 / 0.635648 m |
| 原厂 `full_cloud.pcd` | 272,416 点 |
| 复现 `full_cloud.pcd` | 2,517,969 点 |
| 原厂栅格 | 223 × 249，含 0/128/255 三值 |
| 复现栅格 | 256 × 256，含 0/255 两值 |

结论：本轮已完成同一官方 bag 的真实原厂 A/B。输入传输、原厂 LIO、PGO pose 保存、关键帧 PCD、私有体素树导出的完整点云和 2D 栅格工程均已跑通；轨迹、点云数量/包围盒和栅格语义仍明显不一致，因此当前复现算法尚未达到原厂 1:1。`DEPTH_*`、`HEIGHT_*` 零消息与双方当前禁用参数一致，不应视为测试失败。
