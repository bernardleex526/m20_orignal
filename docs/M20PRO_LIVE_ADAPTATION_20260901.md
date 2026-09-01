# M20Pro V1.1.8.7 真机适配记录（2026-09-01）

## 结论

本仓库的输入合同与实机配置一致：融合点云 `/LIDAR/POINTS`、IMU `/IMU`、DDS
domain 0。实时算法应部署在 NOS `10.21.31.106`，通过 AOS
`192.168.101.36` 跳转；外部 PC 与 AOS/GOS 在本轮检查中都没有发现实时点云发布端。

本轮没有启动导航、发布 `/NAV_CMD`、移动机器人、重启厂商服务或修改 `/opt/robot`。

## 三板证据

| 主机 | 地址 | 本轮观察 |
|---|---|---|
| AOS | `192.168.101.36`, `10.21.31/32/33.103` | `/IMU` 有 publisher；无 `/LIDAR/POINTS` publisher |
| GOS | `10.21.31.104` | `hsLidar` 运行；未发现点云 publisher |
| NOS | `10.21.31.106`, `10.21.33.106` | 原厂定位/规划运行；原生 DrDDS 对 `/LIDAR/POINTS` 为 `matched=2 updated=no`，对 `/IMU` 为 `matched=1 updated=yes` |

两台雷达 `10.21.33.201`、`10.21.33.202` 从 NOS 均可达，驱动监听 UDP 2361/2362，
但 `/LIDAR/STATUS` 同样没有 publisher。ROS 2 图在不同时刻对点云 publisher 的
可见性不稳定，而原生 DrDDS 已证明端点匹配但没有新样本；由此只能证明网口、设备
IP、DDS 端点与驱动进程存在，不能证明点云帧已经从驱动进入算法。

## 原厂 `drmap mapping` 实测

`drmap mapping -s -b -n codex_oem_probe` 最终调用
`systemd-run mapping.service -> start_dds.sh -> chrt 49 taskset -c 4,5,6,7
slam_ddsnode <map_dir> indoor`。启动前停止 `localization`，结束时由 `slam_command`
请求保存，再恢复 `localization` 和 `planner`。视觉 RTK 脚本缺少
`/home/user/fibocom_ws/install/setup.bash`，因此显示不可用，但不阻止激光建图。

两轮静止探测均由原厂进程收到点云并保存 frame 0，分别产出 `full_cloud.pcd`、
`occ_grid.pgm/yaml`、poses、IMU 姿态与优化器目录；使用 `-b` 后 `active` 软链接保持
指向原地图。第二轮地图为
`codex_ros2_direct_probe-20260901-140959`，`full_cloud.pcd` 为 83898 字节。

关键边界：原厂 `slam_ddsnode` 收到点云时，ROS 2 CLI 的 RELIABLE 订阅以及旧版独立
DrDDS receiver 都没有收到 `/LIDAR/POINTS`。厂商 ROS 2 示例使用 BEST_EFFORT，且把
话题写成 `/LIDAR_POINTS`；由于测试到达时第二轮 mapping 已结束，这两个变量仍需在
下一次短窗口中验证，不能把 ROS 图可见性当成原厂数据链证据。

## 安全适配

- 默认输出为 `/m20_slam/*`，`publish_tf=false`；不会覆盖原厂 `/SLAM_*` 或
  `camera_init -> base_link`。
- `start_mapping.sh` 只构建 mapping 目标，使用
  `BUILD_EXPERIMENTAL_NAVIGATION=OFF`；不会启动定位、规划、控制或 Nav2。
- 不能在 ROS 2 `slam_node` 内直接加载本机 `libdrdds.so.1`：实机出现
  `Cannot serialize ParticipantProxyData` 后 SIGSEGV，说明 ROS 2 Foxy 自带 FastDDS
  与厂商 FastDDS 2.14 存在同进程 ABI/全局状态冲突。实时 helper 因此必须是纯原生
  DrDDS 独立进程；它复用系统库和 QoS，不实现或重发第二套 DDS，只通过本地 socket
  隔离 ABI。离线 bag 才使用 ROS 2 订阅。
- 只有显式 `--takeover-vendor-outputs` 才启用原厂输出话题和 TF，并检查
  `slam_ddsnode` 是否仍在运行。

## 部署与运行

从开发机同步源码到 NOS：

```bash
rsync -a --exclude .git --exclude build --exclude install --exclude log \
  --exclude maps --exclude data -e "ssh -J user@192.168.101.36" \
  /home/lee/m20_orignal/ user@10.21.31.106:/home/user/m20_orignal/
```

在 NOS 构建 mapping-only：

```bash
source /opt/robot/scripts/setup_ros2.sh
cd /home/user/m20_orignal
colcon build --symlink-install --packages-select m20_slam_navigation \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_EXPERIMENTAL_NAVIGATION=OFF
```

点云恢复后，由操作者在交互终端运行（sudo 密码只在本机输入）：

```bash
cd /home/user/m20_orignal
./src/m20_slam_navigation/scripts/start_mapping.sh \
  --map-name site_a --no-rviz --skip-build
```

随后必须分别核验 DrDDS 的实际回调计数、LIO 更新、`/m20_slam/odom` 频率、地图
文件与现场几何质量；仅有进程、匹配计数或话题名称不算真机建图通过。

## 本轮验证结果

- x86/Humble：完整构建（含导航模块）成功，7/7 CTest 通过。
- NOS ARM64/Foxy：mapping-only 构建成功，5/5 CTest 通过；接收器链接到原厂
  `libdrdds.so.1` 和 FastDDS 2.14。
- NOS 原厂两次建图均收到并保存一帧融合点云，证明原厂数据链正常。
- 旧版独立 receiver 的 NOS 预检是 LiDAR `matched=2 updated=no`、IMU
  `matched=1 updated=yes`；该重复连接器已从实时启动路径移除。
- 进程内 DrDDS 版本虽通过编译和单测，但 NOS 实跑在 Participant 公告后 SIGSEGV，
  已回退。纯原生 helper 使用默认 UDP 后仍是 LiDAR `matched=2 updated=no`，因此实时
  原始点云接入尚未通过。
- 第三次原厂窗口确认 `/SLAM_ODOM` 是 RELIABLE/volatile 的
  `nav_msgs/msg/Odometry`，约 10 Hz，`frame_id=map`；`/path` 实际是 RELIABLE/
  volatile 的 `sensor_msgs/msg/PointCloud2`，并非 `nav_msgs/msg/Path`；静止时
  `/path` 与 `/SLAM_KEYFRAME_INFO` 没有新样本。原厂运行期间，外部 native DrDDS、
  ROS 2 BEST_EFFORT/RELIABLE 以及 `/LIDAR_POINTS`/`/LIDAR/POINTS` 组合仍全部无点云。
- 新增 `m20_vendor_map_postprocessor`：直接导入原厂 `poses.txt` 和关键帧 PCD，复用
  本仓库 ScanContext/GHT/ICP 与 GTSAM/iSAM2 后端，不创建 DDS 或话题。本轮 1 帧
  实包导入成功：1/1 位姿、28581 点、0 闭环、起终点距离 0；这只验证格式和执行链，
  不构成多帧 PGO 或现场几何验收。相同输入已在 NOS ARM64/Foxy 实机执行成功，
  输出位于 `/home/user/m20_orignal/maps/codex_vendor_output_probe_backend`。
