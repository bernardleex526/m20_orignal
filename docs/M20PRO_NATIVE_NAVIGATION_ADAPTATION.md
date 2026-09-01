# M20Pro 原厂导航算法、参数与话题适配

更新时间：2026-08-31

## 目标与边界

`m20_orignal` 现在包含一个原厂导航合同适配层，目标是让同一套输入能够被本机
的全局规划、局部规划和地形代价链路消费，并在 M20Pro 上使用原厂 DrDDS 消息和
服务名。

原厂全局规划器和局部规划器是闭源二进制，仓库中没有私有源码。因此本次实现是
“可观测接口 + 参数 + 行为结构”的适配，不宣称二进制内部源码 1:1 等价。原厂
算法族和本实现的对应关系如下：

| 原厂组件 | 已适配实现 | 适配内容 |
|---|---|---|
| global planner | `HybridAStar` + `SplineOptimizer` | 原厂 Hybrid A*、8 方向/3 转向采样、机身碰撞、地形代价、cubic spline 平滑 |
| local planner | `DWAPlanner` + `LinePlanner` | 原厂 DWA/LinePlanner 模式切换、速度/加速度窗口、M20 矩形机身、目标和障碍代价 |
| passable area | `TraversabilityMap` | 8 m × 8 m 地形窗口、坡度/粗糙度/台阶代价、`pcl_pass_grid` 裁剪和滤波 |
| native transport | `NativeNavigationBridge` | SDK 存在时使用原厂 DrDDS 生成类型；工作站无 SDK 时使用离线 ROS 回退 |

## 原厂话题和服务

原厂 ROS 图中已核对的导航合同如下。`drdds/*` 是 M20Pro SDK 生成的类型，不能用
普通 ROS 消息在同一话题上冒充。

| 名称 | 类型 | 角色 |
|---|---|---|
| `/GRID_MAP` | `nav_msgs/msg/OccupancyGrid` | 全局规划栅格输入 |
| `/ODOM` | `nav_msgs/msg/Odometry` | 位姿和速度输入 |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | ROS 目标输入 |
| `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 初始位姿输入 |
| `/NAV_POINTS` | `sensor_msgs/msg/PointCloud2` | 局部点云输入 |
| `/IMU` | `sensor_msgs/msg/Imu` | 地形组件 IMU 合同 |
| `/accumulate_cloud/cloud_gravity` | `sensor_msgs/msg/PointCloud2` | 重力坐标点云输入 |
| `/path_Astar` | `nav_msgs/msg/Path` | 全局路径 |
| `/local_goal` | `geometry_msgs/msg/PoseStamped` | 局部目标 |
| `/local_map` | `nav_msgs/msg/OccupancyGrid` | 局部地图/代价地图 |
| `/NAV_CMD` | `drdds/msg/NavCmd` | 原厂运动指令输出 |
| `/PLANNER_STATUS` | `drdds/msg/PlannerStatus` | 局部规划状态 |
| `/GLOBAL_PLANNER_STATUS` | `drdds/msg/PlannerStatus` | 全局规划状态 |
| `/MOTION_INFO` | `drdds/msg/MotionInfo` | 实际速度反馈 |
| `/planner_mode` | `drdds/msg/StdMsgInt32` | LinePlanner/DWA 模式选择 |
| `/GOAL_GLOBAL` | `drdds/srv/PoseStampedToInt32` | 全局目标服务 |
| `/GOAL_PLANNER` | `drdds/srv/PoseStampedToInt32` | 局部目标服务 |
| `/CANCEL_NAV_GLOBAL` | `drdds/srv/StdSrvInt32` | 取消全局规划 |
| `/CANCEL_NAV_PLANNER` | `drdds/srv/StdSrvInt32` | 取消局部规划 |
| `/set_service` | `drdds/srv/SetParam` | 局部参数运行时更新 |

调试输出也保留原厂名称，包括 `/global_path`、`/local_path`、`/track_path_baselink`、
`/target_goal`、`/goal_baselink`、`/local_goal_baselink`、`/free_paths`、
`/local_scans`、`/grid_map_3d` 和 `/global_path_markers`。

## 参数适配

`native_navigation.yaml` 把原厂参数展平到 `navigation_node`，并保留原厂的大小写和
尾部下划线，例如 `local.maxSpeedX`、`local.grid_size_`、`local.adjust_yaw_min_`。
原厂全局配置中的 `max_trajectory_range_`、`local_point_dis_`、
`local_point_dis_v_max_`、`max_speed_x_`、`sharp_turn_angle_` 保持原名加载。

关键原厂值包括：

```text
global: weight_a=1.0, weight_b=1.3, weight_heading=0.8
         step_size=0.4, sample_interval=0.2, max_steer=0.9
         bodyLength=0.45, bodyWidth=0.45, goalDis=0.5
         astar_time=1.0, enable_smoothing=true, smooth_count=7
         local_goal_search_window_=10

local:  lookAheadDis=1.5, maxSpeedX=1.5, maxSpeedY=0.6, maxTheta=1.0
         robotLength=0.84, robotWidth=0.5, stop_distance=0.7
         directLine_mode=true, backward_mode=false, path_num=4641
         path_sample_num=10, monitor_loop_frequency=10.0

terrain: map_length=8.0, map_width=8.0, voxel_size=0.05
         max_drop=0.25, max_roughness=0.10, max_slope_deg=89.0
```

`pcl_pass_grid` 在没有原厂 `/GRID_MAP` 时用于 `/NAV_POINTS` 回退链：先按
`out_*` 裁剪，再按 `in_*` 去除机身区域，之后使用 `leaf_size` 和
`serach_radius/minNeighbors` 做体素及半径滤波。收到 `/GRID_MAP` 后，该栅格优先，
不再用点云回退覆盖它。

## 构建和离线验证

```bash
source /opt/ros/humble/setup.bash
cd /home/lee/m20_orignal
colcon build --packages-select m20_slam_navigation \
  --cmake-args -DBUILD_EXPERIMENTAL_NAVIGATION=ON \
               -DCMAKE_BUILD_TYPE=RelWithDebInfo
ctest --test-dir build/m20_slam_navigation --output-on-failure
```

当前离线结果：7/7 测试目标、37 个测试通过，覆盖输入适配、线协议、LIO、输出
合同、后端几何、定位 ESKF 以及导航话题/参数/运动原语/Hybrid A*/LinePlanner
合同。用
`navigation_system.launch.py` 加载安装后的 `native_navigation.yaml`，生命周期
`configure -> activate` 也已验证成功。

## 建图和定位衔接

- 建图默认订阅 `/LIDAR/POINTS`、`/IMU`，发布 `/SLAM_ODOM`、
  `/SLAM_ALIGNED_POINTS`、`/SLAM_CLOUD_REGISTERED_BODY`、`/DEPTH_POINTS`、
  `/DEPTH_IMAGE`、`/SLAM_ACCUMULATED_POINTS_MAP` 和 `/path`，TF 为
  `camera_init -> base_link`。
- 定位默认订阅 `/LIDAR/POINTS`、`/IMU`、`/GPYBM`、`/leg_odom` 和
  `/initialpose`，发布 `/ODOM`、`/FULL_CLOUD_MAP`、`/LOC_BODY_POINTS`，并发布
  `camera_init -> odom` 修正 TF。`/RTK_RAW_ODOM` 保留为原厂输出合同，但工作站
  回退节点不会伪造没有解码来源的 RTK 数据。
- 工作站没有原厂 DrDDS 生成的 RTK 类型，因此这里只配置 `/GPYBM` 合同而不伪造
  类型；该输入必须在带 SDK 的 M20Pro 构建中完成原生端点验收。

## 运行安全和验收边界

- `navigation.enable_motion_output` 默认是 `false`。因此可以先验证 `/path_Astar`、
  `/local_goal`、状态和调试输出，而不写 `/NAV_CMD`。
- 工作站构建没有 `/usr/local` 原厂 DrDDS 开发包，不能在本机编译原生消息桥的
  `M20_HAS_DRDDS` 分支；本机验证的是 ROS 回退、参数加载和规划行为。部署到 M20Pro
  时必须在目标系统重新构建并检查生成的 `drdds` 类型和 DDS QoS。
- 没有启动原厂闭源 global/local 二进制，也没有做真实机器人运动验收；因此当前
  结论是“适配层可构建、可配置、可离线验证”，不是“已替换原厂导航并通过实机运动”。
- `start_mapping.sh` 仍然只启动建图链路。导航需要单独启动，并且在同一机器人上
  先确认原厂服务是否已经占用同名话题、服务或 DDS participant。
