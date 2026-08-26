# M20 Pro — Industrial-Grade 3D SLAM & Autonomous Navigation for Quadruped Robots

**ROS 2 Humble · C++20 · GTSAM 4.2 · PCL 1.12 · Eigen 3.4**

## System Architecture

```
┌──────────────────────────────────────────────────────────────────┐
│                    M20 Quadruped SLAM & Navigation                │
├──────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌─────────────────────────┐    ┌────────────────────────────┐  │
│  │ Module 1: LIO Front-End │───▶│ Module 2: Factor Graph     │  │
│  │  · IMU Pre-integration  │    │  · GTSAM iSAM2             │  │
│  │  · Point Cloud Deskew   │    │  · Gravity Prior Factor    │  │
│  │  · Hash Voxel Map        │    │  · Degeneracy Detection    │  │
│  │  · FastVAGICP Registration│   │  · Loop Closure (ScanCtx)  │  │
│  └─────────────────────────┘    └────────────┬───────────────┘  │
│                                              │                   │
│  ┌───────────────────────────────────────────┼───────────────┐  │
│  │ Module 3: Prior Map Relocalization        │               │  │
│  │  · 3D NDT Scan-to-Map                    │               │  │
│  │  · Multi-Hypothesis Global Search         ▼               │  │
│  │  · ESKF (IMU + Odom + NDT fusion)      ┌──────────┐      │  │
│  └─────────────────────────────────────────│   TF &   │──────┘  │
│                                            │  Pose    │         │
│  ┌─────────────────────────────────────────│  Graph   │──────┐  │
│  │ Module 4: Terrain Traversability        └──────────┘      │  │
│  │  · 2.5D Elevation Grid                                     │  │
│  │  · PCA Slope Analysis                                       │  │
│  │  · Roughness (σ_z)                                         │  │
│  │  · Step Height Detection                                   │  │
│  └─────────────────────────┐                                   │  │
│                            ▼                                   │  │
│  ┌─────────────────────────────────────────────────────────┐  │  │
│  │ Module 5: Global Planner (Omnidirectional Hybrid A*)    │  │  │
│  │  · 8-Directional Motion Primitives + Rotation-in-Place  │  │  │
│  │  · Dijkstra + Non-Holonomic Max Heuristic               │  │  │
│  │  · Cubic Spline Smoothing (κ ≤ κ_max)                   │  │  │
│  └─────────────────────────┬───────────────────────────────┘  │  │
│                            ▼                                   │  │
│  ┌─────────────────────────────────────────────────────────┐  │  │
│  │ Module 6: Local Controller (DWA + LinePlanner)          │◀─┘  │
│  │  · Omnidirectional DWA (vx, vy, ωz)                      │     │
│  │  · LinePlanner Mode (Cross-track P-Control)              │     │
│  │  · Terrain-Cost-Aware Scoring                            │     │
│  └─────────────────────────────────────────────────────────┘     │
│                                                                  │
└──────────────────────────────────────────────────────────────────┘
```

## Module Summary

| Module | Description | Key Algorithm | Target Performance |
|--------|-------------|---------------|-------------------|
| **LIO Front-End** | LiDAR-Inertial Odometry | FastVAGICP (Voxel-Accelerated GICP) | <15ms/scan |
| **Back-End** | Factor Graph Optimization | GTSAM iSAM2 + Gravity Factor + Degeneracy Fallback | Incremental |
| **Relocalization** | Prior Map Localization | 3D NDT + 15-DOF ESKF | <100ms |
| **Terrain** | Traversability Analysis | 2.5D Elevation Grid + PCA Slope + Step Detection | Per-scan |
| **Global Planner** | Path Planning | Omnidirectional Hybrid A* + Cubic Spline | <5s plan |
| **Local Controller** | Velocity Command | DWA (3-DOF sampling) + LinePlanner | 50Hz |

## Prerequisites

```bash
# ROS 2 Humble
sudo apt install ros-humble-desktop

# GTSAM (factor graph optimization)
sudo apt install libgtsam-dev ros-humble-gtsam

# PCL (point cloud library)
sudo apt install libpcl-dev

# Eigen 3
sudo apt install libeigen3-dev
```

## Build

```bash
cd /home/bernardx526/m20pro_orginal_slam
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## Launch

### SLAM Only (mapping)
```bash
ros2 launch m20_slam_navigation slam_system.launch.py
```

### Localization Against Prior Map
```bash
ros2 launch m20_slam_navigation localization_system.launch.py \
    map_path:=/path/to/full_cloud.pcd
```

### Navigation (requires SLAM or Localization running)
```bash
ros2 launch m20_slam_navigation navigation_system.launch.py
```

### Full System (SLAM + Navigation)
```bash
ros2 launch m20_slam_navigation m20_full_system.launch.py
```

## ROS 2 Topic Interface

### Subscribed Topics
| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/LIDAR/pointcloud` | `PointCloud2` | 10-20Hz | Livox Mid-360 raw scan |
| `/IMU` | `Imu` | 200Hz+ | BMI088 / ICM-20948 |
| `/ODOM` | `Odometry` | 50-100Hz | AOS leg kinematics |
| `/initialpose` | `PoseWithCovarianceStamped` | Event | RViz 2D pose estimate |
| `/goal_pose` | `PoseStamped` | Event | Navigation goal |

### Published Topics
| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/tf` | `TFMessage` | 50Hz | map→odom→base_link→lidar_link |
| `/map` | `OccupancyGrid` | 1Hz | 2D grid map |
| `/TERRAIN_TRAVERSABILITY_MAP` | `OccupancyGrid` | 2Hz | Traversability costmap |
| `/TRACK_PATH` / `/plan` | `Path` | On update | Global/local path |
| `/cmd_vel` | `Twist` | 50Hz | Velocity command |

## Key Design Decisions for Quadruped Robots

1. **Motion Distortion Compensation**: Quadruped trotting induces 5-10cm body oscillation within a single scan. IMU-based deskewing is critical.

2. **Omnidirectional Planning**: Quadrupeds can move sideways — Hybrid A* uses 8-directional translation + rotation-in-place primitives.

3. **Degeneracy Fallback**: In long corridors, LiDAR loses longitudinal observability. The system detects this via Hessian eigenvalue decomposition and falls back to foot odometry + IMU dead reckoning for the degenerate DoF.

4. **Terrain-Aware Control**: DWA scoring includes a terrain traversability term (weight 4.0) to avoid steep slopes, rough terrain, and large steps that exceed the robot's kinematic limits.

5. **Gravity-Constrained Optimization**: GTSAM factor graph includes a gravity prior factor that constrains roll/pitch drift, isolating free optimization to (x, y, z, yaw).

## File Structure

```
m20pro_orginal_slam/
├── README.md
└── src/
    └── m20_slam_navigation/
        ├── CMakeLists.txt
        ├── package.xml
        ├── config/
        │   ├── sensors.yaml
        │   ├── lio_params.yaml
        │   ├── backend_params.yaml
        │   ├── localization_params.yaml
        │   ├── terrain_params.yaml
        │   ├── planner_params.yaml
        │   └── controller_params.yaml
        ├── launch/
        │   ├── slam_system.launch.py
        │   ├── localization_system.launch.py
        │   ├── navigation_system.launch.py
        │   └── m20_full_system.launch.py
        ├── include/m20_slam_navigation/
        │   ├── common/   (types, math_utils, thread_safe_queue, params)
        │   ├── lio/      (imu_processor, deskewer, voxel_map, fast_vgicp, lio_odometry)
        │   ├── backend/  (factor_graph, degeneracy_detector, loop_closure, pose_graph_optimizer)
        │   ├── localization/ (prior_map_loader, ndt_matcher, eskf, relocalizer)
        │   ├── terrain/  (elevation_grid, slope_analyzer, roughness_analyzer, step_detector, traversability_map)
        │   ├── planning/ (motion_primitives, hybrid_astar, spline_optimizer, global_planner_node)
        │   └── control/  (dwa_planner, line_planner, local_controller_node)
        └── src/
            ├── common/   (math_utils.cpp, params.cpp)
            ├── lio/      (5 .cpp files)
            ├── backend/  (4 .cpp files)
            ├── localization/ (4 .cpp files)
            ├── terrain/  (5 .cpp files)
            ├── planning/ (4 .cpp files)
            ├── control/  (3 .cpp files)
            └── nodes/    (slam_node.cpp, localization_node.cpp, navigation_node.cpp)
```

## License

MIT License — See LICENSE file.