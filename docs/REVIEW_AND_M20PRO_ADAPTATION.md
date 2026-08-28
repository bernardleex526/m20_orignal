# M20Pro SLAM repository review and mapping adaptation

Review date: 2026-08-26

This document records the gap between the initial repository snapshot and a runnable M20Pro
mapping workflow. The primary acceptance target is deliberately narrow: consume the official
M20Pro LiDAR and IMU interfaces, run an isolated 3D mapper, and save a point-cloud map through
one script. It does not certify the experimental localization and autonomous navigation code.

## Official M20Pro contract used by this adaptation

The public M20 development guide describes the following external ROS 2 / DDS interfaces:

- `/LIDAR/POINTS`: `sensor_msgs/msg/PointCloud2`, 10 Hz. By default this is the fused front and
  rear cloud from two 96-line RoboSense LiDARs. On GOS it also requires
  `multicast-relay.service`.
- `/IMU`: `sensor_msgs/msg/Imu`, 200 Hz. The AOS driver has already transformed the axes to the
  `base_link` convention; consumers must not apply the documented raw-chip rotation again.
- `/ODOM`: `nav_msgs/msg/Odometry`, 10 Hz, published by the vendor localization service. The
  mapping-only workflow does not claim or overwrite this topic.
- M20/M20 Pro robot hosts use Ubuntu 20.04 and ROS 2 Foxy. External Foxy and Humble hosts are
  documented as compatible when Fast DDS and a matching `ROS_DOMAIN_ID` are used.
- Vendor mapping runs on NOS. Third-party algorithm development should run on GOS or an external
  host rather than modifying AOS/NOS services.

The official M20 office bag additionally establishes the cloud schema used here:

```text
x, y, z, intensity: float32
ring:                uint16
timestamp:           float64 absolute seconds
frame_id:            lidar_link
```

## High-severity findings in the initial import

1. `ament_auto_package()` was called without `ament_cmake_auto`, so CMake configuration failed.
2. No `slam_node`, `localization_node`, or `navigation_node` executable target was created even
   though launch files attempted to run those names.
3. The launch files started lifecycle nodes without configuring or activating them.
4. RViz conditions were Python lambdas rather than launch conditions, and the referenced RViz
   configuration did not exist.
5. Parameter YAML files were not ROS 2 parameter files (`node_name/ros__parameters` was absent),
   so their values were not applied to nodes.
6. The repository subscribed to `/LIDAR/pointcloud` and described a Livox Mid-360. The M20Pro
   interface is `/LIDAR/POINTS` from fused RoboSense 96-line scanners.
7. The point-cloud callback dereferenced an uninitialized PCL cloud pointer.
8. LiDAR and IMU timestamps were replaced with callback arrival time. This breaks synchronization
   during bag playback and over networks.
9. Point intensity was treated as per-point time, destroying intensity semantics and ignoring the
   actual `timestamp(float64)` field.
10. IMU integration requested the zero-width interval `[scan_stamp, scan_stamp]`, so normal scans
    received no useful IMU trajectory.
11. Registration could never bootstrap: an empty voxel map produced no correspondence, and a
    rejected scan was never inserted.
12. VGICP transformed the source twice in the residual path, mixed a left-perturbation Jacobian
    with a right-multiplication update, referenced non-existent parameter members, and did not
    preserve a valid final Hessian outside the iteration scope.
13. No implementation saved `full_cloud.pcd`; `/map` and `/TRACK_PATH` publishers were declared
    but not populated by the mapping node.
14. The initial TF publication labeled the LIO pose as `map -> odom`, which can conflict with the
    vendor localization chain and does not represent the computed transform.
15. ScanContext candidates were inserted as loop factors without geometric ICP verification.
16. The localization/planning/controller prototype is not build-clean. Examples include invalid
    Eigen accessors, missing types, placeholder controller scoring, and incomplete terrain
    occlusion logic. It is disabled by default with `BUILD_EXPERIMENTAL_NAVIGATION=OFF`.

## Implemented mapping adaptation

- Correct CMake package and ROS 2 component executables.
- C++17 and native Foxy/Humble-compatible package structure.
- Official `/LIDAR/POINTS` and `/IMU` defaults.
- Strict M20 cloud-schema validation and extraction of absolute double timestamps.
- Preservation of intensity and real `lidar_link` input frame.
- Small timestamp rollback guard: rollbacks up to 20 ms are clamped; larger resets are dropped.
- Thread-safe IMU buffering and matching scan start/end integration interval.
- First-scan voxel-map bootstrap.
- Corrected VGICP source-frame residual and left-perturbation update.
- Release build by default; the unoptimized implementation is too slow for the 10 Hz stream.
- Isolated outputs:
  - `/m20_slam/odom`
  - `/m20_slam/path`
  - `/m20_slam/map_cloud`
  - TF `m20_slam_map -> m20_slam_lidar`
- `/m20_slam/save_map` service.
- Saved artifacts:
  - `full_cloud.pcd`
  - `mapping_summary.txt`
  - `trajectory.csv`
- Automatic lifecycle configure/activate in `slam_system.launch.py`.
- Duplicate-instance protection, live-topic preflight, optional bag replay, map save, and controlled
  process-group shutdown in `scripts/start_mapping.sh`.
- Loop constraints disabled by default until geometric verification is implemented.

## Offline evidence

The initial repository failed during CMake configuration with:

```text
Unknown CMake command "ament_auto_package"
```

After adaptation:

- Release build: passed for the mapping target.
- M20 cloud adapter tests: 5 focused gtests, 0 failures.
- Launch argument inspection: passed.
- Official 45.3-second M20 office bag produced continuous keyframes and a saved PCD. A representative
  Release run accepted 439 cloud messages and 8244 IMU messages, produced 104 keyframes, and saved
  19,849 voxel-centroid map points. DDS replay delivery varied slightly between runs; the input bag
  contains 452 clouds, so exact transport accounting remains a live acceptance item.

Offline replay demonstrates message compatibility and execution continuity. It does not establish
correct M20Pro extrinsics, physical scale, map accuracy, long-duration stability, GOS resource
headroom, or safe coexistence with vendor services.

## Remaining hardware acceptance

Before using the result for localization or navigation:

1. Build natively on GOS with the robot's Foxy environment and private DrDDS setup sourced.
2. Confirm `multicast-relay.service`, `/LIDAR/POINTS` at approximately 10 Hz, and `/IMU` at
   approximately 200 Hz.
3. Measure the real fused-cloud-to-IMU/base extrinsic and time offset. Do not use guessed static TF.
4. Verify axes, scale, sensor height, trajectory direction, and map geometry against physical
   references.
5. Record CPU, memory, temperature, dropped frames, processing latency, and queue depth.
6. Test stationary initialization, open space, a long corridor, closed loops, stairs, reflective
   surfaces, and deliberate sensor interruption.
7. Keep vendor navigation and all motion-command bridges disabled during mapping A/B tests.
8. Do not enable the experimental navigation build until its compile errors, placeholders,
   lifecycle behavior, costmaps, TF ownership, and command safety have separate acceptance tests.
