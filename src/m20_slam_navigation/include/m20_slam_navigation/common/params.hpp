#pragma once
/**
 * @file params.hpp
 * @brief Centralized parameter structures loaded from YAML configuration.
 *
 * Each sub-system declares its own parameter struct to isolate configs.
 * ROS 2 node wrappers populate these from parameter server / YAML files.
 */

#include "m20_slam_navigation/common/types.hpp"

#include <array>
#include <cstddef>
#include <string>

namespace m20 {

// =============================================================================
// Sensor calibration parameters
// =============================================================================
struct SensorParams {
  // LiDAR
  int    lidar_scan_lines{16};        ///< number of rings (e.g. Livox Mid-360: non-repetitive)
  Scalar lidar_min_range{0.1};        ///< [m]
  Scalar lidar_max_range{200.0};      ///< [m]
  Scalar lidar_hz{10.0};              ///< scan frequency

  // IMU
  Scalar imu_hz{200.0};               ///< IMU output rate
  Scalar imu_accel_noise{0.01};       ///< accelerometer white noise σ [m/s²/√Hz]
  Scalar imu_gyro_noise{0.0001745};   ///< gyroscope white noise σ [rad/s/√Hz] (≈0.01°/s)
  Scalar imu_accel_bias_rw{0.0002};   ///< accel bias random walk [m/s³/√Hz]
  Scalar imu_gyro_bias_rw{0.000003};  ///< gyro bias random walk [rad/s²/√Hz]

  // Vendor extrinsics use T_B_I and T_B_L.  The transform consumed by the
  // LIO state/deskewer is T_I_L = inverse(T_B_I) * T_B_L.
  SE3Pose T_body_imu;
  SE3Pose T_body_lidar;
  SE3Pose T_lidar_imu;  ///< derived LiDAR-in-IMU transform, never loaded directly

  // Foot odometry
  Scalar odom_hz{100.0};
  Scalar odom_slip_ratio{0.02};       ///< expected slip ratio on normal terrain
};

inline SE3Pose composeVendorLidarInImuExtrinsic(
    const SE3Pose& T_body_imu, const SE3Pose& T_body_lidar) {
  return T_body_imu.inverse() * T_body_lidar;
}

// =============================================================================
// LIO Front-End parameters.  Names and defaults mirror the M20Pro dr_lio
// 3.4.0 configuration where the vendor contract is public.
// =============================================================================
struct LIOParams {
  Scalar voxel_size{0.16};            ///< [m] vendor level-2 map resolution
  bool   enable_downsample{true};
  Scalar downsample_leaf_size{0.15};  ///< vendor leaf_size [m]
  Scalar leaf_size_body{0.05};        ///< vendor body/output cloud leaf size [m]
  int    point_stride{5};             ///< vendor skip_num
  int    max_lidar_queue_size{3};     ///< bound latency by discarding stale scans
  int    max_voxels{100000};           ///< max voxels in map
  Scalar keyframe_distance{0.8};      ///< [m] native office-bag spacing is about 0.8 m
  Scalar keyframe_angle{0.4};         ///< [rad] native turn keyframes appear near 0.4 rad

  // Vendor iterated point-to-plane ESKF observation model
  int   max_iterations{3};
  Scalar esti_plane_threshold{0.1};   ///< [m] maximum accepted point-plane residual
  Scalar lidar_cov{0.001};            ///< scalar LiDAR observation covariance
  int deepest_level{2};
  int plane_level{2};
  int top_level{1};
  bool extrinsic_est_en{false};

  // IMU integration
  Scalar imu_integration_dt{0.005};   ///< [s] integration time step
  Scalar init_time{0.1};              ///< [s] stationary IMU initialization window
  int imu_init_samples{200};          ///< vendor MAX_INI_COUNT
  Scalar acc_cov{0.5};
  Scalar gyr_cov{0.5};
  Scalar b_acc_cov{0.001};
  Scalar b_gyr_cov{0.001};

  // Gravity alignment
  Eigen::Matrix<Scalar, 3, 1> gravity{0.0, 0.0, -9.81007};
};

// =============================================================================
// Back-End Factor Graph parameters (GTSAM iSAM2)
// =============================================================================
struct BackendParams {
  Scalar lio_odom_noise_trans{0.1};          ///< [m] relative translation noise σ
  Scalar lio_odom_noise_rot{0.05};            ///< [rad] relative rotation noise σ
  Scalar gravity_noise_sigma{0.01};           ///< gravity factor noise [m/s²]
  Scalar loop_closure_noise_trans{0.5};
  Scalar loop_closure_noise_rot{0.1};
  Scalar loop_matching_error_threshold{0.16};
  Scalar loop_inlier_fraction_threshold{0.95};
  Scalar loop_max_search_distance{8.0};
  Scalar loop_min_submap_overlap{0.65};
  Scalar loop_icp_max_correspondence{2.0};
  int    loop_submap_radius{3};
  int    loop_max_candidates{8};
  int    loop_min_frame_separation{250};
  int    loop_detection_stride{5};
  bool   enable_loop_closure{false};

  // Native PGO noise vectors use Pose3 tangent ordering [rx, ry, rz, tx, ty, tz].
  // Keep the vectors intact instead of collapsing them to one isotropic sigma.
  std::array<Scalar, 6> prior_noise_sigmas{{1.0e6, 1.0e4, 0.001, 0.01, 0.01, 0.01}};
  std::array<Scalar, 6> odom_noise_sigmas{{0.01, 0.01, 0.01, 0.01, 0.01, 0.01}};
  std::array<Scalar, 6> loop_noise_sigmas{{0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001}};
  std::array<Scalar, 6> prior_noise_default_sigmas{{1.0e-6, 1.0e-6, 1.0e-6, 1.0e-7, 1.0e-7, 1.0e-7}};
  std::array<Scalar, 3> gps_noise_precision{{1.0, 1.0, 0.2}};
  bool enable_imu_gravity{true};
  std::array<Scalar, 3> imu_gravity_noise{{0.1, 0.1, 0.1}};
  Scalar distance_threshold_factor{0.03};
  int segment_num{15};
  Scalar keyframe_time{60.0};

  // Degeneracy detection
  Scalar degeneracy_threshold{10.0};          ///< λ_min/λ_max < 1/threshold if degenerate
  Scalar degeneracy_heading_align{0.9};       ///< min dot product for heading alignment (cos(25°))
  bool   enable_degeneracy_fallback{true};

  // iSAM2
  int    isam2_relinearize_skip{1};
  Scalar isam2_wildfire_threshold{0.001};
};

// =============================================================================
// Prior Map Localization parameters
// =============================================================================
struct LocalizationParams {
  std::string map_path{"full_cloud.pcd"};     ///< path to prior map
  Scalar map_voxel_leaf_size{0.2};            ///< [m] downsampling leaf for static map

  // NDT matching
  Scalar ndt_resolution{1.0};                 ///< [m] NDT grid resolution
  int    ndt_max_iterations{30};
  Scalar ndt_step_size{0.1};
  Scalar ndt_epsilon{1e-4};
  Scalar ndt_outlier_ratio{0.55};

  // Multi-hypothesis relocalization
  int    num_hypotheses{8};                   ///< number of initial pose hypotheses
  Scalar hypothesis_trans_range{5.0};         ///< [m]
  Scalar hypothesis_rot_range{3.14159265358979323846}; ///< [rad] full 360°

  // ESKF
  Scalar eskf_accel_noise{0.1};               ///< process noise accel
  Scalar eskf_gyro_noise{0.01};               ///< process noise gyro
  Scalar eskf_accel_bias_noise{0.0001};
  Scalar eskf_gyro_bias_noise{0.000001};
  Scalar eskf_ndt_pos_noise{0.1};             ///< observation noise for NDT position
  Scalar eskf_ndt_rot_noise{0.05};            ///< observation noise for NDT rotation
  Scalar eskf_odom_pos_noise{0.02};           ///< observation noise for foot odometry
};

// =============================================================================
// Terrain Traversability Analysis parameters
// =============================================================================
struct TerrainParams {
  Scalar grid_resolution{0.05};               ///< [m] elevation grid cell size
  Scalar max_range{30.0};                     ///< [m] max LiDAR range for terrain
  Scalar min_range{0.3};                      ///< [m] min range (self-filter)

  // Slope
  Scalar max_climb_angle{0.5235987755982988}; ///< [rad] ≈ 30°
  Scalar slope_weight{1.0};

  // Roughness
  Scalar roughness_threshold{0.1};            ///< [m] σ_z threshold
  Scalar roughness_weight{0.8};

  // Step height
  Scalar max_step_height{0.20};               ///< [m] max traversable positive step
  Scalar max_step_depth{0.15};                ///< [m] max traversable negative step
  Scalar step_weight{1.5};

  // Occlusion / ray visibility
  bool   enable_occlusion_check{true};
  Scalar occlusion_weight{0.5};

  // Slope estimation via PCA: radius for neighbour search
  Scalar normal_estimation_radius{0.2};       ///< [m]
};

// =============================================================================
// Global Planner (Hybrid A*) parameters
// =============================================================================
struct GlobalPlannerParams {
  Scalar grid_resolution{0.1};                ///< [m] planning grid cell size

  // Motion primitives (omnidirectional)
  int    num_heading_bins{72};                ///< discretized headings (72 → 5° bins)
  Scalar max_curvature{0.5};                  ///< [1/m] max path curvature κ_max
  Scalar primitive_length{0.3};               ///< [m] motion primitive step

  // Search
  Scalar heuristic_weight{2.0};               ///< inflated heuristic for speed
  Scalar time_limit_sec{5.0};                 ///< max planning time
  int    max_expansions{100000};

  // Spline smoothing
  Scalar spline_smoothness_weight{0.4};       ///< λ in cubic spline optimization
};

// =============================================================================
// Local Planner/Controller (DWA) parameters
// =============================================================================
struct LocalControllerParams {
  // Velocity space
  Scalar max_linear_vel_x{1.0};              ///< [m/s] max forward
  Scalar max_linear_vel_y{0.5};              ///< [m/s] max lateral (omnidirectional)
  Scalar max_angular_vel{1.0};              ///< [rad/s] max yaw rate
  Scalar max_linear_accel_x{0.5};           ///< [m/s²]
  Scalar max_linear_accel_y{0.3};           ///< [m/s²]
  Scalar max_angular_accel{1.5};            ///< [rad/s²]

  // Sampling
  int    num_vx_samples{20};
  int    num_vy_samples{10};
  int    num_omega_samples{20};
  Scalar sim_time{2.0};                     ///< [s] forward simulation time
  Scalar sim_dt{0.1};                        ///< [s] simulation step

  // DWA objective weights
  Scalar weight_path_clearance{3.0};
  Scalar weight_goal_align{2.0};
  Scalar weight_velocity_progress{1.0};
  Scalar weight_terrain_cost{4.0};
  Scalar weight_smoothness{0.5};

  // LinePlanner mode
  Scalar lineplanner_lateral_gain{0.5};      ///< P gain for lateral drift compensation
  Scalar lineplanner_yaw_threshold{0.1};     ///< [rad] threshold to switch to DWA
};

}  // namespace m20
