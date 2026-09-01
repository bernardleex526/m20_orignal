#pragma once
/**
 * @file params.hpp
 * @brief Centralized parameter structures loaded from YAML configuration.
 *
 * Each sub-system declares its own parameter struct to isolate configs.
 * ROS 2 node wrappers populate these from parameter server / YAML files.
 */

#include "m20_slam_navigation/common/types.hpp"
#include "m20_slam_navigation/common/native_navigation.hpp"

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
  Scalar imu_gravity{9.80511};                 ///< native localization gravity magnitude
};

/** Observable topic contract of the native M20Pro localization service. */
struct LocalizationTopics {
  std::string input_lidar{"/LIDAR/POINTS"};
  std::string input_imu{"/IMU"};
  std::string input_rtk{"/GPYBM"};
  std::string leg_odom{"/leg_odom"};
  std::string initial_pose{"/initialpose"};
  std::string output_odom{"/ODOM"};
  std::string output_enu{"/RTK_RAW_ODOM"};
  std::string output_global_map{"/FULL_CLOUD_MAP"};
  std::string output_body_cloud{"/LOC_BODY_POINTS"};
};

/** Frames and switches used by the native localization contract adapter. */
struct LocalizationRuntimeParams {
  LocalizationTopics topics;
  std::string world_frame{"camera_init"};
  std::string map_frame{"map"};
  std::string odom_frame{"odom"};
  std::string body_frame{"base_link"};
  std::string lidar_frame{"lidar_link"};
  std::string imu_frame{"imu_link"};
  bool publish_world_to_odom_tf{true};
};

// =============================================================================
// Terrain Traversability Analysis parameters
// =============================================================================
struct TerrainParams {
  Scalar grid_resolution{0.05};               ///< [m] elevation grid cell size
  // Native passable_area map geometry.
  Scalar map_length{8.0};                      ///< [m] native rolling map length
  Scalar map_width{8.0};                       ///< [m] native rolling map width
  Scalar map_height_min{-1.0};
  Scalar map_height_max{0.8};
  Scalar voxel_size{0.05};
  Scalar max_drop{0.25};
  Scalar max_roughness{0.10};
  Scalar max_slope_deg{89.0};
  int    max_inpaint_pixels{200};
  bool   enable_center_padding{true};
  Scalar center_dist_thresh{0.8};
  bool   enable_blind_check{false};
  bool   treat_nan_as_stiff{true};

  // Native passable_area topic/frame contract.
  std::string accumulate_cloud_topic{native::kAccumulatedCloudTopic};
  std::string imu_topic{native::kImuTopic};
  std::string passable_cloud_topic{native::kPassableCloudTopic};
  std::string impassable_cloud_topic{native::kImpassableCloudTopic};
  std::string grid_map_topic{native::kGridMapLowerTopic};
  std::string traversal_cost_topic{native::kTraversalCostTopic};
  std::string world_frame{"camera_init"};
  std::string gravity_frame{"base_gravity"};
  std::string body_frame{"base_link"};
  std::string used_frame{"base_gravity"};
  std::string dog_model{"m20"};

  // Native raycast/elevation options.
  bool   raycast_enable{true};
  Scalar raycast_max_ray_distance{4.0};
  Scalar raycast_max_nan_gap{3.0};
  bool   elevation_use_histogram_solver{false};
  int    elevation_histogram_bins{20};
  bool   elevation_region_enabled{true};
  Scalar elevation_region_min_x{-3.0};
  Scalar elevation_region_max_x{3.0};
  Scalar elevation_region_min_y{-1.0};
  Scalar elevation_region_max_y{1.0};

  // Native planner pcl_pass_grid.yaml.  These values are applied to the
  // /NAV_POINTS fallback before terrain analysis; /GRID_MAP remains the
  // authoritative native global-planner input when it is available.
  Scalar pass_grid_out_min_x{-4.0};
  Scalar pass_grid_out_max_x{4.0};
  Scalar pass_grid_out_min_y{-4.0};
  Scalar pass_grid_out_max_y{4.0};
  Scalar pass_grid_out_min_z{-0.30};
  Scalar pass_grid_out_max_z{0.6};
  Scalar pass_grid_in_min_x{-0.45};
  Scalar pass_grid_in_max_x{0.45};
  Scalar pass_grid_in_min_y{-0.25};
  Scalar pass_grid_in_max_y{0.25};
  Scalar pass_grid_in_min_z{-0.30};
  Scalar pass_grid_in_max_z{0.6};
  Scalar pass_grid_search_radius{0.1};
  int    pass_grid_min_neighbors{4};
  Scalar pass_grid_leaf_size{0.01};

  // Native traversal-cost options.
  bool   traversal_cost_enable{true};
  Scalar traversal_slope_free_deg{10.0};
  Scalar traversal_slope_block_deg{80.0};
  Scalar traversal_rough_free{0.01};
  Scalar traversal_rough_block{0.04};
  Scalar traversal_step_free{0.08};
  Scalar traversal_step_block{0.35};
  Scalar traversal_slope_weight{0.3};
  Scalar traversal_roughness_weight{0.3};
  Scalar traversal_step_weight{0.4};
  Scalar traversal_easy_cost{0.0};
  Scalar traversal_hard_cost{90.0};
  Scalar traversal_max_cost{100.0};
  Scalar traversal_missing_cost{20.0};
  Scalar traversal_curve_power{1.0};
  Scalar traversal_low_cost_filter_ratio{0.1};
  int    traversal_terrain_sample_window{1};
  Scalar traversal_safe_zone_side_length{0.6};

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

  // Slope estimation via PCA: radius for neighbour search
  Scalar normal_estimation_radius{0.2};       ///< [m]
};

// =============================================================================
// Global Planner (Hybrid A*) parameters
// =============================================================================
struct GlobalPlannerParams {
  Scalar grid_resolution{0.1};                ///< [m] planning grid cell size

  // Native HybridAstar cost and search model.
  Scalar weight_a{1.0};
  Scalar weight_b{1.3};
  Scalar weight_heading{0.8};
  Scalar cost_steer{1.0};
  Scalar cost_steerchange{0.2};
  Scalar cost_gear{2.0};
  Scalar cost_backward{1.5};
  Scalar cost_reduce{0.85};
  Scalar step_size{0.4};
  Scalar sample_interval{0.2};
  Scalar body_length{0.45};
  Scalar body_width{0.45};
  Scalar max_steer{0.9};
  int    num_steerind{3};
  int    num_directions{8};
  int    num_rotations{3};
  Scalar goal_dis{0.5};
  bool   dynamic_update{false};
  bool   test_mode{false};
  Scalar xy_tolerance{0.5};
  Scalar dynamic_map_size{3.0};
  int    dynamic_map_grid{30};
  Scalar astar_time{1.0};
  int    debug_point_num{7};
  bool   enable_smoothing{true};
  int    smooth_count{7};
  int    smooth_degree{3};
  int    num_samples{7};
  Scalar angle_threshold_deg{60.0};
  Scalar local_goal_freq_hz{10.0};
  Scalar max_trajectory_range{2.0};
  Scalar local_point_dis{0.0};
  Scalar local_point_dis_v_max{0.0};
  Scalar native_max_speed_x{1.0};
  Scalar sharp_turn_angle_deg{45.0};
  int    local_goal_search_window{10};

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
  // Native localPlanner parameter names and behavior.
  Scalar look_ahead_dis{1.5};
  Scalar native_max_speed_x{1.5};
  Scalar native_max_speed_y{0.6};
  Scalar native_max_theta{1.0};
  Scalar barking_deceleration{1.0};
  Scalar stop_distance{0.7};
  Scalar max_v_threshold{1.2};
  Scalar robot_length{0.84};
  Scalar robot_width{0.5};
  Scalar sensor_offset_x{0.0};
  Scalar sensor_offset_y{0.0};
  Scalar obstacle_height_threshold{0.10};
  Scalar goal_in_obstacle_distance{0.25};
  Scalar goal_in_obstacle_z{0.5};
  int    point_per_path_threshold{1};
  std::string input_source{native::kNavPointsTopic};
  Scalar grid_size{0.05};
  int    grid_occupied_number{2};
  bool   enable_cloud_stacking{false};
  int    cloud_stack_size{1};
  bool   direct_line_mode{true};
  bool   judge_close{false};
  bool   change_close{true};
  bool   backward_mode{false};
  bool   judge_turn{false};
  Scalar xy_tolerance{0.1};
  Scalar yaw_tolerance{0.1};
  int    block_number{1};
  Scalar dwa_fine_tune_distance{0.5};
  Scalar weight_goal{1.0};
  Scalar weight_yaw{0.5};
  Scalar weight_spdy{0.2};
  Scalar weight_ob1{0.6};
  Scalar weight_ob2{0.8};
  Scalar weight_ob3{1.0};
  Scalar local_point_dis{3.0};
  Scalar local_point_dis_v_max{5.0};
  bool   local_try{true};
  int    path_num{4641};
  int    path_sample_num{10};
  Scalar grid_voxel_size{0.05};
  Scalar grid_voxel_offset_x{-1.475};
  Scalar grid_voxel_offset_y{-1.975};
  int    grid_voxel_num_x{90};
  int    grid_voxel_num_y{80};
  Scalar turn_yaw_kp{1.5};
  Scalar angle_threshold_deg{30.0};
  Scalar sum_angle_threshold_deg{75.0};
  Scalar min_segment_length{0.02};
  Scalar track_distance{1.5};
  Scalar speed_ratio{1.0};
  Scalar speed_ratio_yaw{1.0};
  Scalar dl_min_yaw{0.7};
  Scalar dl_min_x{0.25};
  Scalar dl_min_y{0.25};
  Scalar pl_min_yaw{0.25};
  Scalar pl_min_x{0.25};
  Scalar pl_min_y{0.25};
  Scalar dl_line_yaw_min{0.2};
  Scalar close_deceleration{1.0};
  Scalar proximity_distance{0.5};
  Scalar warning_threshold{10.0};
  Scalar velocity_accel_threshold{1.0};
  Scalar x_acc_increment{0.05};
  int    count_num{10};
  Scalar only_rotate_yaw{0.35};
  Scalar adjust_yaw_min{1.0};
  Scalar rotate_detect_dis{0.0};
  Scalar turn_approach_threshold{0.2};
  Scalar monitor_loop_frequency{10.0};
  Scalar sharp_turn_angle_deg{60.0};

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

/** ROS topic names used by the native navigation graph. */
struct NavigationTopics {
  std::string grid_map{native::kGridMapTopic};
  std::string initial_pose{native::kInitialPoseTopic};
  std::string goal_pose{native::kGoalPoseTopic};
  std::string goal_global_service{native::kGoalGlobalService};
  std::string odom{native::kOdomTopic};
  std::string motion_info{native::kMotionInfoTopic};
  std::string nav_points{native::kNavPointsTopic};
  std::string cancel_nav{native::kCancelNavTopic};
  std::string cancel_global_service{native::kCancelGlobalService};
  std::string planner_mode{native::kPlannerModeTopic};
  std::string astar_path{native::kAstarPathTopic};
  std::string visible_points{native::kVisiblePointsTopic};
  std::string pruned_visible_points{native::kPrunedVisiblePointsTopic};
  std::string local_goal{native::kLocalGoalTopic};
  std::string local_map{native::kLocalMapTopic};
  std::string global_planner_status{native::kGlobalPlannerStatusTopic};
  std::string nav_cmd{native::kNavCmdTopic};
  std::string planner_status{native::kPlannerStatusTopic};
  std::string target_goal{native::kTargetGoalTopic};
  std::string goal_baselink{native::kGoalBaseLinkTopic};
  std::string local_goal_baselink{native::kLocalGoalBaseLinkTopic};
  std::string free_paths{native::kFreePathsTopic};
  std::string local_path{native::kLocalPathTopic};
  std::string local_scans{native::kLocalScansTopic};
  std::string track_path_baselink{native::kTrackPathBaseLinkTopic};
  std::string global_path{native::kGlobalPathTopic};
  std::string grid_map_3d{native::kGridMap3DTopic};
  std::string global_path_markers{native::kGlobalPathMarkersTopic};
  std::string set_param_service{native::kSetParamService};
  std::string cancel_planner_service{native::kCancelPlannerService};
  std::string goal_planner_service{native::kGoalPlannerService};
};

/** Node-level switches which keep navigation observable without enabling motion. */
struct NavigationRuntimeParams {
  NavigationTopics topics;
  std::string world_frame{"camera_init"};
  std::string body_frame{"base_link"};
  bool enable_motion_output{false};
  bool use_native_dds{true};
  int dds_domain_id{0};
  bool dds_use_shm{false};
  std::string dds_topic_prefix{"rt"};
  std::string dds_network_name;
};

}  // namespace m20
