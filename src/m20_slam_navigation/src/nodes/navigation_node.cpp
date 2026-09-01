/**
 * @file navigation_node.cpp
 * @brief M20Pro native navigation contract adapter.
 *
 * The node keeps the native data path visible and deterministic:
 *
 *   /GRID_MAP + /goal_pose (or /GOAL_GLOBAL)
 *       -> Hybrid A* + cubic smoothing -> /path_Astar, /global_path
 *   /path_Astar + /NAV_POINTS + /ODOM (and /MOTION_INFO)
 *       -> DWA / LinePlanner -> /NAV_CMD
 *
 * The native DrDDS message/service types are provided by the optional bridge.
 * A workstation build uses standard ROS message fallbacks for offline tests;
 * it never advertises a standard type on a native topic when the DrDDS SDK is
 * available.
 */

#include "m20_slam_navigation/common/math_utils.hpp"
#include "m20_slam_navigation/common/native_navigation.hpp"
#include "m20_slam_navigation/common/params.hpp"
#include "m20_slam_navigation/common/types.hpp"
#include "m20_slam_navigation/control/local_controller_node.hpp"
#include "m20_slam_navigation/ros/native_navigation_bridge.hpp"
#include "m20_slam_navigation/terrain/traversability_map.hpp"
#include "m20_slam_navigation/planning/global_planner_node.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace m20::nodes {

namespace {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

double numericParameter(const rclcpp::Parameter& parameter) {
  if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
    return parameter.as_double();
  }
  if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
    return static_cast<double>(parameter.as_int());
  }
  throw std::runtime_error("expected an integer or floating-point ROS parameter");
}

Eigen::Matrix<Scalar, 3, 1> poseVector(const SE3Pose& pose) {
  return Eigen::Matrix<Scalar, 3, 1>(
      pose.t.x(), pose.t.y(), math::quaternion_to_yaw(pose.q));
}

SE3Pose poseFromTransform(const geometry_msgs::msg::TransformStamped& transform) {
  SE3Pose pose;
  pose.t = Eigen::Matrix<Scalar, 3, 1>(
      transform.transform.translation.x,
      transform.transform.translation.y,
      transform.transform.translation.z);
  pose.q = Eigen::Quaternion<Scalar>(
      transform.transform.rotation.w,
      transform.transform.rotation.x,
      transform.transform.rotation.y,
      transform.transform.rotation.z);
  pose.q.normalize();
  return pose;
}

Eigen::Quaternion<Scalar> normalizedQuaternion(
    double w, double x, double y, double z) {
  Eigen::Quaternion<Scalar> quaternion(w, x, y, z);
  if (quaternion.squaredNorm() < 1e-12) {
    return Eigen::Quaternion<Scalar>::Identity();
  }
  quaternion.normalize();
  return quaternion;
}

std::vector<uint8_t> occupancyToCostmap(const nav_msgs::msg::OccupancyGrid& grid) {
  std::vector<uint8_t> costmap;
  costmap.reserve(grid.data.size());
  for (const auto value : grid.data) {
    if (value < 0) {
      costmap.push_back(255);  // unknown
    } else if (value >= 100) {
      costmap.push_back(254);  // native planner lethal threshold
    } else {
      costmap.push_back(static_cast<uint8_t>(
          std::clamp(static_cast<int>(std::lround(value * 253.0 / 100.0)), 0, 253)));
    }
  }
  return costmap;
}

}  // namespace

class NavigationNode : public rclcpp_lifecycle::LifecycleNode {
public:
  explicit NavigationNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : rclcpp_lifecycle::LifecycleNode("navigation_node", options) {
    autostart_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
      if (get_current_state().id() ==
          lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
        trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
      }
      if (get_current_state().id() ==
          lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
        trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
      }
      if (get_current_state().id() ==
          lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        autostart_timer_.reset();
      }
    });
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State&) override {
    RCLCPP_INFO(get_logger(), "NavigationNode: configuring native M20Pro contract");
    loadParameters();

    traversability_map_ = std::make_shared<terrain::TraversabilityMap>(terrain_params_);
    global_planner_ = std::make_shared<planning::GlobalPlannerNode>(
        planner_params_, terrain_params_);
    local_controller_ = std::make_shared<control::LocalControllerNode>(controller_params_);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    createPublishers();
    createSubscriptions();

    global_planner_->setPlanCallback(
        [this](const std::vector<Eigen::Matrix<Scalar, 3, 1>>& path) {
          onPlan(path);
        });
    local_controller_->setVelocityCallback(
        [this](const control::VelocityCommand& command) {
          publishVelocity(command);
        });

    native::DDSOptions dds_options;
    dds_options.domain_id = runtime_params_.dds_domain_id;
    dds_options.use_shm = runtime_params_.dds_use_shm;
    dds_options.topic_prefix = runtime_params_.dds_topic_prefix;
    dds_options.network_name = runtime_params_.dds_network_name;

    ros::NativeNavigationCallbacks callbacks;
    callbacks.goal = [this](double x, double y, double yaw, const std::string& frame_id) {
      return acceptGoal(x, y, yaw, frame_id);
    };
    callbacks.cancel = [this](std::int32_t command) {
      cancelNavigation(command);
    };
    callbacks.planner_mode = [this](std::int32_t mode) {
      if (local_controller_) local_controller_->setPlannerMode(mode);
    };
    callbacks.motion_info = [this](double vx, double vy, double omega) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      measured_velocity_ = control::VelocityCommand{vx, vy, omega};
      have_motion_info_ = true;
    };
    callbacks.set_param = [this](const native::LocalParamUpdate& update) {
      applyRuntimeLocalParams(update);
    };

    std::string dds_error;
    if (runtime_params_.use_native_dds &&
        native_bridge_.start(dds_options, runtime_params_.topics,
                             std::move(callbacks), dds_error)) {
      native_bridge_active_ = true;
      RCLCPP_INFO(get_logger(), "Native DrDDS navigation contract enabled");
    } else {
      native_bridge_active_ = false;
      if (runtime_params_.use_native_dds && !dds_error.empty()) {
        RCLCPP_WARN(get_logger(), "Native DrDDS unavailable: %s", dds_error.c_str());
      }
#ifdef M20_HAS_DRDDS
      RCLCPP_ERROR(
          get_logger(),
          "This build has the vendor SDK but native navigation bridge failed; "
          "standard fallback topics are intentionally disabled");
#else
      createFallbackInterfaces();
#endif
    }

    control_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / std::max(
                controller_params_.monitor_loop_frequency, Scalar(1.0)))),
        [this]() { controlLoop(); });

    RCLCPP_INFO(
        get_logger(),
        "Native topics: map=%s odom=%s points=%s path=%s local_goal=%s cmd=%s",
        runtime_params_.topics.grid_map.c_str(), runtime_params_.topics.odom.c_str(),
        runtime_params_.topics.nav_points.c_str(), runtime_params_.topics.astar_path.c_str(),
        runtime_params_.topics.local_goal.c_str(), runtime_params_.topics.nav_cmd.c_str());
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State&) override {
    RCLCPP_INFO(get_logger(), "NavigationNode: activating (motion output=%s)",
                runtime_params_.enable_motion_output ? "enabled" : "disabled");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override {
    publishVelocity(control::VelocityCommand{});
    RCLCPP_INFO(get_logger(), "NavigationNode: deactivated");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override {
    control_timer_.reset();
    native_bridge_.stop();
    native_bridge_active_ = false;
    lidar_sub_.reset();
    accumulated_cloud_sub_.reset();
    grid_map_sub_.reset();
    odom_sub_.reset();
    goal_sub_.reset();
    initial_pose_sub_.reset();
#ifndef M20_HAS_DRDDS
    planner_mode_sub_.reset();
    cancel_nav_sub_.reset();
    cancel_global_service_.reset();
    cancel_planner_service_.reset();
#endif
    costmap_pub_.reset();
    plan_pub_.reset();
    global_path_pub_.reset();
    local_goal_pub_.reset();
    target_goal_pub_.reset();
    goal_baselink_pub_.reset();
    local_goal_baselink_pub_.reset();
    local_path_pub_.reset();
    track_path_pub_.reset();
    visible_points_pub_.reset();
    pruned_visible_points_pub_.reset();
    free_paths_pub_.reset();
    local_scans_pub_.reset();
    grid_map_3d_pub_.reset();
    markers_pub_.reset();
#ifndef M20_HAS_DRDDS
    cmd_vel_pub_.reset();
    global_status_pub_.reset();
    local_status_pub_.reset();
#endif
    traversability_map_.reset();
    global_planner_.reset();
    local_controller_.reset();
    tf_listener_.reset();
    tf_buffer_.reset();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State&) override {
    native_bridge_.stop();
    return CallbackReturn::SUCCESS;
  }

private:
  // --------------------------------------------------------------------------
  // Parameter loading
  // --------------------------------------------------------------------------
  void loadParameters() {
#define LOAD_SCALAR(name, field) \
    do { \
      if (!has_parameter(name)) declare_parameter<double>(name, static_cast<double>(field)); \
      field = numericParameter(get_parameter(name)); \
    } while (false)
#define LOAD_INT(name, field) \
    do { \
      if (!has_parameter(name)) declare_parameter<int>(name, static_cast<int>(field)); \
      field = static_cast<int>(numericParameter(get_parameter(name))); \
    } while (false)
#define LOAD_BOOL(name, field) \
    do { \
      if (!has_parameter(name)) declare_parameter<bool>(name, static_cast<bool>(field)); \
      field = get_parameter(name).as_bool(); \
    } while (false)
#define LOAD_STRING(name, field) \
    do { \
      if (!has_parameter(name)) declare_parameter<std::string>(name, field); \
      field = get_parameter(name).as_string(); \
    } while (false)

    LOAD_STRING("navigation.world_frame", runtime_params_.world_frame);
    LOAD_STRING("navigation.body_frame", runtime_params_.body_frame);
    LOAD_BOOL("navigation.enable_motion_output", runtime_params_.enable_motion_output);
    LOAD_BOOL("navigation.use_native_dds", runtime_params_.use_native_dds);
    LOAD_INT("drdds.domain_id", runtime_params_.dds_domain_id);
    LOAD_BOOL("drdds.use_shm", runtime_params_.dds_use_shm);
    LOAD_STRING("drdds.topic_prefix", runtime_params_.dds_topic_prefix);
    LOAD_STRING("drdds.network_name", runtime_params_.dds_network_name);

    auto& t = runtime_params_.topics;
    LOAD_STRING("native.topics.grid_map", t.grid_map);
    LOAD_STRING("native.topics.initial_pose", t.initial_pose);
    LOAD_STRING("native.topics.goal_pose", t.goal_pose);
    LOAD_STRING("native.topics.goal_global_service", t.goal_global_service);
    LOAD_STRING("native.topics.odom", t.odom);
    LOAD_STRING("native.topics.motion_info", t.motion_info);
    LOAD_STRING("native.topics.nav_points", t.nav_points);
    LOAD_STRING("native.topics.cancel_nav", t.cancel_nav);
    LOAD_STRING("native.topics.cancel_global_service", t.cancel_global_service);
    LOAD_STRING("native.topics.planner_mode", t.planner_mode);
    LOAD_STRING("native.topics.astar_path", t.astar_path);
    LOAD_STRING("native.topics.visible_points", t.visible_points);
    LOAD_STRING("native.topics.pruned_visible_points", t.pruned_visible_points);
    LOAD_STRING("native.topics.local_goal", t.local_goal);
    LOAD_STRING("native.topics.local_map", t.local_map);
    LOAD_STRING("native.topics.global_planner_status", t.global_planner_status);
    LOAD_STRING("native.topics.nav_cmd", t.nav_cmd);
    LOAD_STRING("native.topics.planner_status", t.planner_status);
    LOAD_STRING("native.topics.target_goal", t.target_goal);
    LOAD_STRING("native.topics.goal_baselink", t.goal_baselink);
    LOAD_STRING("native.topics.local_goal_baselink", t.local_goal_baselink);
    LOAD_STRING("native.topics.free_paths", t.free_paths);
    LOAD_STRING("native.topics.local_path", t.local_path);
    LOAD_STRING("native.topics.local_scans", t.local_scans);
    LOAD_STRING("native.topics.track_path_baselink", t.track_path_baselink);
    LOAD_STRING("native.topics.global_path", t.global_path);
    LOAD_STRING("native.topics.grid_map_3d", t.grid_map_3d);
    LOAD_STRING("native.topics.global_path_markers", t.global_path_markers);
    LOAD_STRING("native.topics.set_param_service", t.set_param_service);
    LOAD_STRING("native.topics.cancel_planner_service", t.cancel_planner_service);
    LOAD_STRING("native.topics.goal_planner_service", t.goal_planner_service);

    LOAD_SCALAR("terrain.grid_resolution", terrain_params_.grid_resolution);
    LOAD_SCALAR("terrain.map_length", terrain_params_.map_length);
    LOAD_SCALAR("terrain.map_width", terrain_params_.map_width);
    LOAD_SCALAR("terrain.map_height_min", terrain_params_.map_height_min);
    LOAD_SCALAR("terrain.map_height_max", terrain_params_.map_height_max);
    LOAD_SCALAR("terrain.voxel_size", terrain_params_.voxel_size);
    LOAD_SCALAR("terrain.max_drop", terrain_params_.max_drop);
    LOAD_SCALAR("terrain.max_roughness", terrain_params_.max_roughness);
    LOAD_SCALAR("terrain.max_slope_deg", terrain_params_.max_slope_deg);
    LOAD_INT("terrain.max_inpaint_pixels", terrain_params_.max_inpaint_pixels);
    LOAD_BOOL("terrain.enable_center_padding", terrain_params_.enable_center_padding);
    LOAD_SCALAR("terrain.center_dist_thresh", terrain_params_.center_dist_thresh);
    LOAD_BOOL("terrain.enable_blind_check", terrain_params_.enable_blind_check);
    LOAD_BOOL("terrain.treat_nan_as_stiff", terrain_params_.treat_nan_as_stiff);
    LOAD_STRING("terrain.accumulate_cloud_topic", terrain_params_.accumulate_cloud_topic);
    LOAD_STRING("terrain.imu_topic", terrain_params_.imu_topic);
    LOAD_STRING("terrain.passable_cloud_topic", terrain_params_.passable_cloud_topic);
    LOAD_STRING("terrain.impassable_cloud_topic", terrain_params_.impassable_cloud_topic);
    LOAD_STRING("terrain.grid_map_topic", terrain_params_.grid_map_topic);
    LOAD_STRING("terrain.traversal_cost_topic", terrain_params_.traversal_cost_topic);
    LOAD_STRING("terrain.world_frame", terrain_params_.world_frame);
    LOAD_STRING("terrain.gravity_frame", terrain_params_.gravity_frame);
    LOAD_STRING("terrain.body_frame", terrain_params_.body_frame);
    LOAD_STRING("terrain.used_frame", terrain_params_.used_frame);
    LOAD_STRING("terrain.dog_model", terrain_params_.dog_model);
    LOAD_BOOL("terrain.raycast.enable", terrain_params_.raycast_enable);
    LOAD_SCALAR("terrain.raycast.max_ray_distance", terrain_params_.raycast_max_ray_distance);
    LOAD_SCALAR("terrain.raycast.max_nan_gap", terrain_params_.raycast_max_nan_gap);
    LOAD_BOOL("terrain.elevation_solver.use_histogram_solver",
              terrain_params_.elevation_use_histogram_solver);
    LOAD_INT("terrain.elevation_solver.histogram_bins", terrain_params_.elevation_histogram_bins);
    LOAD_BOOL("terrain.elevation_solver.region_enabled", terrain_params_.elevation_region_enabled);
    LOAD_SCALAR("terrain.elevation_solver.region_min_x", terrain_params_.elevation_region_min_x);
    LOAD_SCALAR("terrain.elevation_solver.region_max_x", terrain_params_.elevation_region_max_x);
    LOAD_SCALAR("terrain.elevation_solver.region_min_y", terrain_params_.elevation_region_min_y);
    LOAD_SCALAR("terrain.elevation_solver.region_max_y", terrain_params_.elevation_region_max_y);
    LOAD_SCALAR("pcl_pass_grid.out_minx", terrain_params_.pass_grid_out_min_x);
    LOAD_SCALAR("pcl_pass_grid.out_maxx", terrain_params_.pass_grid_out_max_x);
    LOAD_SCALAR("pcl_pass_grid.out_miny", terrain_params_.pass_grid_out_min_y);
    LOAD_SCALAR("pcl_pass_grid.out_maxy", terrain_params_.pass_grid_out_max_y);
    LOAD_SCALAR("pcl_pass_grid.out_minz", terrain_params_.pass_grid_out_min_z);
    LOAD_SCALAR("pcl_pass_grid.out_maxz", terrain_params_.pass_grid_out_max_z);
    LOAD_SCALAR("pcl_pass_grid.in_minx", terrain_params_.pass_grid_in_min_x);
    LOAD_SCALAR("pcl_pass_grid.in_maxx", terrain_params_.pass_grid_in_max_x);
    LOAD_SCALAR("pcl_pass_grid.in_miny", terrain_params_.pass_grid_in_min_y);
    LOAD_SCALAR("pcl_pass_grid.in_maxy", terrain_params_.pass_grid_in_max_y);
    LOAD_SCALAR("pcl_pass_grid.in_minz", terrain_params_.pass_grid_in_min_z);
    LOAD_SCALAR("pcl_pass_grid.in_maxz", terrain_params_.pass_grid_in_max_z);
    LOAD_SCALAR("pcl_pass_grid.serach_radius", terrain_params_.pass_grid_search_radius);
    LOAD_INT("pcl_pass_grid.minNeighbors", terrain_params_.pass_grid_min_neighbors);
    LOAD_SCALAR("pcl_pass_grid.leaf_size", terrain_params_.pass_grid_leaf_size);
    LOAD_BOOL("terrain.traversal_cost.enable", terrain_params_.traversal_cost_enable);
    LOAD_SCALAR("terrain.traversal_cost.slope_free_deg", terrain_params_.traversal_slope_free_deg);
    LOAD_SCALAR("terrain.traversal_cost.slope_block_deg", terrain_params_.traversal_slope_block_deg);
    LOAD_SCALAR("terrain.traversal_cost.rough_free", terrain_params_.traversal_rough_free);
    LOAD_SCALAR("terrain.traversal_cost.rough_block", terrain_params_.traversal_rough_block);
    LOAD_SCALAR("terrain.traversal_cost.step_free", terrain_params_.traversal_step_free);
    LOAD_SCALAR("terrain.traversal_cost.step_block", terrain_params_.traversal_step_block);
    LOAD_SCALAR("terrain.traversal_cost.slope_weight", terrain_params_.traversal_slope_weight);
    LOAD_SCALAR("terrain.traversal_cost.roughness_weight", terrain_params_.traversal_roughness_weight);
    LOAD_SCALAR("terrain.traversal_cost.step_weight", terrain_params_.traversal_step_weight);
    LOAD_SCALAR("terrain.traversal_cost.easy_cost", terrain_params_.traversal_easy_cost);
    LOAD_SCALAR("terrain.traversal_cost.hard_cost", terrain_params_.traversal_hard_cost);
    LOAD_SCALAR("terrain.traversal_cost.max_cost", terrain_params_.traversal_max_cost);
    LOAD_SCALAR("terrain.traversal_cost.missing_cost", terrain_params_.traversal_missing_cost);
    LOAD_SCALAR("terrain.traversal_cost.curve_power", terrain_params_.traversal_curve_power);
    LOAD_SCALAR("terrain.traversal_cost.low_cost_filter_ratio",
                terrain_params_.traversal_low_cost_filter_ratio);
    LOAD_INT("terrain.traversal_cost.terrain_sample_window",
             terrain_params_.traversal_terrain_sample_window);
    LOAD_SCALAR("terrain.traversal_cost.safe_zone_side_length",
                terrain_params_.traversal_safe_zone_side_length);

    LOAD_SCALAR("terrain.max_range", terrain_params_.max_range);
    LOAD_SCALAR("terrain.min_range", terrain_params_.min_range);
    LOAD_SCALAR("terrain.max_climb_angle", terrain_params_.max_climb_angle);
    LOAD_SCALAR("terrain.slope_weight", terrain_params_.slope_weight);
    LOAD_SCALAR("terrain.roughness_threshold", terrain_params_.roughness_threshold);
    LOAD_SCALAR("terrain.roughness_weight", terrain_params_.roughness_weight);
    LOAD_SCALAR("terrain.max_step_height", terrain_params_.max_step_height);
    LOAD_SCALAR("terrain.max_step_depth", terrain_params_.max_step_depth);
    LOAD_SCALAR("terrain.step_weight", terrain_params_.step_weight);
    LOAD_SCALAR("terrain.normal_estimation_radius", terrain_params_.normal_estimation_radius);

    LOAD_SCALAR("global.weight_a", planner_params_.weight_a);
    LOAD_SCALAR("global.weight_b", planner_params_.weight_b);
    LOAD_SCALAR("global.weight_heading", planner_params_.weight_heading);
    LOAD_SCALAR("global.cost_steer", planner_params_.cost_steer);
    LOAD_SCALAR("global.cost_steerchange", planner_params_.cost_steerchange);
    LOAD_SCALAR("global.cost_gear", planner_params_.cost_gear);
    LOAD_SCALAR("global.cost_backward", planner_params_.cost_backward);
    LOAD_SCALAR("global.cost_reduce", planner_params_.cost_reduce);
    LOAD_SCALAR("global.step_size", planner_params_.step_size);
    LOAD_SCALAR("global.sample_interval", planner_params_.sample_interval);
    LOAD_SCALAR("global.bodyLength", planner_params_.body_length);
    LOAD_SCALAR("global.bodyWidth", planner_params_.body_width);
    LOAD_SCALAR("global.max_steer", planner_params_.max_steer);
    LOAD_INT("global.num_steerind", planner_params_.num_steerind);
    LOAD_SCALAR("global.goalDis", planner_params_.goal_dis);
    LOAD_BOOL("global.dynamic_update", planner_params_.dynamic_update);
    LOAD_BOOL("global.test_mode", planner_params_.test_mode);
    LOAD_SCALAR("global.xy_tolerance", planner_params_.xy_tolerance);
    LOAD_SCALAR("global.dynamic_map_size", planner_params_.dynamic_map_size);
    LOAD_INT("global.dynamic_map_grid", planner_params_.dynamic_map_grid);
    LOAD_SCALAR("global.astar_time", planner_params_.astar_time);
    LOAD_INT("global.debug_point_num", planner_params_.debug_point_num);
    LOAD_BOOL("global.enable_smoothing", planner_params_.enable_smoothing);
    LOAD_INT("global.smooth_count", planner_params_.smooth_count);
    LOAD_INT("global.smooth_degree", planner_params_.smooth_degree);
    LOAD_INT("global.num_samples", planner_params_.num_samples);
    LOAD_SCALAR("global.angle_threshold", planner_params_.angle_threshold_deg);
    LOAD_SCALAR("global.local_goal_freq_hz", planner_params_.local_goal_freq_hz);
    LOAD_SCALAR("global.max_trajectory_range_", planner_params_.max_trajectory_range);
    LOAD_SCALAR("global.local_point_dis_", planner_params_.local_point_dis);
    LOAD_SCALAR("global.local_point_dis_v_max_", planner_params_.local_point_dis_v_max);
    LOAD_SCALAR("global.max_speed_x_", planner_params_.native_max_speed_x);
    LOAD_SCALAR("global.sharp_turn_angle_", planner_params_.sharp_turn_angle_deg);
    LOAD_INT("global.local_goal_search_window_", planner_params_.local_goal_search_window);

    LOAD_SCALAR("planner.grid_resolution", planner_params_.grid_resolution);
    LOAD_INT("planner.num_heading_bins", planner_params_.num_heading_bins);
    LOAD_SCALAR("planner.max_curvature", planner_params_.max_curvature);
    LOAD_SCALAR("planner.primitive_length", planner_params_.primitive_length);
    LOAD_INT("planner.num_directions", planner_params_.num_directions);
    LOAD_INT("planner.num_rotations", planner_params_.num_rotations);
    LOAD_SCALAR("planner.heuristic_weight", planner_params_.heuristic_weight);
    LOAD_SCALAR("planner.time_limit_sec", planner_params_.time_limit_sec);
    LOAD_INT("planner.max_expansions", planner_params_.max_expansions);
    LOAD_SCALAR("planner.spline_smoothness_weight", planner_params_.spline_smoothness_weight);
    if (planner_params_.astar_time > 0.0) planner_params_.time_limit_sec = planner_params_.astar_time;
    if (planner_params_.step_size > 0.0) planner_params_.primitive_length = planner_params_.step_size;

    LOAD_SCALAR("local.lookAheadDis", controller_params_.look_ahead_dis);
    LOAD_SCALAR("local.maxSpeedX", controller_params_.native_max_speed_x);
    LOAD_SCALAR("local.maxSpeedY", controller_params_.native_max_speed_y);
    LOAD_SCALAR("local.maxTheta", controller_params_.native_max_theta);
    LOAD_SCALAR("local.barking_deceleration", controller_params_.barking_deceleration);
    LOAD_SCALAR("local.stop_distance", controller_params_.stop_distance);
    LOAD_SCALAR("local.max_v_threshold", controller_params_.max_v_threshold);
    LOAD_SCALAR("local.robotLength", controller_params_.robot_length);
    LOAD_SCALAR("local.robotWidth", controller_params_.robot_width);
    LOAD_SCALAR("local.sensorOffsetX", controller_params_.sensor_offset_x);
    LOAD_SCALAR("local.sensorOffsetY", controller_params_.sensor_offset_y);
    LOAD_SCALAR("local.obstacleHeightThre", controller_params_.obstacle_height_threshold);
    LOAD_SCALAR("local.goal_in_ob_dis", controller_params_.goal_in_obstacle_distance);
    LOAD_SCALAR("local.goal_in_ob_z", controller_params_.goal_in_obstacle_z);
    LOAD_INT("local.pointPerPathThre", controller_params_.point_per_path_threshold);
    LOAD_STRING("local.input_source", controller_params_.input_source);
    LOAD_SCALAR("local.grid_size_", controller_params_.grid_size);
    LOAD_INT("local.grid_occu_num_", controller_params_.grid_occupied_number);
    LOAD_BOOL("local.enable_cloud_stacking", controller_params_.enable_cloud_stacking);
    LOAD_INT("local.cloud_stack_size", controller_params_.cloud_stack_size);
    LOAD_BOOL("local.directLine_mode", controller_params_.direct_line_mode);
    LOAD_BOOL("local.judge_close", controller_params_.judge_close);
    LOAD_BOOL("local.change_close", controller_params_.change_close);
    LOAD_BOOL("local.backward_mode", controller_params_.backward_mode);
    LOAD_BOOL("local.judge_turn", controller_params_.judge_turn);
    LOAD_SCALAR("local.xy_tolerance", controller_params_.xy_tolerance);
    LOAD_SCALAR("local.yaw_tolerance", controller_params_.yaw_tolerance);
    LOAD_INT("local.block_number", controller_params_.block_number);
    LOAD_SCALAR("local.dwa_fine_tune_distance", controller_params_.dwa_fine_tune_distance);
    LOAD_SCALAR("local.weight_goal", controller_params_.weight_goal);
    LOAD_SCALAR("local.weight_yaw", controller_params_.weight_yaw);
    LOAD_SCALAR("local.weight_spdy", controller_params_.weight_spdy);
    LOAD_SCALAR("local.weight_ob1", controller_params_.weight_ob1);
    LOAD_SCALAR("local.weight_ob2", controller_params_.weight_ob2);
    LOAD_SCALAR("local.weight_ob3", controller_params_.weight_ob3);
    LOAD_SCALAR("local.local_point_dis", controller_params_.local_point_dis);
    LOAD_SCALAR("local.local_point_dis_v_max", controller_params_.local_point_dis_v_max);
    LOAD_BOOL("local.local_try", controller_params_.local_try);
    LOAD_INT("local.path_num", controller_params_.path_num);
    LOAD_INT("local.path_sample_num", controller_params_.path_sample_num);
    LOAD_SCALAR("local.grid_voxel_size", controller_params_.grid_voxel_size);
    LOAD_SCALAR("local.grid_voxel_offset_x", controller_params_.grid_voxel_offset_x);
    LOAD_SCALAR("local.grid_voxel_offset_y", controller_params_.grid_voxel_offset_y);
    LOAD_INT("local.grid_voxel_num_x", controller_params_.grid_voxel_num_x);
    LOAD_INT("local.grid_voxel_num_y", controller_params_.grid_voxel_num_y);
    LOAD_SCALAR("local.turn_yaw_kp", controller_params_.turn_yaw_kp);
    LOAD_SCALAR("local.angleThreshold", controller_params_.angle_threshold_deg);
    LOAD_SCALAR("local.sumAngleThreshold", controller_params_.sum_angle_threshold_deg);
    LOAD_SCALAR("local.minSegmentLength", controller_params_.min_segment_length);
    LOAD_SCALAR("local.trackDis", controller_params_.track_distance);
    LOAD_SCALAR("local.speed_ratio", controller_params_.speed_ratio);
    LOAD_SCALAR("local.speed_ratio_yaw", controller_params_.speed_ratio_yaw);
    LOAD_SCALAR("local.dl_min_yaw", controller_params_.dl_min_yaw);
    LOAD_SCALAR("local.dl_min_x", controller_params_.dl_min_x);
    LOAD_SCALAR("local.dl_min_y", controller_params_.dl_min_y);
    LOAD_SCALAR("local.pl_min_yaw", controller_params_.pl_min_yaw);
    LOAD_SCALAR("local.pl_min_x", controller_params_.pl_min_x);
    LOAD_SCALAR("local.pl_min_y", controller_params_.pl_min_y);
    LOAD_SCALAR("local.dl_line_yaw_min", controller_params_.dl_line_yaw_min);
    LOAD_SCALAR("local.close_deceleration", controller_params_.close_deceleration);
    LOAD_SCALAR("local.proximity_distance", controller_params_.proximity_distance);
    LOAD_SCALAR("local.warning_threshold", controller_params_.warning_threshold);
    LOAD_SCALAR("local.vel_acc_threshold", controller_params_.velocity_accel_threshold);
    LOAD_SCALAR("local.x_acc_increment", controller_params_.x_acc_increment);
    LOAD_INT("local.count_num", controller_params_.count_num);
    LOAD_SCALAR("local.only_rotate_yaw", controller_params_.only_rotate_yaw);
    LOAD_SCALAR("local.adjust_yaw_min_", controller_params_.adjust_yaw_min);
    LOAD_SCALAR("local.rotate_detect_dis", controller_params_.rotate_detect_dis);
    LOAD_SCALAR("local.turn_approach_threshold", controller_params_.turn_approach_threshold);
    LOAD_SCALAR("local.monitor_loop_frequency", controller_params_.monitor_loop_frequency);
    LOAD_SCALAR("local.sharp_turn_angle", controller_params_.sharp_turn_angle_deg);

    LOAD_SCALAR("controller.max_linear_vel_x", controller_params_.max_linear_vel_x);
    LOAD_SCALAR("controller.max_linear_vel_y", controller_params_.max_linear_vel_y);
    LOAD_SCALAR("controller.max_angular_vel", controller_params_.max_angular_vel);
    LOAD_SCALAR("controller.max_linear_accel_x", controller_params_.max_linear_accel_x);
    LOAD_SCALAR("controller.max_linear_accel_y", controller_params_.max_linear_accel_y);
    LOAD_SCALAR("controller.max_angular_accel", controller_params_.max_angular_accel);
    LOAD_INT("controller.num_vx_samples", controller_params_.num_vx_samples);
    LOAD_INT("controller.num_vy_samples", controller_params_.num_vy_samples);
    LOAD_INT("controller.num_omega_samples", controller_params_.num_omega_samples);
    LOAD_SCALAR("controller.sim_time", controller_params_.sim_time);
    LOAD_SCALAR("controller.sim_dt", controller_params_.sim_dt);
    LOAD_SCALAR("controller.weight_path_clearance", controller_params_.weight_path_clearance);
    LOAD_SCALAR("controller.weight_goal_align", controller_params_.weight_goal_align);
    LOAD_SCALAR("controller.weight_velocity_progress", controller_params_.weight_velocity_progress);
    LOAD_SCALAR("controller.weight_terrain_cost", controller_params_.weight_terrain_cost);
    LOAD_SCALAR("controller.weight_smoothness", controller_params_.weight_smoothness);
    LOAD_SCALAR("controller.lineplanner_lateral_gain", controller_params_.lineplanner_lateral_gain);
    LOAD_SCALAR("controller.lineplanner_yaw_threshold", controller_params_.lineplanner_yaw_threshold);

    controller_params_.max_linear_vel_x = controller_params_.native_max_speed_x;
    controller_params_.max_linear_vel_y = controller_params_.native_max_speed_y;
    controller_params_.max_angular_vel = controller_params_.native_max_theta;
    controller_params_.lineplanner_lateral_gain = controller_params_.turn_yaw_kp;
    controller_params_.lineplanner_yaw_threshold = controller_params_.dl_line_yaw_min;
    terrain_params_.world_frame = runtime_params_.world_frame;
    terrain_params_.body_frame = runtime_params_.body_frame;

    if (runtime_params_.world_frame.empty() || runtime_params_.body_frame.empty() ||
        runtime_params_.topics.grid_map.empty() || runtime_params_.topics.odom.empty() ||
        runtime_params_.topics.nav_points.empty() || runtime_params_.topics.nav_cmd.empty()) {
      throw std::runtime_error("native navigation frames and required topics must not be empty");
    }
    if (planner_params_.grid_resolution <= 0.0 || planner_params_.num_heading_bins <= 0 ||
        planner_params_.num_directions <= 0 || planner_params_.primitive_length <= 0.0 ||
        planner_params_.time_limit_sec <= 0.0 || planner_params_.max_expansions <= 0 ||
        planner_params_.local_goal_search_window <= 0) {
      throw std::runtime_error("invalid global planner discretization or search limits");
    }
    if (controller_params_.native_max_speed_x <= 0.0 ||
        controller_params_.native_max_speed_y < 0.0 ||
        controller_params_.native_max_theta <= 0.0 ||
        controller_params_.robot_length <= 0.0 || controller_params_.robot_width <= 0.0 ||
        controller_params_.num_vx_samples <= 0 || controller_params_.num_vy_samples <= 0 ||
        controller_params_.num_omega_samples <= 0 || controller_params_.sim_time <= 0.0 ||
        controller_params_.sim_dt <= 0.0 || controller_params_.monitor_loop_frequency <= 0.0) {
      throw std::runtime_error("invalid local planner geometry, velocity, or sampling parameter");
    }
    if (terrain_params_.grid_resolution <= 0.0 || terrain_params_.map_length <= 0.0 ||
        terrain_params_.map_width <= 0.0 || terrain_params_.voxel_size <= 0.0) {
      throw std::runtime_error("invalid traversability-map geometry");
    }
  }

  // --------------------------------------------------------------------------
  // ROS interfaces
  // --------------------------------------------------------------------------
  void createPublishers() {
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        runtime_params_.topics.local_map, qos);
    plan_pub_ = create_publisher<nav_msgs::msg::Path>(
        runtime_params_.topics.astar_path, qos);
    global_path_pub_ = create_publisher<nav_msgs::msg::Path>(
        runtime_params_.topics.global_path, qos);
    local_goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        runtime_params_.topics.local_goal, qos);
    target_goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        runtime_params_.topics.target_goal, qos);
    goal_baselink_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        runtime_params_.topics.goal_baselink, qos);
    local_goal_baselink_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        runtime_params_.topics.local_goal_baselink, qos);
    local_path_pub_ = create_publisher<nav_msgs::msg::Path>(
        runtime_params_.topics.local_path, qos);
    track_path_pub_ = create_publisher<nav_msgs::msg::Path>(
        runtime_params_.topics.track_path_baselink, qos);
    visible_points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        runtime_params_.topics.visible_points, qos);
    pruned_visible_points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        runtime_params_.topics.pruned_visible_points, qos);
    free_paths_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        runtime_params_.topics.free_paths, qos);
    local_scans_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        runtime_params_.topics.local_scans, qos);
    grid_map_3d_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        runtime_params_.topics.grid_map_3d, qos);
    markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        runtime_params_.topics.global_path_markers, qos);
  }

  void createSubscriptions() {
    grid_map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        runtime_params_.topics.grid_map, rclcpp::QoS(rclcpp::KeepLast(10)),
        [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
          onGridMap(*msg);
        });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        runtime_params_.topics.odom, rclcpp::QoS(rclcpp::KeepLast(20)),
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          onOdometry(*msg);
        });
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        runtime_params_.topics.goal_pose, rclcpp::QoS(rclcpp::KeepLast(10)),
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          const double yaw = std::atan2(
              2.0 * (msg->pose.orientation.w * msg->pose.orientation.z +
                     msg->pose.orientation.x * msg->pose.orientation.y),
              1.0 - 2.0 * (msg->pose.orientation.y * msg->pose.orientation.y +
                           msg->pose.orientation.z * msg->pose.orientation.z));
          acceptGoal(msg->pose.position.x, msg->pose.position.y, yaw,
                     msg->header.frame_id);
        });
    initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        runtime_params_.topics.initial_pose, rclcpp::QoS(rclcpp::KeepLast(10)),
        [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
          onInitialPose(*msg);
        });

    lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        runtime_params_.topics.nav_points, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          onPointCloud(*msg);
        });
    if (terrain_params_.accumulate_cloud_topic != runtime_params_.topics.nav_points) {
      accumulated_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
          terrain_params_.accumulate_cloud_topic, rclcpp::SensorDataQoS(),
          [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            onPointCloud(*msg);
          });
    }

#ifndef M20_HAS_DRDDS
    planner_mode_sub_ = create_subscription<std_msgs::msg::Int32>(
        runtime_params_.topics.planner_mode, rclcpp::QoS(rclcpp::KeepLast(10)),
        [this](const std_msgs::msg::Int32::SharedPtr msg) {
          if (local_controller_) local_controller_->setPlannerMode(msg->data);
        });
    cancel_nav_sub_ = create_subscription<std_msgs::msg::Int32>(
        runtime_params_.topics.cancel_nav, rclcpp::QoS(rclcpp::KeepLast(10)),
        [this](const std_msgs::msg::Int32::SharedPtr msg) {
          cancelNavigation(msg->data);
        });
#endif
  }

  void createFallbackInterfaces() {
#ifndef M20_HAS_DRDDS
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
        runtime_params_.topics.nav_cmd, rclcpp::QoS(rclcpp::KeepLast(10)));
    global_status_pub_ = create_publisher<std_msgs::msg::Int32>(
        runtime_params_.topics.global_planner_status, rclcpp::QoS(rclcpp::KeepLast(10)));
    local_status_pub_ = create_publisher<std_msgs::msg::Int32>(
        runtime_params_.topics.planner_status, rclcpp::QoS(rclcpp::KeepLast(10)));
    cancel_global_service_ = create_service<std_srvs::srv::SetBool>(
        runtime_params_.topics.cancel_global_service,
        [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
               std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
          cancelNavigation(request->data ? 1 : 0);
          response->success = true;
          response->message = "navigation canceled";
        });
    cancel_planner_service_ = create_service<std_srvs::srv::SetBool>(
        runtime_params_.topics.cancel_planner_service,
        [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
               std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
          cancelNavigation(request->data ? 1 : 0);
          response->success = true;
          response->message = "planner canceled";
        });
#endif
  }

  // --------------------------------------------------------------------------
  // Input callbacks and state
  // --------------------------------------------------------------------------
  void onInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped& msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    initial_pose_ = SE3Pose{};
    initial_pose_.t = Eigen::Matrix<Scalar, 3, 1>(
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);
    initial_pose_.q = normalizedQuaternion(
        msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);
    current_pose_ = initial_pose_;
    have_pose_ = true;
  }

  void onOdometry(const nav_msgs::msg::Odometry& msg) {
    SE3Pose odom_pose;
    odom_pose.t = Eigen::Matrix<Scalar, 3, 1>(
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);
    odom_pose.q = normalizedQuaternion(
        msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);

    SE3Pose world_odom = SE3Pose::Identity();
    if (!msg.header.frame_id.empty() && msg.header.frame_id != runtime_params_.world_frame &&
        tf_buffer_) {
      try {
        world_odom = poseFromTransform(tf_buffer_->lookupTransform(
            runtime_params_.world_frame, msg.header.frame_id, tf2::TimePointZero));
      } catch (...) {
        // Native M20 /ODOM is normally already in camera_init.  Keep the
        // message usable if a non-native frame has no TF yet.
      }
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    current_pose_ = world_odom * odom_pose;
    measured_velocity_ = control::VelocityCommand{
        msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z};
    have_motion_info_ = true;
    have_pose_ = true;
  }

  SE3Pose cloudTransform(const std::string& frame_id) const {
    if (frame_id.empty() || frame_id == runtime_params_.world_frame) {
      return SE3Pose::Identity();
    }
    if (tf_buffer_) {
      try {
        return poseFromTransform(tf_buffer_->lookupTransform(
            runtime_params_.world_frame, frame_id, tf2::TimePointZero));
      } catch (...) {
      }
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    return current_pose_;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr filterNativeTerrainCloud(
      const pcl::PointCloud<pcl::PointXYZI>::Ptr& input) const {
    pcl::PointCloud<pcl::PointXYZI>::Ptr bounded(
        new pcl::PointCloud<pcl::PointXYZI>());
    bounded->reserve(input ? input->size() : 0);
    if (!input) return bounded;

    // This is the observable pcl_pass_grid contract: crop the local planning
    // window and remove the M20 body box before passable-area analysis.
    for (const auto& point : input->points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
          !std::isfinite(point.z)) {
        continue;
      }
      const bool in_output_box =
          point.x >= terrain_params_.pass_grid_out_min_x &&
          point.x <= terrain_params_.pass_grid_out_max_x &&
          point.y >= terrain_params_.pass_grid_out_min_y &&
          point.y <= terrain_params_.pass_grid_out_max_y &&
          point.z >= terrain_params_.pass_grid_out_min_z &&
          point.z <= terrain_params_.pass_grid_out_max_z;
      const bool in_body_box =
          point.x >= terrain_params_.pass_grid_in_min_x &&
          point.x <= terrain_params_.pass_grid_in_max_x &&
          point.y >= terrain_params_.pass_grid_in_min_y &&
          point.y <= terrain_params_.pass_grid_in_max_y &&
          point.z >= terrain_params_.pass_grid_in_min_z &&
          point.z <= terrain_params_.pass_grid_in_max_z;
      if (in_output_box && !in_body_box) bounded->push_back(point);
    }

    if (bounded->empty()) return bounded;

    pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled = bounded;
    if (terrain_params_.pass_grid_leaf_size > 0.0) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr voxel_cloud(
          new pcl::PointCloud<pcl::PointXYZI>());
      pcl::VoxelGrid<pcl::PointXYZI> voxel;
      voxel.setInputCloud(bounded);
      const float leaf = static_cast<float>(terrain_params_.pass_grid_leaf_size);
      voxel.setLeafSize(leaf, leaf, leaf);
      voxel.filter(*voxel_cloud);
      downsampled = voxel_cloud;
    }

    if (downsampled->empty() || terrain_params_.pass_grid_search_radius <= 0.0 ||
        terrain_params_.pass_grid_min_neighbors <= 0) {
      return downsampled;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr denoised(
        new pcl::PointCloud<pcl::PointXYZI>());
    pcl::RadiusOutlierRemoval<pcl::PointXYZI> radius_filter;
    radius_filter.setInputCloud(downsampled);
    radius_filter.setRadiusSearch(
        static_cast<float>(terrain_params_.pass_grid_search_radius));
    radius_filter.setMinNeighborsInRadius(terrain_params_.pass_grid_min_neighbors);
    radius_filter.filter(*denoised);
    return denoised;
  }

  void onPointCloud(const sensor_msgs::msg::PointCloud2& msg) {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      latest_cloud_ = std::make_shared<sensor_msgs::msg::PointCloud2>(msg);
    }
    if (local_scans_pub_) local_scans_pub_->publish(msg);
    if (grid_map_3d_pub_) grid_map_3d_pub_->publish(msg);

    bool use_fallback_terrain = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      use_fallback_terrain = !have_native_grid_map_;
    }
    if (!use_fallback_terrain || !traversability_map_) return;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZI>());
    try {
      pcl::fromROSMsg(msg, *cloud);
    } catch (...) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Cannot decode native navigation cloud on %s", msg.header.frame_id.c_str());
      return;
    }
    cloud = filterNativeTerrainCloud(cloud);
    if (cloud->empty()) return;
    traversability_map_->update(cloud, cloudTransform(msg.header.frame_id));
    const auto costmap = traversability_map_->exportCostmap();
    global_planner_->setCostmap(
        costmap, traversability_map_->width(), traversability_map_->height(),
        traversability_map_->resolution(), traversability_map_->originX(),
        traversability_map_->originY());
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      active_costmap_ = costmap;
      active_width_ = traversability_map_->width();
      active_height_ = traversability_map_->height();
      active_resolution_ = traversability_map_->resolution();
      active_origin_x_ = traversability_map_->originX();
      active_origin_y_ = traversability_map_->originY();
    }
    publishCostmap(costmap, traversability_map_->width(), traversability_map_->height(),
                   traversability_map_->resolution(), traversability_map_->originX(),
                   traversability_map_->originY(), msg.header.stamp);
  }

  void onGridMap(const nav_msgs::msg::OccupancyGrid& msg) {
    if (msg.info.width == 0 || msg.info.height == 0 || msg.info.resolution <= 0.0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Ignoring invalid /GRID_MAP");
      return;
    }
    auto costmap = occupancyToCostmap(msg);
    if (costmap.size() != static_cast<std::size_t>(msg.info.width) * msg.info.height) return;

    global_planner_->setCostmap(
        costmap, static_cast<int>(msg.info.width), static_cast<int>(msg.info.height),
        msg.info.resolution, msg.info.origin.position.x, msg.info.origin.position.y);
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      have_native_grid_map_ = true;
      active_costmap_ = std::move(costmap);
      active_width_ = static_cast<int>(msg.info.width);
      active_height_ = static_cast<int>(msg.info.height);
      active_resolution_ = msg.info.resolution;
      active_origin_x_ = msg.info.origin.position.x;
      active_origin_y_ = msg.info.origin.position.y;
    }
    if (costmap_pub_) costmap_pub_->publish(msg);
  }

  bool acceptGoal(double x, double y, double yaw, const std::string& frame_id) {
    if (!global_planner_ || !local_controller_) return false;

    geometry_msgs::msg::PoseStamped goal_msg;
    goal_msg.header.frame_id = frame_id.empty() ? runtime_params_.world_frame : frame_id;
    goal_msg.header.stamp = now();
    goal_msg.pose.position.x = x;
    goal_msg.pose.position.y = y;
    goal_msg.pose.orientation.w = std::cos(yaw * 0.5);
    goal_msg.pose.orientation.z = std::sin(yaw * 0.5);

    if (goal_msg.header.frame_id != runtime_params_.world_frame) {
      try {
        goal_msg = tf_buffer_->transform(
            goal_msg, runtime_params_.world_frame, tf2::durationFromSec(0.2));
      } catch (const std::exception& exception) {
        RCLCPP_ERROR(get_logger(), "Goal frame transform failed: %s", exception.what());
        publishStatus(true, native::PlannerStatus::kError);
        return false;
      }
    }

    SE3Pose current;
    std::vector<uint8_t> costmap;
    int width = 0;
    int height = 0;
    Scalar resolution = 0.0;
    Scalar origin_x = 0.0;
    Scalar origin_y = 0.0;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      costmap = active_costmap_;
      width = active_width_;
      height = active_height_;
      resolution = active_resolution_;
      origin_x = active_origin_x_;
      origin_y = active_origin_y_;
    }
    if (!havePoseOrTF() || costmap.empty()) {
      RCLCPP_WARN(get_logger(), "Rejecting goal: no /ODOM/TF or native costmap yet");
      publishStatus(true, native::PlannerStatus::kError);
      return false;
    }
    // /ODOM is optional when the native TF tree is already publishing
    // camera_init -> base_link.  Use the same source that admitted the goal;
    // otherwise a TF-only robot would accidentally plan from identity.
    current = currentPose();

    global_planner_->setCostmap(costmap, width, height, resolution, origin_x, origin_y);
    local_controller_->resume();
    publishStatus(true, native::PlannerStatus::kPlanning);
    const Eigen::Matrix<Scalar, 3, 1> start = poseVector(current);
    const Eigen::Matrix<Scalar, 3, 1> goal(
        goal_msg.pose.position.x, goal_msg.pose.position.y,
        std::atan2(
            2.0 * (goal_msg.pose.orientation.w * goal_msg.pose.orientation.z +
                   goal_msg.pose.orientation.x * goal_msg.pose.orientation.y),
            1.0 - 2.0 * (goal_msg.pose.orientation.y * goal_msg.pose.orientation.y +
                         goal_msg.pose.orientation.z * goal_msg.pose.orientation.z)));
    const bool ok = global_planner_->plan(start, goal);
    if (!ok) {
      global_planner_->clearPlan();
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        goal_active_ = false;
      }
      publishStatus(true, native::PlannerStatus::kError);
      RCLCPP_WARN(get_logger(), "Native HybridAstar adaptation failed to find a path");
      return false;
    }

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      goal_active_ = true;
      target_goal_ = goal_msg;
    }
    if (target_goal_pub_) target_goal_pub_->publish(goal_msg);
    if (goal_baselink_pub_) goal_baselink_pub_->publish(toBaseLink(goal_msg));
    publishStatus(true, native::PlannerStatus::kRunning);
    return true;
  }

  bool havePoseOrTF() const {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (have_pose_) return true;
    }
    if (!tf_buffer_) return false;
    try {
      (void)tf_buffer_->lookupTransform(
          runtime_params_.world_frame, runtime_params_.body_frame, tf2::TimePointZero);
      return true;
    } catch (...) {
      return false;
    }
  }

  void cancelNavigation(std::int32_t command) {
    static_cast<void>(command);
    if (local_controller_) local_controller_->cancel();
    if (global_planner_) global_planner_->clearPlan();
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      goal_active_ = false;
    }
    publishStatus(true, native::PlannerStatus::kCanceled);
    publishStatus(false, native::PlannerStatus::kCanceled);
    publishVelocity(control::VelocityCommand{});
  }

  void applyRuntimeLocalParams(const native::LocalParamUpdate& update) {
    if (!local_controller_) return;
    controller_params_.direct_line_mode = update.directline_mode;
    controller_params_.native_max_speed_x = update.max_speed_x;
    controller_params_.native_max_speed_y = update.max_speed_y;
    controller_params_.native_max_theta = update.max_theta;
    controller_params_.barking_deceleration = update.barking_deceleration;
    controller_params_.xy_tolerance = update.xy_tolerance;
    controller_params_.yaw_tolerance = update.yaw_tolerance;
    controller_params_.stop_distance = update.stop_distance;
    controller_params_.judge_close = update.judge_close;
    controller_params_.backward_mode = update.backward_mode;
    controller_params_.dl_min_yaw = update.dl_min_yaw;
    controller_params_.dl_min_x = update.dl_min_x;
    controller_params_.dl_min_y = update.dl_min_y;
    controller_params_.dl_line_yaw_min = update.dl_line_yaw_min;
    controller_params_.max_linear_vel_x = update.max_speed_x;
    controller_params_.max_linear_vel_y = update.max_speed_y;
    controller_params_.max_angular_vel = update.max_theta;
    controller_params_.lineplanner_yaw_threshold = update.dl_line_yaw_min;
    local_controller_->setParams(controller_params_);
  }

  // --------------------------------------------------------------------------
  // Output and control loop
  // --------------------------------------------------------------------------
  SE3Pose currentPose() const {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (have_pose_) return current_pose_;
    }
    if (tf_buffer_) {
      try {
        return poseFromTransform(tf_buffer_->lookupTransform(
            runtime_params_.world_frame, runtime_params_.body_frame, tf2::TimePointZero));
      } catch (...) {
      }
    }
    return SE3Pose::Identity();
  }

  control::VelocityCommand currentVelocity() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return measured_velocity_;
  }

  void controlLoop() {
    if (!global_planner_ || !local_controller_) return;
    const std::vector<Eigen::Matrix<Scalar, 3, 1>> plan = global_planner_->getPlan();
    bool active = false;
    std::vector<uint8_t> costmap;
    int width = 0;
    int height = 0;
    Scalar resolution = 0.0;
    Scalar origin_x = 0.0;
    Scalar origin_y = 0.0;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      active = goal_active_;
      costmap = active_costmap_;
      width = active_width_;
      height = active_height_;
      resolution = active_resolution_;
      origin_x = active_origin_x_;
      origin_y = active_origin_y_;
    }
    if (!active || plan.empty() || costmap.empty()) return;

    const auto pose = currentPose();
    const auto command = local_controller_->step(
        poseVector(pose), currentVelocity(), plan, costmap, width, height,
        resolution, origin_x, origin_y);
    publishLocalPlan(plan);
    publishLocalGoal(plan, pose);
    publishStatus(false, native::PlannerStatus::kRunning);

    const Scalar distance = std::hypot(
        pose.t.x() - plan.back().x(), pose.t.y() - plan.back().y());
    if (distance <= controller_params_.stop_distance &&
        std::hypot(command.vx, command.vy) < 1e-3 && std::abs(command.omega) < 1e-3) {
      publishStatus(false, native::PlannerStatus::kGoalReached);
    }
  }

  void publishVelocity(const control::VelocityCommand& command) {
    if (!runtime_params_.enable_motion_output) return;
    if (native_bridge_active_) {
      if (!native_bridge_.publishNavCmd(command)) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000, "Native /NAV_CMD write has no matched consumer");
      }
      return;
    }
#ifndef M20_HAS_DRDDS
    if (cmd_vel_pub_) {
      geometry_msgs::msg::Twist twist;
      twist.linear.x = command.vx;
      twist.linear.y = command.vy;
      twist.angular.z = command.omega;
      cmd_vel_pub_->publish(twist);
    }
#endif
  }

  void publishStatus(bool global, native::PlannerStatus status, std::int32_t warning = 0) {
    const auto value = static_cast<std::int32_t>(status);
    if (native_bridge_active_ && native_bridge_.publishPlannerStatus(global, value, warning)) {
      return;
    }
#ifndef M20_HAS_DRDDS
    auto* publisher = global ? global_status_pub_.get() : local_status_pub_.get();
    if (publisher) {
      std_msgs::msg::Int32 message;
      message.data = value;
      publisher->publish(message);
    }
#else
    static_cast<void>(warning);
#endif
  }

  void publishCostmap(const std::vector<uint8_t>& costmap, int width, int height,
                      Scalar resolution, Scalar origin_x, Scalar origin_y,
                      const builtin_interfaces::msg::Time& stamp) {
    if (!costmap_pub_) return;
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.stamp = stamp;
    grid.header.frame_id = runtime_params_.world_frame;
    grid.info.resolution = resolution;
    grid.info.width = static_cast<uint32_t>(std::max(width, 0));
    grid.info.height = static_cast<uint32_t>(std::max(height, 0));
    grid.info.origin.position.x = origin_x;
    grid.info.origin.position.y = origin_y;
    grid.info.origin.orientation.w = 1.0;
    grid.data.reserve(costmap.size());
    for (const auto value : costmap) {
      grid.data.push_back(value >= 254 ? 100 : static_cast<int8_t>(
          std::clamp(static_cast<int>(std::lround(value * 100.0 / 253.0)), 0, 100)));
    }
    costmap_pub_->publish(grid);
  }

  void onPlan(const std::vector<Eigen::Matrix<Scalar, 3, 1>>& path) {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      latest_plan_ = path;
    }
    nav_msgs::msg::Path message;
    message.header.stamp = now();
    message.header.frame_id = runtime_params_.world_frame;
    for (const auto& point : path) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = message.header;
      pose.pose.position.x = point.x();
      pose.pose.position.y = point.y();
      pose.pose.orientation.w = std::cos(point.z() * 0.5);
      pose.pose.orientation.z = std::sin(point.z() * 0.5);
      message.poses.push_back(pose);
    }
    if (plan_pub_) plan_pub_->publish(message);
    if (global_path_pub_) global_path_pub_->publish(message);
    if (markers_pub_) publishPathMarkers(path, message.header);
    if (visible_points_pub_ || pruned_visible_points_pub_) publishPathCloud(path, message.header);
  }

  void publishLocalPlan(const std::vector<Eigen::Matrix<Scalar, 3, 1>>& path) {
    if (!local_path_pub_ && !track_path_pub_) return;
    nav_msgs::msg::Path message;
    message.header.stamp = now();
    message.header.frame_id = runtime_params_.world_frame;
    Scalar distance = 0.0;
    for (std::size_t i = 0; i < path.size(); ++i) {
      if (i > 0) distance += std::hypot(
          path[i].x() - path[i-1].x(), path[i].y() - path[i-1].y());
      if (controller_params_.local_point_dis > 0.0 &&
          distance > controller_params_.local_point_dis) break;
      geometry_msgs::msg::PoseStamped pose;
      pose.header = message.header;
      pose.pose.position.x = path[i].x();
      pose.pose.position.y = path[i].y();
      pose.pose.orientation.w = std::cos(path[i].z() * 0.5);
      pose.pose.orientation.z = std::sin(path[i].z() * 0.5);
      message.poses.push_back(pose);
    }
    if (local_path_pub_) local_path_pub_->publish(message);
    if (track_path_pub_) {
      nav_msgs::msg::Path base_link_path = message;
      base_link_path.header.frame_id = runtime_params_.body_frame;
      const auto pose = currentPose();
      const auto inverse = pose.inverse();
      for (auto& waypoint : base_link_path.poses) {
        const Eigen::Matrix<Scalar, 3, 1> world(
            waypoint.pose.position.x, waypoint.pose.position.y, 0.0);
        const auto local = inverse.transformPoint(world);
        waypoint.pose.position.x = local.x();
        waypoint.pose.position.y = local.y();
      }
      track_path_pub_->publish(base_link_path);
    }
  }

  void publishLocalGoal(const std::vector<Eigen::Matrix<Scalar, 3, 1>>& path,
                        const SE3Pose& robot_pose) {
    if (path.empty()) return;
    std::size_t closest = 0;
    Scalar closest_distance = std::numeric_limits<Scalar>::max();
    for (std::size_t i = 0; i < path.size(); ++i) {
      const Scalar distance = std::hypot(
          path[i].x() - robot_pose.t.x(), path[i].y() - robot_pose.t.y());
      if (distance < closest_distance) {
        closest_distance = distance;
        closest = i;
      }
    }

    std::size_t selected = closest;
    Scalar accumulated = 0.0;
    const Scalar lookahead = std::max(controller_params_.look_ahead_dis, Scalar(0.1));
    const std::size_t search_window = static_cast<std::size_t>(
        std::max(planner_params_.local_goal_search_window, 1));
    const std::size_t search_end = std::min(path.size(), closest + search_window + 1U);
    for (std::size_t i = closest + 1U; i < search_end; ++i) {
      accumulated += std::hypot(
          path[i].x() - path[i-1].x(), path[i].y() - path[i-1].y());
      if (accumulated >= lookahead) {
        selected = i;
        break;
      }
    }
    if (selected == closest && closest + 1U < search_end) {
      selected = search_end - 1U;
    }
    geometry_msgs::msg::PoseStamped goal;
    goal.header.stamp = now();
    goal.header.frame_id = runtime_params_.world_frame;
    goal.pose.position.x = path[selected].x();
    goal.pose.position.y = path[selected].y();
    goal.pose.orientation.w = std::cos(path[selected].z() * 0.5);
    goal.pose.orientation.z = std::sin(path[selected].z() * 0.5);
    if (local_goal_pub_) local_goal_pub_->publish(goal);
    if (local_goal_baselink_pub_) local_goal_baselink_pub_->publish(toBaseLink(goal, robot_pose));
  }

  geometry_msgs::msg::PoseStamped toBaseLink(
      const geometry_msgs::msg::PoseStamped& world_goal,
      const SE3Pose& robot_pose = SE3Pose::Identity()) const {
    SE3Pose pose = robot_pose;
    if (robot_pose.q.isApprox(Eigen::Quaternion<Scalar>::Identity()) &&
        robot_pose.t.norm() < 1e-9) {
      pose = currentPose();
    }
    const auto local = pose.inverse().transformPoint(Eigen::Matrix<Scalar, 3, 1>(
        world_goal.pose.position.x, world_goal.pose.position.y, world_goal.pose.position.z));
    geometry_msgs::msg::PoseStamped output = world_goal;
    output.header.frame_id = runtime_params_.body_frame;
    output.pose.position.x = local.x();
    output.pose.position.y = local.y();
    output.pose.position.z = local.z();
    output.pose.orientation.w = 1.0;
    output.pose.orientation.x = output.pose.orientation.y = output.pose.orientation.z = 0.0;
    return output;
  }

  void publishPathCloud(const std::vector<Eigen::Matrix<Scalar, 3, 1>>& path,
                       const std_msgs::msg::Header& header) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.reserve(path.size());
    for (const auto& point : path) {
      cloud.push_back(pcl::PointXYZ(static_cast<float>(point.x()),
                                    static_cast<float>(point.y()), 0.0f));
    }
    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(cloud, message);
    message.header = header;
    if (visible_points_pub_) visible_points_pub_->publish(message);
    if (pruned_visible_points_pub_) pruned_visible_points_pub_->publish(message);
    if (free_paths_pub_) free_paths_pub_->publish(message);
  }

  void publishPathMarkers(const std::vector<Eigen::Matrix<Scalar, 3, 1>>& path,
                          const std_msgs::msg::Header& header) {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker line;
    line.header = header;
    line.ns = "m20_native_global_path";
    line.id = 0;
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.scale.x = 0.03;
    line.color.r = 0.1f;
    line.color.g = 0.9f;
    line.color.b = 0.1f;
    line.color.a = 1.0f;
    for (const auto& point : path) {
      geometry_msgs::msg::Point p;
      p.x = point.x();
      p.y = point.y();
      line.points.push_back(p);
    }
    markers.markers.push_back(line);
    markers_pub_->publish(markers);
  }

  TerrainParams terrain_params_;
  GlobalPlannerParams planner_params_;
  LocalControllerParams controller_params_;
  NavigationRuntimeParams runtime_params_;

  std::shared_ptr<terrain::TraversabilityMap> traversability_map_;
  std::shared_ptr<planning::GlobalPlannerNode> global_planner_;
  std::shared_ptr<control::LocalControllerNode> local_controller_;
  ros::NativeNavigationBridge native_bridge_;
  bool native_bridge_active_{false};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr accumulated_cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
#ifndef M20_HAS_DRDDS
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr planner_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr cancel_nav_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr cancel_global_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr cancel_planner_service_;
#endif

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_goal_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_goal_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_baselink_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_goal_baselink_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr track_path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr visible_points_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pruned_visible_points_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr free_paths_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_scans_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr grid_map_3d_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
#ifndef M20_HAS_DRDDS
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr global_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr local_status_pub_;
#else
  // Native DrDDS publishers are owned by NativeNavigationBridge.
#endif
  rclcpp::TimerBase::SharedPtr autostart_timer_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  mutable std::mutex state_mutex_;
  SE3Pose current_pose_;
  SE3Pose initial_pose_;
  control::VelocityCommand measured_velocity_;
  geometry_msgs::msg::PoseStamped target_goal_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_;
  std::vector<Eigen::Matrix<Scalar, 3, 1>> latest_plan_;
  std::vector<uint8_t> active_costmap_;
  int active_width_{0};
  int active_height_{0};
  Scalar active_resolution_{0.0};
  Scalar active_origin_x_{0.0};
  Scalar active_origin_y_{0.0};
  bool have_pose_{false};
  bool have_motion_info_{false};
  bool have_native_grid_map_{false};
  bool goal_active_{false};
};

}  // namespace m20::nodes

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(m20::nodes::NavigationNode)
