#pragma once

/**
 * @file native_navigation.hpp
 * @brief Observable M20Pro navigation contract.
 *
 * The native planner binaries are closed source, therefore this header keeps
 * the part of their contract that can be verified from the deployed ROS graph
 * and configuration files in one place.  The algorithm implementations in
 * this repository consume these names and values; the optional DrDDS adapter
 * exposes the vendor message/service types when the robot SDK is installed.
 */

#include <cstdint>
#include <string>

namespace m20::native {

// Global planner input and output topics.
inline constexpr char kGridMapTopic[] = "/GRID_MAP";
inline constexpr char kInitialPoseTopic[] = "/initialpose";
inline constexpr char kGoalPoseTopic[] = "/goal_pose";
inline constexpr char kGoalGlobalService[] = "/GOAL_GLOBAL";
inline constexpr char kOdomTopic[] = "/ODOM";
inline constexpr char kCancelNavTopic[] = "/CANCEL_NAV";
inline constexpr char kCancelGlobalService[] = "/CANCEL_NAV_GLOBAL";
inline constexpr char kPlannerModeTopic[] = "/planner_mode";
inline constexpr char kMotionInfoTopic[] = "/MOTION_INFO";
inline constexpr char kNavPointsTopic[] = "/NAV_POINTS";
inline constexpr char kAstarPathTopic[] = "/path_Astar";
inline constexpr char kVisiblePointsTopic[] = "/vis_global_points";
inline constexpr char kPrunedVisiblePointsTopic[] = "/vis_global_points_pruned";
inline constexpr char kLocalGoalTopic[] = "/local_goal";
inline constexpr char kLocalMapTopic[] = "/local_map";
inline constexpr char kGlobalPlannerStatusTopic[] = "/GLOBAL_PLANNER_STATUS";

// Local planner input, output, diagnostics, and services.
inline constexpr char kNavCmdTopic[] = "/NAV_CMD";
inline constexpr char kPlannerStatusTopic[] = "/PLANNER_STATUS";
inline constexpr char kTargetGoalTopic[] = "/target_goal";
inline constexpr char kGoalBaseLinkTopic[] = "/goal_baselink";
inline constexpr char kLocalGoalBaseLinkTopic[] = "/local_goal_baselink";
inline constexpr char kFreePathsTopic[] = "/free_paths";
inline constexpr char kLocalPathTopic[] = "/local_path";
inline constexpr char kLocalScansTopic[] = "/local_scans";
inline constexpr char kTrackPathBaseLinkTopic[] = "/track_path_baselink";
inline constexpr char kGlobalPathTopic[] = "/global_path";
inline constexpr char kGridMap3DTopic[] = "/grid_map_3d";
inline constexpr char kGlobalPathMarkersTopic[] = "/global_path_markers";
inline constexpr char kSetParamService[] = "/set_service";
inline constexpr char kCancelPlannerService[] = "/CANCEL_NAV_PLANNER";
inline constexpr char kGoalPlannerService[] = "/GOAL_PLANNER";

// Native terrain/passable-area inputs and outputs.
inline constexpr char kAccumulatedCloudTopic[] = "/accumulate_cloud/cloud_gravity";
inline constexpr char kImuTopic[] = "/IMU";
inline constexpr char kPassableCloudTopic[] = "/passable_area";
inline constexpr char kImpassableCloudTopic[] = "/impassable_area";
inline constexpr char kGridMapLowerTopic[] = "/grid_map";
inline constexpr char kTraversalCostTopic[] = "/traversal_cost";

enum class PlannerStatus : std::int32_t {
  kIdle = 0,
  kPlanning = 1,
  kRunning = 2,
  kGoalReached = 3,
  kCanceled = 4,
  kError = -1,
};

/** Runtime fields accepted by the vendor SetParam service. */
struct LocalParamUpdate {
  bool directline_mode{true};
  double max_speed_x{1.5};
  double max_speed_y{0.6};
  double max_theta{1.0};
  double barking_deceleration{1.0};
  double xy_tolerance{0.1};
  double yaw_tolerance{0.1};
  double stop_distance{0.7};
  bool judge_close{false};
  bool backward_mode{false};
  double dl_min_yaw{0.7};
  double dl_min_x{0.25};
  double dl_min_y{0.25};
  double dl_line_yaw_min{0.2};
};

/** Common DrDDS runtime options used by native message/service adapters. */
struct DDSOptions {
  int domain_id{0};
  bool use_shm{false};
  std::string topic_prefix{"rt"};
  std::string network_name;
};

}  // namespace m20::native
