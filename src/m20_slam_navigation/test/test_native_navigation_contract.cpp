#include "m20_slam_navigation/common/native_navigation.hpp"
#include "m20_slam_navigation/common/params.hpp"
#include "m20_slam_navigation/control/local_controller_node.hpp"
#include "m20_slam_navigation/planning/hybrid_astar.hpp"
#include "m20_slam_navigation/planning/motion_primitives.hpp"
#include "m20_slam_navigation/terrain/traversability_map.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace {

using m20::GlobalPlannerParams;
using m20::LocalControllerParams;
using m20::Scalar;

TEST(NativeNavigationContract, TopicsAndDefaultsMatchM20Pro) {
  m20::NavigationTopics topics;
  EXPECT_EQ(topics.grid_map, "/GRID_MAP");
  EXPECT_EQ(topics.odom, "/ODOM");
  EXPECT_EQ(topics.motion_info, "/MOTION_INFO");
  EXPECT_EQ(topics.nav_points, "/NAV_POINTS");
  EXPECT_EQ(topics.astar_path, "/path_Astar");
  EXPECT_EQ(topics.local_goal, "/local_goal");
  EXPECT_EQ(topics.nav_cmd, "/NAV_CMD");
  EXPECT_EQ(topics.planner_status, "/PLANNER_STATUS");
  EXPECT_EQ(topics.global_planner_status, "/GLOBAL_PLANNER_STATUS");
  EXPECT_EQ(topics.goal_planner_service, "/GOAL_PLANNER");
  EXPECT_EQ(topics.cancel_planner_service, "/CANCEL_NAV_PLANNER");

  GlobalPlannerParams global;
  EXPECT_DOUBLE_EQ(global.step_size, 0.4);
  EXPECT_DOUBLE_EQ(global.sample_interval, 0.2);
  EXPECT_DOUBLE_EQ(global.body_length, 0.45);
  EXPECT_DOUBLE_EQ(global.body_width, 0.45);
  EXPECT_DOUBLE_EQ(global.max_steer, 0.9);
  EXPECT_EQ(global.num_steerind, 3);
  EXPECT_DOUBLE_EQ(global.goal_dis, 0.5);
  EXPECT_DOUBLE_EQ(global.astar_time, 1.0);
  EXPECT_EQ(global.local_goal_search_window, 10);

  LocalControllerParams local;
  EXPECT_DOUBLE_EQ(local.look_ahead_dis, 1.5);
  EXPECT_DOUBLE_EQ(local.native_max_speed_x, 1.5);
  EXPECT_DOUBLE_EQ(local.native_max_speed_y, 0.6);
  EXPECT_DOUBLE_EQ(local.native_max_theta, 1.0);
  EXPECT_DOUBLE_EQ(local.stop_distance, 0.7);
  EXPECT_DOUBLE_EQ(local.robot_length, 0.84);
  EXPECT_DOUBLE_EQ(local.robot_width, 0.5);
  EXPECT_EQ(local.path_num, 4641);
  EXPECT_EQ(local.path_sample_num, 10);
}

TEST(NativeLocalizationContract, TopicsFramesAndDefaultsMatchM20Pro) {
  m20::LocalizationTopics topics;
  EXPECT_EQ(topics.input_lidar, "/LIDAR/POINTS");
  EXPECT_EQ(topics.input_imu, "/IMU");
  EXPECT_EQ(topics.input_rtk, "/GPYBM");
  EXPECT_EQ(topics.leg_odom, "/leg_odom");
  EXPECT_EQ(topics.output_odom, "/ODOM");
  EXPECT_EQ(topics.output_enu, "/RTK_RAW_ODOM");
  EXPECT_EQ(topics.output_global_map, "/FULL_CLOUD_MAP");
  EXPECT_EQ(topics.output_body_cloud, "/LOC_BODY_POINTS");

  m20::LocalizationRuntimeParams runtime;
  EXPECT_EQ(runtime.world_frame, "camera_init");
  EXPECT_EQ(runtime.odom_frame, "odom");
  EXPECT_EQ(runtime.body_frame, "base_link");

  m20::LocalizationParams localization;
  EXPECT_DOUBLE_EQ(localization.imu_gravity, 9.80511);
}

TEST(NativeNavigationContract, NativePrimitiveSetHasEightDirectionsAndThreeSteers) {
  GlobalPlannerParams params;
  const auto primitives = m20::planning::MotionPrimitives::generateOmnidirectional(
      8, params.step_size, params.max_curvature, params.num_steerind,
      params.max_steer * params.sample_interval);
  // 8 translation directions x 3 steering choices plus three in-place yaw
  // choices used by the adapter.
  EXPECT_EQ(primitives.size(), 27U);
  EXPECT_TRUE(std::any_of(primitives.begin(), primitives.end(), [](const auto& primitive) {
    return std::abs(primitive.dtheta) > 0.0;
  }));
}

TEST(NativeNavigationContract, HybridAstarPlansOnFreeNativeGrid) {
  GlobalPlannerParams params;
  params.time_limit_sec = 0.5;
  params.astar_time = 0.5;
  params.max_expansions = 100000;
  m20::TerrainParams terrain;
  m20::planning::HybridAStar planner(params, terrain);

  constexpr int width = 80;
  constexpr int height = 80;
  constexpr Scalar resolution = 0.1;
  constexpr Scalar origin = -4.0;
  std::vector<uint8_t> costmap(width * height, 0);
  const Eigen::Matrix<Scalar, 3, 1> start(-2.5, -2.5, 0.0);
  const Eigen::Matrix<Scalar, 3, 1> goal(2.0, 1.5, 0.0);

  const auto path = planner.plan(
      costmap, width, height, resolution, origin, origin, start, goal);
  ASSERT_FALSE(path.empty());
  EXPECT_NEAR(path.front().x(), start.x(), 1e-9);
  EXPECT_NEAR(path.front().y(), start.y(), 1e-9);
  EXPECT_NEAR(path.back().x(), goal.x(), 1e-9);
  EXPECT_NEAR(path.back().y(), goal.y(), 1e-9);
}

TEST(NativeNavigationContract, HybridAstarRejectsLethalGoalFootprint) {
  GlobalPlannerParams params;
  params.time_limit_sec = 0.2;
  params.astar_time = 0.2;
  params.max_expansions = 10000;
  m20::TerrainParams terrain;
  m20::planning::HybridAStar planner(params, terrain);

  constexpr int width = 60;
  constexpr int height = 60;
  constexpr Scalar resolution = 0.1;
  constexpr Scalar origin = -3.0;
  std::vector<uint8_t> costmap(width * height, 0);
  const Eigen::Matrix<Scalar, 3, 1> start(-1.5, 0.0, 0.0);
  const Eigen::Matrix<Scalar, 3, 1> goal(1.5, 0.0, 0.0);
  const int goal_x = static_cast<int>(std::floor((goal.x() - origin) / resolution));
  const int goal_y = static_cast<int>(std::floor((goal.y() - origin) / resolution));
  costmap[goal_y * width + goal_x] = 254;

  EXPECT_TRUE(planner.plan(
      costmap, width, height, resolution, origin, origin, start, goal).empty());
}

TEST(NativeNavigationContract, LocalPlannerUsesLineModeAndNativeLimits) {
  LocalControllerParams params;
  params.direct_line_mode = true;
  params.native_max_speed_x = 1.5;
  params.native_max_speed_y = 0.6;
  params.native_max_theta = 1.0;
  params.max_linear_vel_x = 1.5;
  params.max_linear_vel_y = 0.6;
  params.max_angular_vel = 1.0;
  params.monitor_loop_frequency = 10.0;

  m20::control::LocalControllerNode controller(params);
  const Eigen::Matrix<Scalar, 3, 1> pose(0.0, 0.0, 0.0);
  const std::vector<Eigen::Matrix<Scalar, 3, 1>> path{
      Eigen::Matrix<Scalar, 3, 1>(0.0, 0.0, 0.0),
      Eigen::Matrix<Scalar, 3, 1>(3.0, 0.0, 0.0)};
  const std::vector<uint8_t> costmap(200 * 200, 0);

  const auto command = controller.step(
      pose, m20::control::VelocityCommand{}, path, costmap,
      200, 200, 0.05, -5.0, -5.0);
  EXPECT_EQ(controller.getMode(), m20::control::LocalControllerNode::Mode::LINE);
  EXPECT_GT(command.vx, 0.0);
  EXPECT_LE(command.vx, params.native_max_speed_x);
  EXPECT_LE(std::abs(command.vy), params.native_max_speed_y);
  EXPECT_LE(std::abs(command.omega), params.native_max_theta);
}

TEST(NativeNavigationContract, TerrainFallbackUsesWorldFrameAndDoesNotAccumulateCost) {
  m20::TerrainParams params;
  params.map_length = 8.0;
  params.grid_resolution = 0.05;
  params.min_range = 0.1;
  params.max_range = 10.0;
  m20::terrain::TraversabilityMap map(params);

  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  for (int ix = -5; ix <= 5; ++ix) {
    for (int iy = -5; iy <= 5; ++iy) {
      pcl::PointXYZI point;
      point.x = 1.0F + static_cast<float>(ix) * 0.02F;
      point.y = static_cast<float>(iy) * 0.02F;
      point.z = 0.0F;
      cloud->push_back(point);
    }
  }
  m20::SE3Pose world_lidar;
  world_lidar.t.x() = 10.0;

  map.update(cloud, world_lidar);
  const auto first_cost = map.costAt(11.0, 0.0);
  EXPECT_LT(first_cost, 254);

  map.update(cloud, world_lidar);
  EXPECT_EQ(map.costAt(11.0, 0.0), first_cost);
}

}  // namespace
