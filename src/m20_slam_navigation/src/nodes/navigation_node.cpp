/**
 * @file navigation_node.cpp
 * @brief ROS 2 Lifecycle Node: Autonomous Navigation (Global + Local Planner).
 *
 * Subscribes: /TERRAIN_TRAVERSABILITY_MAP, /goal_pose, /tf
 * Publishes:   /cmd_vel, /plan, /TRACK_PATH
 */

#include "m20_slam_navigation/common/types.hpp"
#include "m20_slam_navigation/common/params.hpp"
#include "m20_slam_navigation/terrain/traversability_map.hpp"
#include "m20_slam_navigation/planning/global_planner_node.hpp"
#include "m20_slam_navigation/control/local_controller_node.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <pcl_conversions/pcl_conversions.h>

#include <memory>

namespace m20::nodes {

class NavigationNode : public rclcpp_lifecycle::LifecycleNode {
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit NavigationNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : rclcpp_lifecycle::LifecycleNode("navigation_node", options) {}

  CallbackReturn on_configure(const rclcpp_lifecycle::State&) override {
    RCLCPP_INFO(get_logger(), "NavigationNode: configuring...");

    // Load params
    declare_parameter("terrain.grid_resolution", 0.05);
    declare_parameter("terrain.max_climb_angle", 0.5236);  // 30°
    declare_parameter("terrain.max_step_height", 0.20);
    declare_parameter("planner.grid_resolution", 0.1);
    declare_parameter("planner.max_curvature", 0.5);
    declare_parameter("controller.max_linear_vel_x", 1.0);
    declare_parameter("controller.max_angular_vel", 1.0);

    terrain_params_.grid_resolution = get_parameter("terrain.grid_resolution").as_double();
    terrain_params_.max_climb_angle = get_parameter("terrain.max_climb_angle").as_double();
    terrain_params_.max_step_height = get_parameter("terrain.max_step_height").as_double();

    planner_params_.grid_resolution = get_parameter("planner.grid_resolution").as_double();
    planner_params_.max_curvature   = get_parameter("planner.max_curvature").as_double();

    controller_params_.max_linear_vel_x  = get_parameter("controller.max_linear_vel_x").as_double();
    controller_params_.max_angular_vel   = get_parameter("controller.max_angular_vel").as_double();

    // Create components
    traversability_map_ = std::make_shared<terrain::TraversabilityMap>(terrain_params_);
    global_planner_     = std::make_shared<planning::GlobalPlannerNode>(
        planner_params_, terrain_params_);
    local_controller_   = std::make_shared<control::LocalControllerNode>(controller_params_);

    // Wire callbacks
    global_planner_->setPlanCallback(
        [this](const std::vector<Eigen::Matrix<Scalar, 3, 1>>& path) {
          publishPlan(path);
        });

    local_controller_->setVelocityCallback(
        [this](const control::VelocityCommand& cmd) {
          publishVelocity(cmd);
        });

    // TF listener
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Subscriptions
    lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/LIDAR/pointcloud", rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
          pcl::fromROSMsg(*msg, *cloud);

          SE3Pose T_world_lidar = getCurrentPose();
          traversability_map_->update(cloud, T_world_lidar);

          // Periodically publish costmap
          static auto last_pub = std::chrono::steady_clock::now();
          auto now = std::chrono::steady_clock::now();
          if (std::chrono::duration<double>(now - last_pub).count() > 0.5) {
            publishCostmap();
            last_pub = now;
          }
        });

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10,
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          Eigen::Matrix<Scalar, 3, 1> goal(
              msg->pose.position.x,
              msg->pose.position.y,
              0);  // yaw from quaternion
          Eigen::Matrix<Scalar, 3, 1> start(
              current_pose_x_, current_pose_y_, current_pose_yaw_);

          bool ok = global_planner_->plan(start, goal);
          RCLCPP_INFO(get_logger(), "Plan %s", ok ? "found" : "FAILED");
        });

    // Publishers
    costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/TERRAIN_TRAVERSABILITY_MAP", 10);
    plan_pub_    = create_publisher<nav_msgs::msg::Path>("/plan", 10);
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Control loop timer (50Hz)
    control_timer_ = create_wall_timer(
        std::chrono::milliseconds(20),
        [this]() { controlLoop(); });

    RCLCPP_INFO(get_logger(), "NavigationNode: configured OK");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State&) override {
    RCLCPP_INFO(get_logger(), "NavigationNode: activating...");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override {
    RCLCPP_INFO(get_logger(), "NavigationNode: deactivating...");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override {
    lidar_sub_.reset();
    goal_sub_.reset();
    costmap_pub_.reset();
    plan_pub_.reset();
    cmd_vel_pub_.reset();
    traversability_map_.reset();
    global_planner_.reset();
    local_controller_.reset();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State&) override {
    return CallbackReturn::SUCCESS;
  }

private:
  SE3Pose getCurrentPose() {
    try {
      auto tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
      SE3Pose pose;
      pose.t = Eigen::Matrix<Scalar, 3, 1>(
          tf.transform.translation.x,
          tf.transform.translation.y,
          tf.transform.translation.z);
      pose.q = Eigen::Quaternion<Scalar>(
          tf.transform.rotation.w,
          tf.transform.rotation.x,
          tf.transform.rotation.y,
          tf.transform.rotation.z);
      return pose;
    } catch (...) {
      return SE3Pose::Identity();
    }
  }

  void controlLoop() {
    SE3Pose pose = getCurrentPose();
    current_pose_x_   = pose.t.x();
    current_pose_y_   = pose.t.y();
    current_pose_yaw_ = math::quaternion_to_yaw(pose.q);

    Eigen::Matrix<Scalar, 3, 1> current_pose_vec(
        current_pose_x_, current_pose_y_, current_pose_yaw_);

    control::VelocityCommand current_vel{0, 0, 0};  // TODO: track actual velocity

    auto plan = global_planner_->getPlan();
    if (plan.empty()) return;

    auto costmap = traversability_map_->exportCostmap();

    auto cmd = local_controller_->step(
        current_pose_vec, current_vel, plan,
        costmap, traversability_map_->width(), traversability_map_->height(),
        traversability_map_->resolution(), 0.0, 0.0  // TODO: proper origin
    );

    geometry_msgs::msg::Twist twist;
    twist.linear.x  = cmd.vx;
    twist.linear.y  = cmd.vy;
    twist.angular.z = cmd.omega;
    cmd_vel_pub_->publish(twist);
  }

  void publishCostmap() {
    auto costmap = traversability_map_->exportCostmap();
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.stamp = now();
    grid.header.frame_id = "map";
    grid.info.resolution = traversability_map_->resolution();
    grid.info.width  = traversability_map_->width();
    grid.info.height = traversability_map_->height();
    grid.info.origin.position.x = 0;  // TODO: proper origin
    grid.info.origin.position.y = 0;
    grid.data = costmap;
    costmap_pub_->publish(grid);
  }

  void publishPlan(const std::vector<Eigen::Matrix<Scalar, 3, 1>>& path) {
    nav_msgs::msg::Path plan_msg;
    plan_msg.header.stamp = now();
    plan_msg.header.frame_id = "map";
    for (const auto& wp : path) {
      geometry_msgs::msg::PoseStamped ps;
      ps.pose.position.x = wp.x();
      ps.pose.position.y = wp.y();
      ps.pose.position.z = 0;
      plan_msg.poses.push_back(ps);
    }
    plan_pub_->publish(plan_msg);
  }

  void publishVelocity(const control::VelocityCommand& cmd) {
    geometry_msgs::msg::Twist twist;
    twist.linear.x  = cmd.vx;
    twist.linear.y  = cmd.vy;
    twist.angular.z = cmd.omega;
    cmd_vel_pub_->publish(twist);
  }

  // Params
  TerrainParams                       terrain_params_;
  GlobalPlannerParams                 planner_params_;
  LocalControllerParams               controller_params_;

  // Components
  std::shared_ptr<terrain::TraversabilityMap>    traversability_map_;
  std::shared_ptr<planning::GlobalPlannerNode>   global_planner_;
  std::shared_ptr<control::LocalControllerNode>  local_controller_;

  // TF
  std::shared_ptr<tf2_ros::Buffer>              tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener>   tf_listener_;

  // ROS 2
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr       costmap_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                plan_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr          cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr                                      control_timer_;

  // State
  Scalar current_pose_x_{0}, current_pose_y_{0}, current_pose_yaw_{0};
};

}  // namespace m20::nodes

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(m20::nodes::NavigationNode)