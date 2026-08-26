/**
 * @file localization_node.cpp
 * @brief ROS 2 Lifecycle Node: Prior Map Relocalization.
 *
 * Subscribes: /LIDAR/pointcloud, /IMU, /ODOM, /initialpose
 * Publishes:   /tf (map→odom corrected), localization pose
 */

#include "m20_slam_navigation/common/types.hpp"
#include "m20_slam_navigation/common/params.hpp"
#include "m20_slam_navigation/localization/relocalizer.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>

#include <memory>
#include <string>

namespace m20::nodes {

class LocalizationNode : public rclcpp_lifecycle::LifecycleNode {
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit LocalizationNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : rclcpp_lifecycle::LifecycleNode("localization_node", options) {}

  CallbackReturn on_configure(const rclcpp_lifecycle::State&) override {
    RCLCPP_INFO(get_logger(), "LocalizationNode: configuring...");

    // Declare parameters
    declare_parameter("map_path", "full_cloud.pcd");
    declare_parameter("localization.ndt_resolution", 1.0);
    declare_parameter("localization.ndt_max_iterations", 30);
    declare_parameter("localization.num_hypotheses", 8);
    declare_parameter("localization.hypothesis_trans_range", 5.0);

    std::string map_path = get_parameter("map_path").as_string();
    loc_params_.ndt_resolution = get_parameter("localization.ndt_resolution").as_double();
    loc_params_.ndt_max_iterations = get_parameter("localization.ndt_max_iterations").as_int();
    loc_params_.num_hypotheses = get_parameter("localization.num_hypotheses").as_int();
    loc_params_.hypothesis_trans_range = get_parameter("localization.hypothesis_trans_range").as_double();
    loc_params_.map_path = map_path;

    // Create relocalizer
    relocalizer_ = std::make_shared<localization::Relocalizer>(
        loc_params_, sensor_params_);

    if (!relocalizer_->loadMap(map_path)) {
      RCLCPP_ERROR(get_logger(), "Failed to load map: %s", map_path.c_str());
      return CallbackReturn::FAILURE;
    }

    RCLCPP_INFO(get_logger(), "Loaded map: %s with %lu points",
                map_path.c_str(),
                relocalizer_->getCurrentPose().pose.t.x());  // placeholder

    // Subscriptions
    lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/LIDAR/pointcloud", rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
          pcl::fromROSMsg(*msg, *cloud);

          if (!relocalizer_->isLocalized()) {
            // Attempt relocalization
            PoseWithCovariance init_guess;
            relocalizer_->relocalize(cloud, init_guess);
          } else {
            relocalizer_->updateNDT(cloud);
          }
        });

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/IMU", rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
          ImuPacket imu;
          imu.stamp = std::chrono::steady_clock::now();
          imu.accel = Eigen::Matrix<Scalar, 3, 1>(
              msg->linear_acceleration.x,
              msg->linear_acceleration.y,
              msg->linear_acceleration.z);
          imu.gyro = Eigen::Matrix<Scalar, 3, 1>(
              msg->angular_velocity.x,
              msg->angular_velocity.y,
              msg->angular_velocity.z);
          relocalizer_->predict(imu);
        });

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/ODOM", rclcpp::SensorDataQoS(),
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          FootOdomPacket odom;
          odom.stamp = std::chrono::steady_clock::now();
          odom.pose.t = Eigen::Matrix<Scalar, 3, 1>(
              msg->pose.pose.position.x,
              msg->pose.pose.position.y,
              msg->pose.pose.position.z);
          odom.pose.q = Eigen::Quaternion<Scalar>(
              msg->pose.pose.orientation.w,
              msg->pose.pose.orientation.x,
              msg->pose.pose.orientation.y,
              msg->pose.pose.orientation.z);
          relocalizer_->updateOdometry(odom);
        });

    initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 10,
        [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr) {
          // Handle initial pose from RViz
          RCLCPP_INFO(get_logger(), "Received initial pose");
        });

    // TF broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    // Timer for TF publishing (50Hz)
    tf_timer_ = create_wall_timer(
        std::chrono::milliseconds(20),
        [this]() {
          if (!relocalizer_->isLocalized()) return;
          auto pwc = relocalizer_->getCurrentPose();

          geometry_msgs::msg::TransformStamped tf;
          tf.header.stamp = now();
          tf.header.frame_id = "map";
          tf.child_frame_id  = "odom";
          tf.transform.translation.x = pwc.pose.t.x();
          tf.transform.translation.y = pwc.pose.t.y();
          tf.transform.translation.z = pwc.pose.t.z();
          tf.transform.rotation.w = pwc.pose.q.w();
          tf.transform.rotation.x = pwc.pose.q.x();
          tf.transform.rotation.y = pwc.pose.q.y();
          tf.transform.rotation.z = pwc.pose.q.z();
          tf_broadcaster_->sendTransform(tf);
        });

    RCLCPP_INFO(get_logger(), "LocalizationNode: configured OK");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State&) override {
    RCLCPP_INFO(get_logger(), "LocalizationNode: activating...");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override {
    RCLCPP_INFO(get_logger(), "LocalizationNode: deactivating...");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override {
    RCLCPP_INFO(get_logger(), "LocalizationNode: cleaning up...");
    lidar_sub_.reset();
    imu_sub_.reset();
    odom_sub_.reset();
    initial_pose_sub_.reset();
    relocalizer_.reset();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State&) override {
    return CallbackReturn::SUCCESS;
  }

private:
  SensorParams                              sensor_params_;
  LocalizationParams                        loc_params_;
  std::shared_ptr<localization::Relocalizer> relocalizer_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr        lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr                imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr              odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster>                        tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr                                          tf_timer_;
};

}  // namespace m20::nodes

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(m20::nodes::LocalizationNode)