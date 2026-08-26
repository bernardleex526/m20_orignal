/**
 * @file slam_node.cpp
 * @brief ROS 2 Lifecycle Node: SLAM system (LIO Front-End + Back-End).
 *
 * Subscribes: /LIDAR/pointcloud, /IMU, /ODOM
 * Publishes:   /tf (map→odom→base_link→lidar_link), /map, /TRACK_PATH
 */

#include "m20_slam_navigation/common/types.hpp"
#include "m20_slam_navigation/common/params.hpp"
#include "m20_slam_navigation/lio/lio_odometry.hpp"
#include "m20_slam_navigation/backend/pose_graph_optimizer.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>

#include <memory>
#include <string>
#include <thread>

namespace m20::nodes {

class SlamNode : public rclcpp_lifecycle::LifecycleNode {
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit SlamNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : rclcpp_lifecycle::LifecycleNode("slam_node", options) {}

  CallbackReturn on_configure(const rclcpp_lifecycle::State&) override {
    RCLCPP_INFO(get_logger(), "SlamNode: configuring...");

    // Declare parameters
    declare_parameter("sensors.lidar_scan_lines", 16);
    declare_parameter("sensors.lidar_min_range", 0.1);
    declare_parameter("sensors.lidar_max_range", 200.0);
    declare_parameter("sensors.lidar_hz", 10.0);
    declare_parameter("sensors.imu_hz", 200.0);
    declare_parameter("sensors.odom_hz", 100.0);
    declare_parameter("lio.voxel_size", 0.5);
    declare_parameter("lio.max_voxels", 100000);
    declare_parameter("lio.keyframe_distance", 0.5);
    declare_parameter("lio.keyframe_angle", 0.2);
    declare_parameter("lio.max_iterations", 30);
    declare_parameter("lio.correspondence_radius", 1.0);
    declare_parameter("lio.num_threads", 4);
    declare_parameter("backend.degeneracy_threshold", 10.0);

    // --- Load params into structs ---
    loadParams();

    // --- Create components ---
    lio_odom_ = std::make_shared<lio::LIOOdometry>(
        sensor_params_, lio_params_, backend_params_);

    pose_graph_ = std::make_shared<backend::PoseGraphOptimizer>(backend_params_);

    // Wire LIO → Pose Graph
    lio_odom_->setOdometryCallback(
        [this](const PoseWithCovariance& pwc, FrameId fid) {
          // Send to back-end
          static FrameId prev_fid = 0;
          if (prev_fid != 0) {
            SE3Pose relative = getPose(prev_fid).pose.inverse() * pwc.pose;
            pose_graph_->addOdometry(prev_fid, fid, relative, pwc.covariance);
          }
          prev_fid = fid;

          // Publish TF
          publishTF(pwc.pose, fid);
        });

    lio_odom_->setKeyframeCallback(
        [this](FrameId fid, const SE3Pose& pose,
               const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
          pose_graph_->addKeyframe(fid, pose, cloud);
        });

    pose_graph_->setOptimizedPoseCallback(
        [this](FrameId fid, const SE3Pose& pose) {
          latest_optimized_pose_ = pose;
        });

    // --- ROS 2 subscriptions ---
    lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/LIDAR/pointcloud", rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          LiDARPacket pkt;
          pkt.stamp = std::chrono::steady_clock::now();
          pcl::fromROSMsg(*msg, *pkt.cloud);
          pkt.cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>(
              *pkt.cloud);  // ensure shared ownership
          lio_odom_->addPointCloud(pkt);
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
          lio_odom_->addImu(imu);
        });

    // Publishers
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/TRACK_PATH", 10);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    // Optimization timer (10Hz)
    opt_timer_ = create_wall_timer(
        std::chrono::milliseconds(100),
        [this]() { pose_graph_->optimize(); });

    RCLCPP_INFO(get_logger(), "SlamNode: configured OK");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State&) override {
    RCLCPP_INFO(get_logger(), "SlamNode: activating...");

    // Start LIO thread
    lio_thread_ = std::make_unique<std::thread>([this]() {
      lio_odom_->run();
    });

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override {
    RCLCPP_INFO(get_logger(), "SlamNode: deactivating...");
    lio_odom_->stop();
    if (lio_thread_ && lio_thread_->joinable()) {
      lio_thread_->join();
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override {
    RCLCPP_INFO(get_logger(), "SlamNode: cleaning up...");
    lidar_sub_.reset();
    imu_sub_.reset();
    map_pub_.reset();
    path_pub_.reset();
    lio_odom_.reset();
    pose_graph_.reset();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State&) override {
    RCLCPP_INFO(get_logger(), "SlamNode: shutting down...");
    return CallbackReturn::SUCCESS;
  }

private:
  void loadParams() {
    sensor_params_.lidar_scan_lines = get_parameter("sensors.lidar_scan_lines").as_int();
    sensor_params_.lidar_min_range  = get_parameter("sensors.lidar_min_range").as_double();
    sensor_params_.lidar_max_range  = get_parameter("sensors.lidar_max_range").as_double();
    sensor_params_.lidar_hz         = get_parameter("sensors.lidar_hz").as_double();
    sensor_params_.imu_hz           = get_parameter("sensors.imu_hz").as_double();
    sensor_params_.odom_hz          = get_parameter("sensors.odom_hz").as_double();

    lio_params_.voxel_size            = get_parameter("lio.voxel_size").as_double();
    lio_params_.max_voxels            = get_parameter("lio.max_voxels").as_int();
    lio_params_.keyframe_distance     = get_parameter("lio.keyframe_distance").as_double();
    lio_params_.keyframe_angle        = get_parameter("lio.keyframe_angle").as_double();
    lio_params_.max_iterations        = get_parameter("lio.max_iterations").as_int();
    lio_params_.correspondence_radius = get_parameter("lio.correspondence_radius").as_double();
    lio_params_.num_threads           = get_parameter("lio.num_threads").as_int();

    backend_params_.degeneracy_threshold = get_parameter("backend.degeneracy_threshold").as_double();
  }

  void publishTF(const SE3Pose& pose, FrameId fid) {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now();
    tf.header.frame_id = "map";
    tf.child_frame_id  = "odom";
    tf.transform.translation.x = pose.t.x();
    tf.transform.translation.y = pose.t.y();
    tf.transform.translation.z = pose.t.z();
    tf.transform.rotation.w = pose.q.w();
    tf.transform.rotation.x = pose.q.x();
    tf.transform.rotation.y = pose.q.y();
    tf.transform.rotation.z = pose.q.z();
    tf_broadcaster_->sendTransform(tf);
  }

  SE3Pose getPose(FrameId fid) const {
    return pose_graph_ ? pose_graph_->getPose(fid) : SE3Pose::Identity();
  }

  // Params
  SensorParams    sensor_params_;
  LIOParams       lio_params_;
  BackendParams   backend_params_;

  // Components
  std::shared_ptr<lio::LIOOdometry>                lio_odom_;
  std::shared_ptr<backend::PoseGraphOptimizer>     pose_graph_;
  std::unique_ptr<std::thread>                     lio_thread_;

  // ROS 2
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr         imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr     map_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr              path_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster>                 tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr                                   opt_timer_;

  SE3Pose latest_optimized_pose_;
};

}  // namespace m20::nodes

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(m20::nodes::SlamNode)