/**
 * @file localization_node.cpp
 * @brief M20Pro prior-map localization using the deployed topic/parameter contract.
 *
 * Native inputs:  /LIDAR/POINTS, /IMU, /GPYBM, /leg_odom
 * Native outputs: /ODOM, /RTK_RAW_ODOM, /FULL_CLOUD_MAP, /LOC_BODY_POINTS
 */

#include "m20_slam_navigation/common/params.hpp"
#include "m20_slam_navigation/common/math_utils.hpp"
#include "m20_slam_navigation/common/types.hpp"
#include "m20_slam_navigation/localization/relocalizer.hpp"

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <nav_msgs/msg/odometry.hpp>
#if __has_include(<rclcpp/generic_subscription.hpp>)
#include <rclcpp/generic_subscription.hpp>
#define M20_HAS_RCLCPP_GENERIC_SUBSCRIPTION 1
#else
#define M20_HAS_RCLCPP_GENERIC_SUBSCRIPTION 0
#endif
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

namespace m20::nodes {

namespace {

Timestamp sensorTimestamp(const builtin_interfaces::msg::Time& stamp) {
  return Timestamp(std::chrono::nanoseconds(rclcpp::Time(stamp).nanoseconds()));
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

}  // namespace

class LocalizationNode : public rclcpp_lifecycle::LifecycleNode {
public:
  using CallbackReturn =
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit LocalizationNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : rclcpp_lifecycle::LifecycleNode("localization_node", options) {
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
    RCLCPP_INFO(get_logger(), "LocalizationNode: configuring native M20Pro contract");
    declareParameters();
    loadParameters();

    relocalizer_ = std::make_shared<localization::Relocalizer>(
        loc_params_, sensor_params_);
    if (!relocalizer_->loadMap(loc_params_.map_path)) {
      RCLCPP_ERROR(get_logger(), "Failed to load native localization map: %s",
                   loc_params_.map_path.c_str());
      return CallbackReturn::FAILURE;
    }

    createPublishers();
    createSubscriptions();
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    publishGlobalMap();

    output_timer_ = create_wall_timer(
        std::chrono::milliseconds(20), [this]() { publishLocalization(); });

    RCLCPP_INFO(
        get_logger(),
        "Native localization: lidar=%s imu=%s leg_odom=%s odom_out=%s world=%s map=%s",
        runtime_params_.topics.input_lidar.c_str(),
        runtime_params_.topics.input_imu.c_str(),
        runtime_params_.topics.leg_odom.c_str(),
        runtime_params_.topics.output_odom.c_str(),
        runtime_params_.world_frame.c_str(), loc_params_.map_path.c_str());
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State&) override {
    RCLCPP_INFO(get_logger(), "LocalizationNode: active");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override {
    RCLCPP_INFO(get_logger(), "LocalizationNode: inactive");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override {
    output_timer_.reset();
    lidar_sub_.reset();
    imu_sub_.reset();
    leg_odom_sub_.reset();
    initial_pose_sub_.reset();
#if M20_HAS_RCLCPP_GENERIC_SUBSCRIPTION
    rtk_sub_.reset();
#endif
    odom_pub_.reset();
    global_map_pub_.reset();
    body_cloud_pub_.reset();
    tf_broadcaster_.reset();
    relocalizer_.reset();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State&) override {
    return CallbackReturn::SUCCESS;
  }

private:
  void declareParameters() {
    declare_parameter<int>("common.mode", 0);
    declare_parameter<int>("common.max_thread_num", 8);
    declare_parameter<std::vector<double>>(
        "common.extrinsic_I_B",
        {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
         0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0});
    declare_parameter<std::vector<double>>(
        "common.init_pose", {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0});
    declare_parameter<int>("common.frame_queue_keep_length", 3);

    declare_parameter<std::string>("node.input_lidar_topic", "/LIDAR/POINTS");
    declare_parameter<std::string>("node.input_imu_topic", "/IMU");
    declare_parameter<std::string>("node.input_rtk_topic", "/GPYBM");
    declare_parameter<std::string>("node.leg_odom_topic", "/leg_odom");
    declare_parameter<std::string>("node.output_odom_topic", "/ODOM");
    declare_parameter<std::string>("node.output_enu_topic", "/RTK_RAW_ODOM");
    declare_parameter<std::string>("node.output_global_map_topic", "/FULL_CLOUD_MAP");
    declare_parameter<std::string>("node.output_body_cloud_topic", "/LOC_BODY_POINTS");
    declare_parameter<std::string>("node.initial_pose_topic", "/initialpose");
    declare_parameter<double>("node.global_map_leaf_size_viz", 0.5);
    declare_parameter<int>("node.lidar_type", 1);
    declare_parameter<bool>("node.lidar_use_system_time", false);
    declare_parameter<bool>("node.imu_use_system_time", false);
    declare_parameter<bool>("node.body_cloud_box_filter", true);
    declare_parameter<std::vector<double>>(
        "node.body_cloud_box_min", {-5.0, -5.0, -1.5});
    declare_parameter<std::vector<double>>(
        "node.body_cloud_box_max", {5.0, 5.0, 1.5});
    declare_parameter<bool>("node.body_cloud_voxel_filter", false);
    declare_parameter<double>("node.body_cloud_voxel_leaf_size", 0.03);

    declare_parameter<int>("cloud_preprocess.skip_num", 1);
    declare_parameter<double>("cloud_preprocess.leaf_size", 0.3);
    declare_parameter<std::vector<double>>(
        "cloud_preprocess.extrinsic_I_L",
        {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
         0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0});

    declare_parameter<double>("optimization.smoother_lag", 3.0);
    declare_parameter<int>("optimization.update_count", 1);
    declare_parameter<double>("optimization.acc_cov", 0.1);
    declare_parameter<double>("optimization.gyr_cov", 0.1);
    declare_parameter<double>("optimization.bias_acc_cov", 0.01);
    declare_parameter<double>("optimization.bias_gyr_cov", 0.01);
    declare_parameter<double>("optimization.integration_cov", 0.01);
    declare_parameter<double>("optimization.ght_vio_cov_scale", 1.0);
    declare_parameter<double>("optimization.imu_gravity", 9.80511);
    declare_parameter<std::vector<double>>("optimization.max_edge", {0.6, 0.6, 0.5});
    declare_parameter<std::vector<double>>("optimization.min_edge", {-0.6, -0.6, -0.5});
    declare_parameter<std::vector<double>>("optimization.vel_lim", {5.0, 5.0, 10.0});
    declare_parameter<std::vector<double>>(
        "optimization.default_prior_pose_cov",
        {1e-8, 1e-8, 1e-8, 1e-6, 1e-6, 1e-6});
    declare_parameter<std::vector<double>>(
        "optimization.default_prior_vel_cov", {1e8, 1e8, 1e8});
    declare_parameter<std::vector<double>>(
        "optimization.default_prior_bias_cov",
        {1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4});
    declare_parameter<std::vector<double>>(
        "optimization.fixed_global_registration_cov",
        {1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4});
    declare_parameter<std::vector<double>>(
        "optimization.leg_odom_cov", {0.005, 0.005, 0.005, 0.005, 0.005, 0.005});
    declare_parameter<std::vector<double>>(
        "optimization.grav_cov", {1e5, 1e5, 1e15});

    declare_parameter<std::string>(
        "static_map_registration.static_map_path",
        "/var/opt/robot/data/maps/active/");
    declare_parameter<std::string>(
        "static_map_registration.pcd_file_name", "full_cloud.pcd");
    declare_parameter<int>(
        "static_map_registration.registration.num_threads", 4);
    declare_parameter<int>(
        "static_map_registration.registration.max_iterations", 20);
    declare_parameter<double>(
        "static_map_registration.registration.convergence_thresh_rot", 1e-4);
    declare_parameter<double>(
        "static_map_registration.registration.convergence_thresh_trans", 1e-4);
    declare_parameter<double>(
        "static_map_registration.valid_global_registration_ratio_thresh", 0.3);

    declare_parameter<int>("monitor.state_win_size", 10);
    declare_parameter<int>("monitor.error_win_size", 10);
    declare_parameter<double>("monitor.vel_check_duration", 1.0);
    declare_parameter<double>("monitor.vel_linear_limit", 5.0);
    declare_parameter<double>("monitor.vel_angular_limit", 6.0);
    declare_parameter<double>("monitor.vel_error_frac_th", 0.8);
    declare_parameter<double>("monitor.predict_error_check_duration", 1.0);
    declare_parameter<double>("monitor.error_linear_limit", 0.5);
    declare_parameter<double>("monitor.error_angular_limit", 0.5);
    declare_parameter<double>("monitor.predict_error_frac_th", 0.8);
    declare_parameter<int>("monitor.matching_error_win_size", 20);
    declare_parameter<double>("monitor.matching_error_check_duration", 2.0);
    declare_parameter<double>("monitor.matching_error_th", 0.25);
    declare_parameter<double>("monitor.inlier_ratio_th", 0.85);
    declare_parameter<double>("monitor.matching_error_frac_th", 0.5);
    declare_parameter<double>("monitor.odom_update_time_limit", 1.0);
    declare_parameter<double>("monitor.found_judge_time_limit", 3.0);
    declare_parameter<bool>("monitor.auto_reloc", false);
    declare_parameter<double>("monitor.reloc_loss_time", 10.0);
    declare_parameter<double>("monitor.reloc_loss_again_time", 10.0);

    declare_parameter<std::vector<double>>(
        "ght.extrinsic",
        {-1.0, 0.0, 0.0, 0.16, 0.0, 0.0, -1.0, 0.0,
         0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0});
    declare_parameter<std::vector<double>>(
        "ght.vio_default_cov", {0.001, 0.001, 0.001, 0.001, 0.001, 0.001});
    declare_parameter<std::vector<double>>(
        "ght.viro_default_cov", {0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001});
    declare_parameter<std::vector<double>>(
        "ght.rtk_default_cov", {0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001});
    declare_parameter<std::string>("ght.path", "/var/opt/robot/data/maps/active/");

    declare_parameter<std::string>("frames.world", "camera_init");
    declare_parameter<std::string>("frames.map", "map");
    declare_parameter<std::string>("frames.odom", "odom");
    declare_parameter<std::string>("frames.body", "base_link");
    declare_parameter<std::string>("frames.lidar", "lidar_link");
    declare_parameter<std::string>("frames.imu", "imu_link");
    declare_parameter<bool>("frames.publish_world_to_odom_tf", true);

    declare_parameter<std::string>("adapter.map_path", "");
    declare_parameter<double>("adapter.ndt_resolution", 1.0);
    declare_parameter<double>("adapter.ndt_step_size", 0.1);
    declare_parameter<double>("adapter.ndt_outlier_ratio", 0.55);
    declare_parameter<int>("adapter.num_hypotheses", 8);
    declare_parameter<double>("adapter.hypothesis_trans_range", 5.0);
    declare_parameter<double>("adapter.hypothesis_rot_range", 3.141592653589793);
    declare_parameter<std::string>("adapter.rtk_ros_type", "");
  }

  void loadParameters() {
    auto& topics = runtime_params_.topics;
    topics.input_lidar = get_parameter("node.input_lidar_topic").as_string();
    topics.input_imu = get_parameter("node.input_imu_topic").as_string();
    topics.input_rtk = get_parameter("node.input_rtk_topic").as_string();
    topics.leg_odom = get_parameter("node.leg_odom_topic").as_string();
    topics.output_odom = get_parameter("node.output_odom_topic").as_string();
    topics.output_enu = get_parameter("node.output_enu_topic").as_string();
    topics.output_global_map =
        get_parameter("node.output_global_map_topic").as_string();
    topics.output_body_cloud =
        get_parameter("node.output_body_cloud_topic").as_string();
    topics.initial_pose = get_parameter("node.initial_pose_topic").as_string();

    runtime_params_.world_frame = get_parameter("frames.world").as_string();
    runtime_params_.map_frame = get_parameter("frames.map").as_string();
    runtime_params_.odom_frame = get_parameter("frames.odom").as_string();
    runtime_params_.body_frame = get_parameter("frames.body").as_string();
    runtime_params_.lidar_frame = get_parameter("frames.lidar").as_string();
    runtime_params_.imu_frame = get_parameter("frames.imu").as_string();
    runtime_params_.publish_world_to_odom_tf =
        get_parameter("frames.publish_world_to_odom_tf").as_bool();

    const auto static_map_path =
        get_parameter("static_map_registration.static_map_path").as_string();
    const auto pcd_file_name =
        get_parameter("static_map_registration.pcd_file_name").as_string();
    const auto map_override = get_parameter("adapter.map_path").as_string();
    loc_params_.map_path = map_override.empty()
        ? (std::filesystem::path(static_map_path) / pcd_file_name).string()
        : map_override;
    loc_params_.map_voxel_leaf_size = get_parameter("cloud_preprocess.leaf_size").as_double();
    loc_params_.ndt_resolution = get_parameter("adapter.ndt_resolution").as_double();
    loc_params_.ndt_max_iterations = static_cast<int>(
        get_parameter("static_map_registration.registration.max_iterations").as_int());
    loc_params_.ndt_step_size = get_parameter("adapter.ndt_step_size").as_double();
    loc_params_.ndt_epsilon = std::max(
        get_parameter("static_map_registration.registration.convergence_thresh_rot").as_double(),
        get_parameter("static_map_registration.registration.convergence_thresh_trans").as_double());
    loc_params_.ndt_outlier_ratio = get_parameter("adapter.ndt_outlier_ratio").as_double();
    loc_params_.num_hypotheses = static_cast<int>(
        get_parameter("adapter.num_hypotheses").as_int());
    loc_params_.hypothesis_trans_range =
        get_parameter("adapter.hypothesis_trans_range").as_double();
    loc_params_.hypothesis_rot_range =
        get_parameter("adapter.hypothesis_rot_range").as_double();
    loc_params_.eskf_accel_noise = get_parameter("optimization.acc_cov").as_double();
    loc_params_.eskf_gyro_noise = get_parameter("optimization.gyr_cov").as_double();
    loc_params_.eskf_accel_bias_noise =
        get_parameter("optimization.bias_acc_cov").as_double();
    loc_params_.eskf_gyro_bias_noise =
        get_parameter("optimization.bias_gyr_cov").as_double();
    loc_params_.imu_gravity = get_parameter("optimization.imu_gravity").as_double();

    if (loc_params_.ndt_resolution <= 0.0 || loc_params_.ndt_max_iterations <= 0 ||
        loc_params_.ndt_step_size <= 0.0 || loc_params_.ndt_epsilon <= 0.0 ||
        loc_params_.ndt_outlier_ratio < 0.0 || loc_params_.ndt_outlier_ratio > 1.0 ||
        loc_params_.num_hypotheses <= 0 || loc_params_.hypothesis_trans_range < 0.0 ||
        loc_params_.hypothesis_rot_range < 0.0 ||
        loc_params_.hypothesis_rot_range > math::kPI || loc_params_.imu_gravity <= 0.0) {
      throw std::runtime_error("invalid localization NDT, hypothesis, or gravity parameter");
    }

    sensor_params_.imu_hz = 200.0;
    sensor_params_.odom_hz = 100.0;
    cloud_skip_num_ = static_cast<int>(std::max<int64_t>(
        1, get_parameter("cloud_preprocess.skip_num").as_int()));
    cloud_leaf_size_ = get_parameter("cloud_preprocess.leaf_size").as_double();
    map_visual_leaf_size_ = get_parameter("node.global_map_leaf_size_viz").as_double();
    body_box_filter_ = get_parameter("node.body_cloud_box_filter").as_bool();
    body_voxel_filter_ = get_parameter("node.body_cloud_voxel_filter").as_bool();
    body_voxel_leaf_size_ = get_parameter("node.body_cloud_voxel_leaf_size").as_double();
    rtk_ros_type_ = get_parameter("adapter.rtk_ros_type").as_string();

    const auto body_min = get_parameter("node.body_cloud_box_min").as_double_array();
    const auto body_max = get_parameter("node.body_cloud_box_max").as_double_array();
    if (body_min.size() != 3U || body_max.size() != 3U) {
      throw std::runtime_error("node.body_cloud_box_min/max must contain three values");
    }
    for (std::size_t i = 0; i < 3U; ++i) {
      body_box_min_[i] = body_min[i];
      body_box_max_[i] = body_max[i];
      if (body_box_min_[i] >= body_box_max_[i]) {
        throw std::runtime_error("node.body_cloud_box_min must be below box_max");
      }
    }

    const auto leg_cov = get_parameter("optimization.leg_odom_cov").as_double_array();
    if (leg_cov.size() != 6U) {
      throw std::runtime_error("optimization.leg_odom_cov must contain six values");
    }
    for (std::size_t i = 0; i < 6U; ++i) leg_odom_cov_[i] = leg_cov[i];

    const auto initial = get_parameter("common.init_pose").as_double_array();
    if (initial.size() != 7U) {
      throw std::runtime_error("common.init_pose must contain x,y,z,qw,qx,qy,qz");
    }
    initial_guess_.pose.t = {initial[0], initial[1], initial[2]};
    initial_guess_.pose.q = normalizedQuaternion(
        initial[3], initial[4], initial[5], initial[6]);
  }

  void createPublishers() {
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
        runtime_params_.topics.output_odom,
        rclcpp::QoS(rclcpp::KeepLast(20)).reliable().durability_volatile());
    global_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        runtime_params_.topics.output_global_map,
        rclcpp::QoS(1).transient_local().reliable());
    body_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        runtime_params_.topics.output_body_cloud,
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile());
  }

  void createSubscriptions() {
    lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        runtime_params_.topics.input_lidar,
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile(),
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr message) {
          onLidar(*message);
        });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        runtime_params_.topics.input_imu,
        rclcpp::QoS(rclcpp::KeepLast(512)).reliable().durability_volatile(),
        [this](const sensor_msgs::msg::Imu::SharedPtr message) {
          ImuPacket imu;
          imu.stamp = sensorTimestamp(message->header.stamp);
          imu.accel = {message->linear_acceleration.x,
                       message->linear_acceleration.y,
                       message->linear_acceleration.z};
          imu.gyro = {message->angular_velocity.x,
                      message->angular_velocity.y,
                      message->angular_velocity.z};
          relocalizer_->predict(imu);
        });
    leg_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        runtime_params_.topics.leg_odom,
        rclcpp::QoS(rclcpp::KeepLast(100)).reliable().durability_volatile(),
        [this](const nav_msgs::msg::Odometry::SharedPtr message) {
          onLegOdometry(*message);
        });
    initial_pose_sub_ =
        create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            runtime_params_.topics.initial_pose, rclcpp::QoS(rclcpp::KeepLast(10)),
            [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message) {
              onInitialPose(*message);
            });

#if M20_HAS_RCLCPP_GENERIC_SUBSCRIPTION
    if (!rtk_ros_type_.empty()) {
      rtk_sub_ = create_generic_subscription(
          runtime_params_.topics.input_rtk, rtk_ros_type_,
          rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile(),
          [this](std::shared_ptr<rclcpp::SerializedMessage>) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "Received /GPYBM fallback payload, but native RTK fusion requires the DrDDS SDK codec");
          });
    } else {
      RCLCPP_WARN(
          get_logger(),
          "Native RTK topic %s is configured but not subscribed: generated DrDDS type unavailable",
          runtime_params_.topics.input_rtk.c_str());
    }
#else
    RCLCPP_WARN(
        get_logger(),
        "Native RTK topic %s is not subscribed: this ROS 2 release has no generic subscription API",
        runtime_params_.topics.input_rtk.c_str());
#endif
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr preprocessCloud(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& input) const {
    pcl::PointCloud<pcl::PointXYZ>::Ptr skipped(
        new pcl::PointCloud<pcl::PointXYZ>());
    skipped->reserve((input->size() + cloud_skip_num_ - 1U) / cloud_skip_num_);
    for (std::size_t i = 0; i < input->size(); i += cloud_skip_num_) {
      const auto& point = input->points[i];
      if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
        skipped->push_back(point);
      }
    }
    if (cloud_leaf_size_ <= 0.0 || skipped->empty()) return skipped;
    pcl::PointCloud<pcl::PointXYZ>::Ptr output(
        new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(skipped);
    const float leaf = static_cast<float>(cloud_leaf_size_);
    voxel.setLeafSize(leaf, leaf, leaf);
    voxel.filter(*output);
    return output;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr bodyCloud(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& input) const {
    auto bounded = input;
    if (body_box_filter_) {
      bounded.reset(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::CropBox<pcl::PointXYZ> crop;
      crop.setInputCloud(input);
      crop.setMin(Eigen::Vector4f(
          static_cast<float>(body_box_min_[0]), static_cast<float>(body_box_min_[1]),
          static_cast<float>(body_box_min_[2]), 1.0f));
      crop.setMax(Eigen::Vector4f(
          static_cast<float>(body_box_max_[0]), static_cast<float>(body_box_max_[1]),
          static_cast<float>(body_box_max_[2]), 1.0f));
      crop.filter(*bounded);
    }
    if (!body_voxel_filter_ || body_voxel_leaf_size_ <= 0.0 || bounded->empty()) {
      return bounded;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr output(
        new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(bounded);
    const float leaf = static_cast<float>(body_voxel_leaf_size_);
    voxel.setLeafSize(leaf, leaf, leaf);
    voxel.filter(*output);
    return output;
  }

  void onLidar(const sensor_msgs::msg::PointCloud2& message) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr input(
        new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(message, *input);
    if (input->empty()) return;

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      latest_sensor_stamp_ = message.header.stamp;
      have_sensor_stamp_ = true;
    }

    const auto body = bodyCloud(input);
    if (body_cloud_pub_) {
      sensor_msgs::msg::PointCloud2 output;
      pcl::toROSMsg(*body, output);
      output.header = message.header;
      output.header.frame_id = runtime_params_.body_frame;
      body_cloud_pub_->publish(output);
    }

    const auto registration_cloud = preprocessCloud(input);
    if (registration_cloud->empty()) return;
    if (!relocalizer_->isLocalized()) {
      PoseWithCovariance guess;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        guess = initial_guess_;
      }
      (void)relocalizer_->relocalize(registration_cloud, guess);
    } else {
      relocalizer_->updateNDT(registration_cloud);
    }
  }

  void onLegOdometry(const nav_msgs::msg::Odometry& message) {
    FootOdomPacket odom;
    odom.stamp = sensorTimestamp(message.header.stamp);
    odom.pose.t = {message.pose.pose.position.x,
                   message.pose.pose.position.y,
                   message.pose.pose.position.z};
    odom.pose.q = normalizedQuaternion(
        message.pose.pose.orientation.w, message.pose.pose.orientation.x,
        message.pose.pose.orientation.y, message.pose.pose.orientation.z);
    odom.twist.linear = {message.twist.twist.linear.x,
                         message.twist.twist.linear.y,
                         message.twist.twist.linear.z};
    odom.twist.angular = {message.twist.twist.angular.x,
                          message.twist.twist.angular.y,
                          message.twist.twist.angular.z};
    odom.covariance.setZero();
    bool message_has_covariance = false;
    for (Eigen::Index row = 0; row < 6; ++row) {
      for (Eigen::Index col = 0; col < 6; ++col) {
        const auto index = static_cast<std::size_t>(row * 6 + col);
        const double value = message.pose.covariance[index];
        odom.covariance(row, col) = value;
        message_has_covariance = message_has_covariance || std::abs(value) > 0.0;
      }
    }
    if (!message_has_covariance) {
      for (Eigen::Index i = 0; i < 6; ++i) {
        odom.covariance(i, i) = leg_odom_cov_[static_cast<std::size_t>(i)];
      }
    }
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      latest_leg_pose_ = odom.pose;
      latest_leg_twist_ = odom.twist;
      have_leg_odom_ = true;
    }
    relocalizer_->updateOdometry(odom);
  }

  void onInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped& message) {
    PoseWithCovariance initial;
    initial.pose.t = {message.pose.pose.position.x,
                      message.pose.pose.position.y,
                      message.pose.pose.position.z};
    initial.pose.q = normalizedQuaternion(
        message.pose.pose.orientation.w, message.pose.pose.orientation.x,
        message.pose.pose.orientation.y, message.pose.pose.orientation.z);
    for (Eigen::Index row = 0; row < 6; ++row) {
      for (Eigen::Index col = 0; col < 6; ++col) {
        const auto index = static_cast<std::size_t>(row * 6 + col);
        initial.covariance(row, col) = message.pose.covariance[index];
      }
    }
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      initial_guess_ = initial;
    }
    if (relocalizer_) relocalizer_->reset();
    RCLCPP_INFO(get_logger(), "Updated native localization initial pose");
  }

  void publishGlobalMap() {
    const auto map = relocalizer_->getGlobalMap();
    if (!map || map->empty() || !global_map_pub_) return;
    auto visual_map = map;
    if (map_visual_leaf_size_ > 0.0) {
      visual_map.reset(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::VoxelGrid<pcl::PointXYZ> voxel;
      voxel.setInputCloud(map);
      const float leaf = static_cast<float>(map_visual_leaf_size_);
      voxel.setLeafSize(leaf, leaf, leaf);
      voxel.filter(*visual_map);
    }
    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(*visual_map, message);
    message.header.stamp = now();
    message.header.frame_id = runtime_params_.world_frame;
    global_map_pub_->publish(message);
  }

  void publishLocalization() {
    if (!relocalizer_ || !relocalizer_->isLocalized()) return;
    const auto current = relocalizer_->getCurrentPose();
    SE3Pose leg_pose;
    Twist leg_twist;
    bool have_leg = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      leg_pose = latest_leg_pose_;
      leg_twist = latest_leg_twist_;
      have_leg = have_leg_odom_;
    }

    nav_msgs::msg::Odometry odometry;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (have_sensor_stamp_) {
        odometry.header.stamp = latest_sensor_stamp_;
      } else {
        odometry.header.stamp = now();
      }
    }
    odometry.header.frame_id = runtime_params_.world_frame;
    odometry.child_frame_id = runtime_params_.body_frame;
    odometry.pose.pose.position.x = current.pose.t.x();
    odometry.pose.pose.position.y = current.pose.t.y();
    odometry.pose.pose.position.z = current.pose.t.z();
    odometry.pose.pose.orientation.w = current.pose.q.w();
    odometry.pose.pose.orientation.x = current.pose.q.x();
    odometry.pose.pose.orientation.y = current.pose.q.y();
    odometry.pose.pose.orientation.z = current.pose.q.z();
    odometry.twist.twist.linear.x = leg_twist.linear.x();
    odometry.twist.twist.linear.y = leg_twist.linear.y();
    odometry.twist.twist.linear.z = leg_twist.linear.z();
    odometry.twist.twist.angular.x = leg_twist.angular.x();
    odometry.twist.twist.angular.y = leg_twist.angular.y();
    odometry.twist.twist.angular.z = leg_twist.angular.z();
    for (Eigen::Index row = 0; row < 6; ++row) {
      for (Eigen::Index col = 0; col < 6; ++col) {
        const auto index = static_cast<std::size_t>(row * 6 + col);
        odometry.pose.covariance[index] = current.covariance(row, col);
      }
    }
    odom_pub_->publish(odometry);

    if (runtime_params_.publish_world_to_odom_tf && tf_broadcaster_) {
      const SE3Pose world_odom = have_leg
          ? current.pose * leg_pose.inverse()
          : current.pose;
      geometry_msgs::msg::TransformStamped transform;
      transform.header = odometry.header;
      transform.child_frame_id = runtime_params_.odom_frame;
      transform.transform.translation.x = world_odom.t.x();
      transform.transform.translation.y = world_odom.t.y();
      transform.transform.translation.z = world_odom.t.z();
      transform.transform.rotation.w = world_odom.q.w();
      transform.transform.rotation.x = world_odom.q.x();
      transform.transform.rotation.y = world_odom.q.y();
      transform.transform.rotation.z = world_odom.q.z();
      tf_broadcaster_->sendTransform(transform);
    }
  }

  LocalizationParams loc_params_;
  LocalizationRuntimeParams runtime_params_;
  SensorParams sensor_params_;
  std::shared_ptr<localization::Relocalizer> relocalizer_;

  int cloud_skip_num_{1};
  double cloud_leaf_size_{0.3};
  double map_visual_leaf_size_{0.5};
  bool body_box_filter_{true};
  bool body_voxel_filter_{false};
  double body_voxel_leaf_size_{0.03};
  std::array<double, 3> body_box_min_{{-5.0, -5.0, -1.5}};
  std::array<double, 3> body_box_max_{{5.0, 5.0, 1.5}};
  std::array<double, 6> leg_odom_cov_{{0.005, 0.005, 0.005, 0.005, 0.005, 0.005}};
  std::string rtk_ros_type_;

  mutable std::mutex state_mutex_;
  PoseWithCovariance initial_guess_;
  SE3Pose latest_leg_pose_;
  Twist latest_leg_twist_;
  bool have_leg_odom_{false};
  builtin_interfaces::msg::Time latest_sensor_stamp_;
  bool have_sensor_stamp_{false};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr leg_odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      initial_pose_sub_;
#if M20_HAS_RCLCPP_GENERIC_SUBSCRIPTION
  rclcpp::GenericSubscription::SharedPtr rtk_sub_;
#endif
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr body_cloud_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr autostart_timer_;
  rclcpp::TimerBase::SharedPtr output_timer_;
};

}  // namespace m20::nodes

RCLCPP_COMPONENTS_REGISTER_NODE(m20::nodes::LocalizationNode)
