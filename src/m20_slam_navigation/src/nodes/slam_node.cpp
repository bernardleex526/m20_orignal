/**
 * @file slam_node.cpp
 * @brief M20 Pro mapping node: official RoboSense cloud + IMU -> isolated 3D map.
 */

#include "m20_slam_navigation/backend/pose_graph_optimizer.hpp"
#include "m20_slam_navigation/common/params.hpp"
#include "m20_slam_navigation/common/types.hpp"
#include "m20_slam_navigation/lio/lio_odometry.hpp"
#include "m20_slam_navigation/ros/m20_cloud_adapter.hpp"
#include "m20_slam_navigation/ros/socket_imu_source.hpp"
#include "m20_slam_navigation/ros/socket_pointcloud_source.hpp"
#include "m20_slam_navigation/ros/vendor_output_contract.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>

#include <chrono>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <unordered_map>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace m20::nodes
{

class SlamNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit SlamNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp_lifecycle::LifecycleNode("slam_node", options)
  {
    autostart_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
        if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
          trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
        }
        if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
          trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
        }
        if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
          autostart_timer_.reset();
        }
      });
  }

  ~SlamNode() override
  {
    if (auto_save_on_shutdown_ && lio_odom_) {
      std::string message;
      if (!saveMap(message)) {
        RCLCPP_WARN(get_logger(), "Shutdown map save skipped: %s", message.c_str());
      }
    }
    stopWorker();
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    declareParameters();
    loadParameters();

    lio_odom_ = std::make_shared<lio::LIOOdometry>(
      sensor_params_, lio_params_, backend_params_);
    pose_graph_ = std::make_shared<backend::PoseGraphOptimizer>(backend_params_);

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 20);
    path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic_, 10);
    // The vendor Foxy graph advertises these channels as RELIABLE/volatile.
    // SensorDataQoS is BEST_EFFORT and silently prevents matching that graph.
    const auto vendor_cloud_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
    aligned_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      aligned_cloud_topic_, vendor_cloud_qos);
    body_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      body_cloud_topic_, vendor_cloud_qos);
    voxel_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      voxel_cloud_topic_, rclcpp::QoS(1).transient_local().reliable());
    map_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      map_cloud_topic_, rclcpp::QoS(1).transient_local().reliable());
    depth_image_pub_ = create_publisher<sensor_msgs::msg::Image>(
      depth_image_topic_, rclcpp::QoS(5).reliable());
    height_image_pub_ = create_publisher<sensor_msgs::msg::Image>(
      height_image_topic_, rclcpp::QoS(5).reliable());
    height_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      height_cloud_topic_, rclcpp::QoS(5).reliable());
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    configureAlgorithmCallbacks();
    configureSubscriptions();

    save_map_service_ = create_service<std_srvs::srv::Trigger>(
      "/m20_slam/save_map",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        response->success = saveMap(response->message);
      });

    opt_timer_ = create_wall_timer(
      std::chrono::milliseconds(200), [this]() {
        if (pose_graph_) {
          pose_graph_->optimize();
        }
      });
    input_status_timer_ = create_wall_timer(
      std::chrono::seconds(2), [this]() {
        RCLCPP_INFO(
          get_logger(),
          "M20 input: Cloud=%llu IMU=%llu processed=%llu init_wait=%llu bootstrap=%llu "
          "updated=%llu rejected=%llu dropped_cloud=%llu dropped_imu=%llu keyframes=%llu "
          "lidar_queue=%zu stamp_delta=%.3f ms",
          static_cast<unsigned long long>(accepted_clouds_.load()),
          static_cast<unsigned long long>(accepted_imus_.load()),
          static_cast<unsigned long long>(lio_odom_ ? lio_odom_->processedScans() : 0U),
          static_cast<unsigned long long>(lio_odom_ ? lio_odom_->initializationWaitScans() : 0U),
          static_cast<unsigned long long>(lio_odom_ ? lio_odom_->bootstrapScans() : 0U),
          static_cast<unsigned long long>(lio_odom_ ? lio_odom_->successfulUpdates() : 0U),
          static_cast<unsigned long long>(lio_odom_ ? lio_odom_->rejectedUpdates() : 0U),
          static_cast<unsigned long long>(lio_odom_ ? lio_odom_->droppedScans() : 0U),
          static_cast<unsigned long long>(dropped_imus_.load()),
          static_cast<unsigned long long>(keyframe_count_.load()),
          lio_odom_ ? lio_odom_->lidarQueueSize() : 0U,
          static_cast<double>(last_lidar_scan_end_ns_.load() - last_imu_stamp_ns_.load()) * 1e-6);
      });
    if (checkpoint_save_period_s_ > 0.0) {
      checkpoint_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(checkpoint_save_period_s_)),
        [this]() {
          std::string message;
          if (!saveMap(message)) {
            RCLCPP_WARN(get_logger(), "Checkpoint map save skipped: %s", message.c_str());
          }
        });
    }

    RCLCPP_INFO(
      get_logger(),
      "Configured M20 mapping: lidar=%s imu=%s output=%s frame=%s",
      lidar_topic_.c_str(), imu_topic_.c_str(), map_cloud_topic_.c_str(), map_frame_.c_str());
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    if (!lio_odom_ || lio_thread_) {
      return CallbackReturn::FAILURE;
    }
    odom_pub_->on_activate();
    path_pub_->on_activate();
    aligned_cloud_pub_->on_activate();
    body_cloud_pub_->on_activate();
    voxel_cloud_pub_->on_activate();
    map_cloud_pub_->on_activate();
    lio_thread_ = std::make_unique<std::thread>([this]() {lio_odom_->run();});
    RCLCPP_INFO(get_logger(), "M20 mapping is active");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    std::string message;
    if (auto_save_on_shutdown_ && !saveMap(message)) {
      RCLCPP_ERROR(get_logger(), "Automatic map save failed: %s", message.c_str());
    }
    stopWorker();
    odom_pub_->on_deactivate();
    path_pub_->on_deactivate();
    aligned_cloud_pub_->on_deactivate();
    body_cloud_pub_->on_deactivate();
    voxel_cloud_pub_->on_deactivate();
    map_cloud_pub_->on_deactivate();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    stopWorker();
    drdds_lidar_source_.reset();
    drdds_imu_source_.reset();
    lidar_sub_.reset();
    imu_sub_.reset();
    odom_pub_.reset();
    path_pub_.reset();
    aligned_cloud_pub_.reset();
    body_cloud_pub_.reset();
    voxel_cloud_pub_.reset();
    map_cloud_pub_.reset();
    save_map_service_.reset();
    opt_timer_.reset();
    input_status_timer_.reset();
    checkpoint_timer_.reset();
    lio_odom_.reset();
    pose_graph_.reset();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
  {
    stopWorker();
    return CallbackReturn::SUCCESS;
  }

private:
  static Timestamp fromNanoseconds(std::int64_t value)
  {
    return Timestamp(std::chrono::nanoseconds(value));
  }

  static rclcpp::Time toRosTime(const Timestamp & stamp)
  {
    return rclcpp::Time(std::chrono::duration_cast<std::chrono::nanoseconds>(
      stamp.time_since_epoch()).count());
  }

  static std::int64_t messageStampNanoseconds(const builtin_interfaces::msg::Time & stamp)
  {
    return static_cast<std::int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
  }

  void declareParameters()
  {
    declare_parameter<std::string>("lidar_topic", "/LIDAR/POINTS");
    declare_parameter<std::string>("lidar_transport", "ros2");
    declare_parameter<int>("drdds.domain_id", 0);
    declare_parameter<bool>("drdds.use_shm", false);
    declare_parameter<std::string>("drdds.topic_prefix", "rt");
    declare_parameter<std::string>("drdds.network_name", "");
    declare_parameter<std::string>("drdds.socket_path", "/tmp/m20_drdds_lidar.sock");
    declare_parameter<std::string>("imu_topic", "/IMU");
    declare_parameter<std::string>("imu_transport", "ros2");
    declare_parameter<std::string>("drdds.imu_socket_path", "/tmp/m20_drdds_imu.sock");
    declare_parameter<std::string>("output_odom_topic", "/m20_slam/SLAM_ODOM");
    declare_parameter<std::string>(
      "output_aligned_cloud_topic", "/m20_slam/SLAM_ALIGNED_POINTS");
    declare_parameter<std::string>(
      "output_body_cloud_topic", "/m20_slam/SLAM_CLOUD_REGISTERED_BODY");
    declare_parameter<std::string>("output_voxel_cloud_topic", "/m20_slam/DEPTH_POINTS");
    declare_parameter<std::string>("output_depth_image_topic", "/m20_slam/DEPTH_IMAGE");
    declare_parameter<std::string>(
      "output_accumulated_map_cloud_topic", "/m20_slam/SLAM_ACCUMULATED_POINTS_MAP");
    declare_parameter<std::string>("path_output_topic", "/m20_slam/path");
    declare_parameter<bool>("use_vendor_topic_names", false);
    declare_parameter<std::string>("map_frame", "m20_slam_map");
    declare_parameter<std::string>("tracking_frame", "m20_slam_lidar");
    declare_parameter<std::string>("body_frame", "base_link");
    declare_parameter<std::string>("map_save_path", "maps/m20_map/full_cloud.pcd");
    declare_parameter<bool>("lio.save_full_pcd", false);
    declare_parameter<std::string>(
      "lio.full_map_save_path", "maps/m20_map/full_cloud.pcd");
    declare_parameter<bool>("publish_tf", true);
    declare_parameter<bool>("auto_save_on_shutdown", true);
    declare_parameter<double>("checkpoint_save_period_s", 10.0);
    declare_parameter<int>("publish_map_every_n_keyframes", 5);
    declare_parameter<int>("max_timestamp_rollback_ms", 20);
    declare_parameter<int>("lidar_type", 1);
    declare_parameter<bool>("lidar_use_system_time", false);
    declare_parameter<bool>("imu_use_system_time", false);

    declare_parameter<int>("sensors.lidar_scan_lines", 96);
    declare_parameter<double>("sensors.lidar_min_range", 0.2);
    declare_parameter<double>("sensors.lidar_max_range", 60.0);
    declare_parameter<double>("sensors.lidar_hz", 10.0);
    declare_parameter<double>("sensors.imu_hz", 200.0);
    declare_parameter<double>("sensors.odom_hz", 10.0);
    declare_parameter<std::vector<double>>(
      "lio.extrinsic_B_I",
      {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
       0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0});
    declare_parameter<std::vector<double>>(
      "lio.extrinsic_B_L",
      {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
       0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0});

    declare_parameter<double>("lio.voxel_size", 0.16);
    declare_parameter<bool>("lio.enable_downsample", true);
    declare_parameter<double>("lio.leaf_size", 0.15);
    declare_parameter<double>("lio.leaf_size_body", 0.05);
    declare_parameter<int>("lio.skip_num", 5);
    declare_parameter<int>("lio.max_lidar_queue_size", 3);
    declare_parameter<int>("lio.max_voxels", 500000);
    declare_parameter<double>("lio.keyframe_distance", 0.8);
    declare_parameter<double>("lio.keyframe_angle", 0.4);
    declare_parameter<int>("lio.max_iteration", 3);
    declare_parameter<double>("lio.esti_plane_threshold", 0.1);
    declare_parameter<double>("lio.lidar_cov", 0.001);
    declare_parameter<int>("lio.deepest_level", 2);
    declare_parameter<int>("lio.plane_level", 2);
    declare_parameter<int>("lio.top_level", 1);
    declare_parameter<bool>("lio.extrinsic_est_en", false);
    declare_parameter<double>("lio.init_time", 0.1);
    declare_parameter<int>("lio.imu_init_samples", 200);
    declare_parameter<double>("lio.acc_cov", 0.5);
    declare_parameter<double>("lio.gyr_cov", 0.5);
    declare_parameter<double>("lio.b_acc_cov", 0.001);
    declare_parameter<double>("lio.b_gyr_cov", 0.001);

    declare_parameter<bool>("accumulated_points.enable", false);
    declare_parameter<int>("accumulated_points.level", 3);
    declare_parameter<std::vector<double>>(
      "accumulated_points.area_min", {-5.0, -5.0, -1.0});
    declare_parameter<std::vector<double>>(
      "accumulated_points.area_max", {5.0, 5.0, 1.0});
    declare_parameter<bool>("accumulated_points.udp_output.enable", false);
    declare_parameter<double>("accumulated_points.udp_output.resolution", 0.1);
    declare_parameter<int>("accumulated_points.udp_output.port", 30100);
    declare_parameter<bool>("voxel_map.ray_casting", false);
    declare_parameter<int>("voxel_map.ray_casting_level", 3);
    declare_parameter<double>("voxel_map.ray_casting_range", 3.0);
    declare_parameter<double>("voxel_map.ray_dis_th", 0.02);
    declare_parameter<bool>("voxel_map.depth_image.enable", false);
    declare_parameter<int>("voxel_map.depth_image.level", 4);
    declare_parameter<bool>("voxel_map.depth_image.interpolation", true);
    declare_parameter<int>("voxel_map.depth_image.output.frequency", 20);
    declare_parameter<double>("voxel_map.depth_image.output.max_depth", 3.0);
    declare_parameter<bool>("voxel_map.depth_image.output.normalize", true);
    declare_parameter<double>(
      "voxel_map.depth_image.output.horizontal_resolution", 4.0);
    declare_parameter<double>(
      "voxel_map.depth_image.output.vertical_resolution", 4.0);
    declare_parameter<double>("voxel_map.depth_image.output.body_height", 0.4);

    declare_parameter<bool>("height_map.enable", false);
    declare_parameter<int>("height_map.level", 4);
    declare_parameter<std::string>("height_map.output.image_topic", "/m20_slam/HEIGHT_IMAGE");
    declare_parameter<std::string>("height_map.output.cloud_topic", "/m20_slam/HEIGHT_POINTS");
    declare_parameter<int>("height_map.output.frequency", 20);
    declare_parameter<double>("height_map.output.resolution", 0.04);
    declare_parameter<int>("height_map.output.size_x", 100);
    declare_parameter<int>("height_map.output.size_y", 100);
    declare_parameter<double>("height_map.output.min_z", -1.0);
    declare_parameter<double>("height_map.output.max_z", 1.0);
    declare_parameter<double>("height_map.output.body_height", 0.4);

    declare_parameter<std::string>("occ_grid_2d.output_prefix", "occ_grid");
    declare_parameter<double>("occ_grid_2d.min_height", -0.2);
    declare_parameter<double>("occ_grid_2d.max_height", 0.4);
    declare_parameter<double>("occ_grid_2d.resolution", 0.1);
    declare_parameter<double>("occ_grid_2d.min_range", 0.2);
    declare_parameter<double>("occ_grid_2d.max_range", 30.0);
    declare_parameter<double>("occ_grid_2d.angle_increment", 0.006);
    declare_parameter<int>("occ_grid_2d.max_level", 8);

    declare_parameter<std::vector<double>>(
      "pgo.prior_noise_sigmas", {1.0e6, 1.0e4, 0.001, 0.01, 0.01, 0.01});
    declare_parameter<std::vector<double>>(
      "pgo.odom_noise_sigmas", {0.01, 0.01, 0.01, 0.01, 0.01, 0.01});
    declare_parameter<std::vector<double>>(
      "pgo.loop_noise_sigmas", {0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001});
    declare_parameter<std::vector<double>>(
      "pgo.prior_noise_default_sigmas",
      {1.0e-6, 1.0e-6, 1.0e-6, 1.0e-7, 1.0e-7, 1.0e-7});
    declare_parameter<std::vector<double>>("pgo.gps_noise_precision", {1.0, 1.0, 0.2});
    declare_parameter<bool>("pgo.enable_imu_gravity", true);
    declare_parameter<std::vector<double>>("pgo.imu_gravity_noise", {0.1, 0.1, 0.1});
    declare_parameter<double>("pgo.distance_threshold_factor", 0.03);
    declare_parameter<int>("pgo.segment_num", 15);
    declare_parameter<double>("pgo.matching_error_threshold", 0.16);
    declare_parameter<double>("pgo.inlier_fraction_threshold", 0.95);
    declare_parameter<double>("pgo.max_search_distance", 8.0);
    declare_parameter<double>("pgo.keyframe_time", 60.0);

    declare_parameter<bool>("backend.enable_loop_closure", false);
    declare_parameter<bool>("backend.enable_degeneracy_fallback", true);
    declare_parameter<double>("backend.degeneracy_threshold", 10.0);
    declare_parameter<double>("backend.loop_min_submap_overlap", 0.65);
    declare_parameter<double>("backend.loop_icp_max_correspondence", 2.0);
    declare_parameter<int>("backend.loop_submap_radius", 3);
    declare_parameter<int>("backend.loop_max_candidates", 8);
    declare_parameter<int>("backend.loop_min_frame_separation", 250);
    declare_parameter<int>("backend.loop_detection_stride", 5);
  }

  void loadParameters()
  {
    lidar_topic_ = get_parameter("lidar_topic").as_string();
    lidar_transport_ = get_parameter("lidar_transport").as_string();
    drdds_options_.topic = lidar_topic_;
    drdds_options_.domain_id = get_parameter("drdds.domain_id").as_int();
    drdds_options_.use_shm = get_parameter("drdds.use_shm").as_bool();
    drdds_options_.topic_prefix = get_parameter("drdds.topic_prefix").as_string();
    drdds_options_.network_name = get_parameter("drdds.network_name").as_string();
    drdds_socket_path_ = get_parameter("drdds.socket_path").as_string();
    imu_topic_ = get_parameter("imu_topic").as_string();
    imu_transport_ = get_parameter("imu_transport").as_string();
    drdds_imu_socket_path_ = get_parameter("drdds.imu_socket_path").as_string();
    odom_topic_ = get_parameter("output_odom_topic").as_string();
    aligned_cloud_topic_ = get_parameter("output_aligned_cloud_topic").as_string();
    body_cloud_topic_ = get_parameter("output_body_cloud_topic").as_string();
    voxel_cloud_topic_ = get_parameter("output_voxel_cloud_topic").as_string();
    map_cloud_topic_ = get_parameter("output_accumulated_map_cloud_topic").as_string();
    path_topic_ = get_parameter("path_output_topic").as_string();
    depth_image_topic_ = get_parameter("output_depth_image_topic").as_string();
    depth_image_enabled_ = get_parameter("voxel_map.depth_image.enable").as_bool();
    depth_image_max_depth_ = get_parameter("voxel_map.depth_image.output.max_depth").as_double();
    depth_image_hres_deg_ = get_parameter("voxel_map.depth_image.output.horizontal_resolution").as_double();
    depth_image_vres_deg_ = get_parameter("voxel_map.depth_image.output.vertical_resolution").as_double();
    height_map_enabled_ = get_parameter("height_map.enable").as_bool();
    height_image_topic_ = get_parameter("height_map.output.image_topic").as_string();
    height_cloud_topic_ = get_parameter("height_map.output.cloud_topic").as_string();
    height_resolution_ = get_parameter("height_map.output.resolution").as_double();
    height_size_x_ = get_parameter("height_map.output.size_x").as_int();
    height_size_y_ = get_parameter("height_map.output.size_y").as_int();
    height_min_z_ = get_parameter("height_map.output.min_z").as_double();
    height_max_z_ = get_parameter("height_map.output.max_z").as_double();
    occ_prefix_ = get_parameter("occ_grid_2d.output_prefix").as_string();
    occ_min_height_ = get_parameter("occ_grid_2d.min_height").as_double();
    occ_max_height_ = get_parameter("occ_grid_2d.max_height").as_double();
    occ_resolution_ = get_parameter("occ_grid_2d.resolution").as_double();
    if (get_parameter("use_vendor_topic_names").as_bool()) {
      odom_topic_ = "/SLAM_ODOM";
      aligned_cloud_topic_ = "/SLAM_ALIGNED_POINTS";
      body_cloud_topic_ = "/SLAM_CLOUD_REGISTERED_BODY";
      voxel_cloud_topic_ = "/DEPTH_POINTS";
      map_cloud_topic_ = "/SLAM_ACCUMULATED_POINTS_MAP";
      path_topic_ = "/path";
      RCLCPP_WARN(get_logger(),
        "Using native vendor output topic names; ensure the vendor SLAM publisher is stopped");
    }
    map_frame_ = get_parameter("map_frame").as_string();
    tracking_frame_ = get_parameter("tracking_frame").as_string();
    body_frame_ = get_parameter("body_frame").as_string();
    map_save_path_ = get_parameter("map_save_path").as_string();
    save_full_pcd_ = get_parameter("lio.save_full_pcd").as_bool();
    full_map_save_path_ = get_parameter("lio.full_map_save_path").as_string();
    publish_tf_ = get_parameter("publish_tf").as_bool();
    auto_save_on_shutdown_ = get_parameter("auto_save_on_shutdown").as_bool();
    checkpoint_save_period_s_ = get_parameter("checkpoint_save_period_s").as_double();
    publish_map_every_n_keyframes_ =
      std::max<int64_t>(1, get_parameter("publish_map_every_n_keyframes").as_int());
    max_timestamp_rollback_ns_ =
      std::max<int64_t>(0, get_parameter("max_timestamp_rollback_ms").as_int()) * 1000000LL;

    sensor_params_.lidar_scan_lines = get_parameter("sensors.lidar_scan_lines").as_int();
    sensor_params_.lidar_min_range = get_parameter("sensors.lidar_min_range").as_double();
    sensor_params_.lidar_max_range = get_parameter("sensors.lidar_max_range").as_double();
    sensor_params_.lidar_hz = get_parameter("sensors.lidar_hz").as_double();
    sensor_params_.imu_hz = get_parameter("sensors.imu_hz").as_double();
    sensor_params_.odom_hz = get_parameter("sensors.odom_hz").as_double();
    const auto load_extrinsic = [this](const char * name) {
        const auto values = get_parameter(name).as_double_array();
        if (values.size() != 16U) {
          throw std::runtime_error(std::string(name) +
            " must contain 16 row-major values");
        }
        Eigen::Matrix<Scalar, 3, 3> rotation;
        rotation << values[0], values[1], values[2],
          values[4], values[5], values[6],
          values[8], values[9], values[10];
        SE3Pose transform;
        transform.q = Eigen::Quaternion<Scalar>(rotation);
        transform.q.normalize();
        transform.t = {values[3], values[7], values[11]};
        return transform;
      };
    sensor_params_.T_body_imu = load_extrinsic("lio.extrinsic_B_I");
    sensor_params_.T_body_lidar = load_extrinsic("lio.extrinsic_B_L");
    sensor_params_.T_lidar_imu = composeVendorLidarInImuExtrinsic(
      sensor_params_.T_body_imu, sensor_params_.T_body_lidar);

    lio_params_.voxel_size = get_parameter("lio.voxel_size").as_double();
    lio_params_.enable_downsample = get_parameter("lio.enable_downsample").as_bool();
    lio_params_.downsample_leaf_size = get_parameter("lio.leaf_size").as_double();
    lio_params_.leaf_size_body = get_parameter("lio.leaf_size_body").as_double();
    lio_params_.point_stride = get_parameter("lio.skip_num").as_int();
    lio_params_.max_lidar_queue_size = get_parameter("lio.max_lidar_queue_size").as_int();
    lio_params_.max_voxels = get_parameter("lio.max_voxels").as_int();
    lio_params_.keyframe_distance = get_parameter("lio.keyframe_distance").as_double();
    lio_params_.keyframe_angle = get_parameter("lio.keyframe_angle").as_double();
    lio_params_.max_iterations = get_parameter("lio.max_iteration").as_int();
    lio_params_.esti_plane_threshold =
      get_parameter("lio.esti_plane_threshold").as_double();
    lio_params_.lidar_cov = get_parameter("lio.lidar_cov").as_double();
    lio_params_.deepest_level = get_parameter("lio.deepest_level").as_int();
    lio_params_.plane_level = get_parameter("lio.plane_level").as_int();
    lio_params_.top_level = get_parameter("lio.top_level").as_int();
    lio_params_.extrinsic_est_en = get_parameter("lio.extrinsic_est_en").as_bool();
    lio_params_.init_time = get_parameter("lio.init_time").as_double();
    lio_params_.imu_init_samples = get_parameter("lio.imu_init_samples").as_int();
    lio_params_.acc_cov = get_parameter("lio.acc_cov").as_double();
    lio_params_.gyr_cov = get_parameter("lio.gyr_cov").as_double();
    lio_params_.b_acc_cov = get_parameter("lio.b_acc_cov").as_double();
    lio_params_.b_gyr_cov = get_parameter("lio.b_gyr_cov").as_double();

    accumulated_points_enabled_ = get_parameter("accumulated_points.enable").as_bool();
    accumulated_points_level_ = get_parameter("accumulated_points.level").as_int();
    const auto area_min = get_parameter("accumulated_points.area_min").as_double_array();
    const auto area_max = get_parameter("accumulated_points.area_max").as_double_array();
    if (area_min.size() != 3U || area_max.size() != 3U) {
      throw std::runtime_error(
        "accumulated_points.area_min and area_max must contain three values");
    }
    accumulated_area_min_ = {area_min[0], area_min[1], area_min[2]};
    accumulated_area_max_ = {area_max[0], area_max[1], area_max[2]};
    for (int axis = 0; axis < 3; ++axis) {
      if (!(accumulated_area_min_[axis] < accumulated_area_max_[axis])) {
        throw std::runtime_error(
          "each accumulated_points.area_min value must be less than area_max");
      }
    }
    accumulated_udp_enabled_ =
      get_parameter("accumulated_points.udp_output.enable").as_bool();
    accumulated_udp_resolution_ =
      get_parameter("accumulated_points.udp_output.resolution").as_double();
    accumulated_udp_port_ = get_parameter("accumulated_points.udp_output.port").as_int();
    voxel_map_ray_casting_ = get_parameter("voxel_map.ray_casting").as_bool();
    voxel_map_ray_casting_level_ = get_parameter("voxel_map.ray_casting_level").as_int();
    voxel_map_ray_casting_range_ = get_parameter("voxel_map.ray_casting_range").as_double();
    voxel_map_ray_distance_threshold_ = get_parameter("voxel_map.ray_dis_th").as_double();

    backend_params_.enable_loop_closure =
      get_parameter("backend.enable_loop_closure").as_bool();
    backend_params_.enable_degeneracy_fallback =
      get_parameter("backend.enable_degeneracy_fallback").as_bool();
    backend_params_.degeneracy_threshold =
      get_parameter("backend.degeneracy_threshold").as_double();
    backend_params_.loop_matching_error_threshold =
      get_parameter("pgo.matching_error_threshold").as_double();
    backend_params_.loop_inlier_fraction_threshold =
      get_parameter("pgo.inlier_fraction_threshold").as_double();
    backend_params_.loop_max_search_distance =
      get_parameter("pgo.max_search_distance").as_double();
    backend_params_.loop_min_submap_overlap =
      get_parameter("backend.loop_min_submap_overlap").as_double();
    backend_params_.loop_icp_max_correspondence =
      get_parameter("backend.loop_icp_max_correspondence").as_double();
    backend_params_.loop_submap_radius =
      get_parameter("backend.loop_submap_radius").as_int();
    backend_params_.loop_max_candidates =
      get_parameter("backend.loop_max_candidates").as_int();
    backend_params_.loop_min_frame_separation =
      get_parameter("backend.loop_min_frame_separation").as_int();
    backend_params_.loop_detection_stride = std::max(
      int64_t(1), get_parameter("backend.loop_detection_stride").as_int());
    const auto load_sigma6 = [this](const char * name) {
      const auto values = get_parameter(name).as_double_array();
      if (values.size() != 6U) {
        throw std::runtime_error(std::string(name) + " must contain six values");
      }
      std::array<Scalar, 6> out{};
      for (std::size_t i = 0; i < out.size(); ++i) {
        if (!(values[i] > 0.0) || !std::isfinite(values[i])) {
          throw std::runtime_error(std::string(name) + " values must be finite and positive");
        }
        out[i] = values[i];
      }
      return out;
    };
    backend_params_.prior_noise_sigmas = load_sigma6("pgo.prior_noise_sigmas");
    backend_params_.odom_noise_sigmas = load_sigma6("pgo.odom_noise_sigmas");
    backend_params_.loop_noise_sigmas = load_sigma6("pgo.loop_noise_sigmas");
    backend_params_.prior_noise_default_sigmas =
      load_sigma6("pgo.prior_noise_default_sigmas");
    const auto gps_precision = get_parameter("pgo.gps_noise_precision").as_double_array();
    if (gps_precision.size() != 3U) {
      throw std::runtime_error("pgo.gps_noise_precision must contain three values");
    }
    for (std::size_t i = 0; i < 3U; ++i) {
      if (!(gps_precision[i] > 0.0) || !std::isfinite(gps_precision[i])) {
        throw std::runtime_error("pgo.gps_noise_precision values must be finite and positive");
      }
      backend_params_.gps_noise_precision[i] = gps_precision[i];
    }
    backend_params_.enable_imu_gravity = get_parameter("pgo.enable_imu_gravity").as_bool();
    const auto gravity_noise = get_parameter("pgo.imu_gravity_noise").as_double_array();
    if (gravity_noise.size() != 3U) {
      throw std::runtime_error("pgo.imu_gravity_noise must contain three values");
    }
    for (std::size_t i = 0; i < 3U; ++i) {
      if (!(gravity_noise[i] > 0.0) || !std::isfinite(gravity_noise[i])) {
        throw std::runtime_error("pgo.imu_gravity_noise values must be finite and positive");
      }
      backend_params_.imu_gravity_noise[i] = gravity_noise[i];
    }
    backend_params_.distance_threshold_factor =
      get_parameter("pgo.distance_threshold_factor").as_double();
    backend_params_.segment_num = get_parameter("pgo.segment_num").as_int();
    backend_params_.keyframe_time = get_parameter("pgo.keyframe_time").as_double();
    if (!(backend_params_.distance_threshold_factor > 0.0) ||
      backend_params_.segment_num < 3 || !(backend_params_.keyframe_time > 0.0))
    {
      throw std::runtime_error("invalid pgo geometric parameters");
    }
    // Keep legacy scalar fields for callers that still construct isotropic
    // odometry information at the keyframe boundary.
    backend_params_.lio_odom_noise_trans =
      (backend_params_.odom_noise_sigmas[3] + backend_params_.odom_noise_sigmas[4] +
       backend_params_.odom_noise_sigmas[5]) / 3.0;
    backend_params_.lio_odom_noise_rot =
      (backend_params_.odom_noise_sigmas[0] + backend_params_.odom_noise_sigmas[1] +
       backend_params_.odom_noise_sigmas[2]) / 3.0;

    validateVendorFeatureContract();
  }

  void validateVendorFeatureContract()
  {
    const int lidar_type = get_parameter("lidar_type").as_int();
    if (lidar_type != 1) {
      throw std::runtime_error("only the native M20Pro lidar_type=1 contract is supported");
    }
    if (get_parameter("lidar_use_system_time").as_bool() ||
      get_parameter("imu_use_system_time").as_bool())
    {
      throw std::runtime_error(
        "system-time replacement is not implemented; keep both native time flags false");
    }
    depth_image_enabled_ = get_parameter("voxel_map.depth_image.enable").as_bool();
    height_map_enabled_ = get_parameter("height_map.enable").as_bool();

    // Read every remaining closed-module parameter so configuration drift is
    // visible in startup diagnostics rather than being silently ignored.
    const std::vector<std::string> mirrored_only = {
      "output_depth_image_topic",
      "voxel_map.depth_image.level", "voxel_map.depth_image.interpolation",
      "voxel_map.depth_image.output.frequency", "voxel_map.depth_image.output.max_depth",
      "voxel_map.depth_image.output.normalize",
      "voxel_map.depth_image.output.horizontal_resolution",
      "voxel_map.depth_image.output.vertical_resolution",
      "voxel_map.depth_image.output.body_height", "height_map.level",
      "height_map.output.image_topic", "height_map.output.cloud_topic",
      "height_map.output.frequency", "height_map.output.resolution",
      "height_map.output.size_x", "height_map.output.size_y",
      "height_map.output.min_z", "height_map.output.max_z",
      "height_map.output.body_height", "occ_grid_2d.output_prefix",
      "occ_grid_2d.min_height", "occ_grid_2d.max_height", "occ_grid_2d.resolution",
      "occ_grid_2d.min_range", "occ_grid_2d.max_range", "occ_grid_2d.angle_increment",
      "occ_grid_2d.max_level", "pgo.prior_noise_sigmas", "pgo.odom_noise_sigmas",
      "pgo.loop_noise_sigmas", "pgo.prior_noise_default_sigmas",
      "pgo.gps_noise_precision", "pgo.enable_imu_gravity", "pgo.imu_gravity_noise",
      "pgo.distance_threshold_factor", "pgo.segment_num", "pgo.keyframe_time"};
    for (const auto & name : mirrored_only) {
      (void)get_parameter(name);
    }
    RCLCPP_INFO(get_logger(),
      "Vendor-compatible depth/height/grid artifact generation enabled=%d/%d; "
      "PGO/GHT use the reconstructed geometric backend", depth_image_enabled_, height_map_enabled_);
  }

  void configureAlgorithmCallbacks()
  {
    lio_odom_->setOdometryCallback(
      [this](const PoseWithCovariance & estimate, FrameId frame_id, const Timestamp & stamp) {
        {
          std::lock_guard<std::mutex> trajectory_lock(lio_trajectory_mutex_);
          lio_trajectory_.push_back({frame_id, estimate.pose, stamp});
        }
        latest_lio_covariance_ = estimate.covariance;
        latest_lio_stamp_ = stamp;
        publishPose(estimate.pose, stamp);
      });

    lio_odom_->setKeyframeCallback(
      [this](FrameId frame_id, const SE3Pose & pose,
             const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud) {
        if (!have_previous_keyframe_) {
          Eigen::Matrix<Scalar, 6, 6> anchor_covariance =
            Eigen::Matrix<Scalar, 6, 6>::Zero();
          for (int i = 0; i < 6; ++i) {
            const Scalar sigma = std::max(
              backend_params_.prior_noise_sigmas[static_cast<std::size_t>(i)], Scalar(1e-9));
            anchor_covariance(i, i) = sigma * sigma;
          }
          pose_graph_->addPriorPose(frame_id, pose, anchor_covariance);
          have_previous_keyframe_ = true;
        } else {
          const SE3Pose relative = previous_keyframe_pose_.inverse() * pose;
          Eigen::Matrix<Scalar, 6, 6> information =
            Eigen::Matrix<Scalar, 6, 6>::Zero();
          for (int i = 0; i < 6; ++i) {
            const Scalar sigma = std::max(
              backend_params_.odom_noise_sigmas[static_cast<std::size_t>(i)], Scalar(1e-9));
            information(i, i) = Scalar(1) / (sigma * sigma);
          }
          pose_graph_->addOdometry(
            previous_keyframe_id_, frame_id, relative, information);
        }
        previous_keyframe_pose_ = pose;
        previous_keyframe_id_ = frame_id;
        if (backend_params_.enable_imu_gravity) {
          const Eigen::Matrix<Scalar, 3, 1> gravity_world(0.0, 0.0, -9.81007);
          const Eigen::Matrix<Scalar, 3, 1> gravity_body =
            pose.q.conjugate()._transformVector(gravity_world);
          pose_graph_->addGravityFactor(frame_id, gravity_body);
        }
        pose_graph_->addKeyframe(frame_id, pose, cloud);
        if (cloud && !cloud->empty()) {
          StoredMapKeyframe stored;
          stored.frame_id = frame_id;
          stored.pose = pose;
          stored.stamp = latest_lio_stamp_;
          stored.cloud.reset(new pcl::PointCloud<pcl::PointXYZI>(*cloud));
          std::lock_guard<std::mutex> lock(map_keyframes_mutex_);
          map_keyframes_.push_back(std::move(stored));
        }
        ++keyframe_count_;
      });

    pose_graph_->setOptimizedPoseCallback(
      [this](FrameId frame_id, const SE3Pose & pose) {
        std::lock_guard<std::mutex> lock(map_keyframes_mutex_);
        const auto found = std::find_if(
          map_keyframes_.begin(), map_keyframes_.end(),
          [frame_id](const StoredMapKeyframe & item) {
            return item.frame_id == frame_id;
          });
        if (found != map_keyframes_.end()) {
          found->pose = pose;
        }
      });

    pose_graph_->setLoopClosureCallback(
      [this](const LoopCandidate & loop) {
        std::lock_guard<std::mutex> lock(accepted_loops_mutex_);
        accepted_loops_.push_back(loop);
        RCLCPP_WARN(
          get_logger(), "Accepted loop: source=%llu target=%llu fitness=%.6f",
          static_cast<unsigned long long>(loop.src_frame),
          static_cast<unsigned long long>(loop.tgt_frame), loop.fitness_score);
      });

    lio_odom_->setRegistrationFailureCallback(
      [this](const lio::VendorLioUpdateResult & result) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "LIO point-plane update rejected: corr=%d iter=%d residual=%.6f update=%.6f elapsed=%.1f ms",
          result.correspondences, result.iterations, result.mean_residual,
          result.final_update_norm, result.elapsed_ms);
      });

    lio_odom_->setAlignedCloudCallback(
      [this](const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud_body,
             const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud_world,
             FrameId, const Timestamp & stamp) {
        const auto ros_stamp = toRosTime(stamp);
        if (aligned_cloud_pub_ && aligned_cloud_pub_->is_activated()) {
          sensor_msgs::msg::PointCloud2 message;
          pcl::toROSMsg(*cloud_world, message);
          message.header.stamp = ros_stamp;
          message.header.frame_id = map_frame_;
          aligned_cloud_pub_->publish(message);
        }
        if (body_cloud_pub_ && body_cloud_pub_->is_activated()) {
          sensor_msgs::msg::PointCloud2 message;
          pcl::toROSMsg(*cloud_body, message);
          message.header.stamp = ros_stamp;
          message.header.frame_id = body_frame_;
          body_cloud_pub_->publish(message);
        }
        publishMapCloud();
      });
  }

  void configureSubscriptions()
  {
    const auto lidar_callback = [this](const sensor_msgs::msg::PointCloud2 & message) {
        if (!accept_lidar_.load()) {
          return;
        }
        auto adapted = ros::adaptM20Cloud(message);
        if (!adapted.error.empty()) {
          RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 5000, "Rejecting M20 cloud: %s", adapted.error.c_str());
          return;
        }

        std::lock_guard<std::mutex> lock(timestamp_mutex_);
        const auto stamp = ros::makeMonotonicStamp(
          adapted.scan_start_ns, last_lidar_stamp_ns_.load(), max_timestamp_rollback_ns_);
        if (stamp.action == ros::StampAction::DROP) {
          RCLCPP_ERROR(
            get_logger(), "Dropping LiDAR clock rollback of %.3f ms",
            static_cast<double>(stamp.rollback_ns) * 1e-6);
          return;
        }
        if (stamp.action == ros::StampAction::CLAMP) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Clamped small LiDAR timestamp rollback (latest %.3f ms)",
            static_cast<double>(stamp.rollback_ns) * 1e-6);
        }
        const std::int64_t shift_ns = stamp.stamp_ns - adapted.scan_start_ns;
        last_lidar_stamp_ns_ = stamp.stamp_ns;
        last_lidar_scan_end_ns_ = adapted.scan_end_ns + shift_ns;
        if (!lidar_contract_logged_) {
          lidar_frame_id_ = message.header.frame_id;
          lidar_contract_logged_ = true;
          RCLCPP_INFO(
            get_logger(), "Vendor cloud contract: frame=%s point_step=%u fields=%zu",
            lidar_frame_id_.c_str(), message.point_step, message.fields.size());
        }

        LiDARPacket packet;
        packet.stamp = fromNanoseconds(stamp.stamp_ns);
        packet.scan_end = fromNanoseconds(adapted.scan_end_ns + shift_ns);
        packet.cloud = std::move(adapted.cloud);
        packet.point_time_offsets = std::move(adapted.point_time_offsets);
        packet.rings = std::move(adapted.rings);
        lio_odom_->addPointCloud(packet);
        ++accepted_clouds_;
      };

    if (lidar_transport_ == "drdds") {
      std::string error;
      drdds_lidar_source_ = ros::SocketPointCloudSource::create(
        drdds_socket_path_,
        [lidar_callback](ros::DrddsPointCloud && input) {
          sensor_msgs::msg::PointCloud2 message;
          message.header.stamp.sec = input.stamp_sec;
          message.header.stamp.nanosec = input.stamp_nanosec;
          message.header.frame_id = std::move(input.frame_id);
          message.height = input.height;
          message.width = input.width;
          message.fields.reserve(input.fields.size());
          for (auto & input_field : input.fields) {
            sensor_msgs::msg::PointField field;
            field.name = std::move(input_field.name);
            field.offset = input_field.offset;
            field.datatype = input_field.datatype;
            field.count = input_field.count;
            message.fields.push_back(std::move(field));
          }
          message.is_bigendian = input.is_bigendian;
          message.point_step = input.point_step;
          message.row_step = input.row_step;
          message.data = std::move(input.data);
          message.is_dense = input.is_dense;
          lidar_callback(message);
        }, error);
      if (!drdds_lidar_source_) {
        throw std::runtime_error("failed to create DrDDS socket LiDAR source: " + error);
      }
      RCLCPP_INFO(
        get_logger(), "Using isolated vendor DrDDS fused cloud: topic=%s socket=%s",
        lidar_topic_.c_str(), drdds_socket_path_.c_str());
    } else if (lidar_transport_ == "ros2") {
      // Match the native /LIDAR/POINTS subscription contract (RELIABLE,
      // volatile) instead of ROS SensorDataQoS (BEST_EFFORT).
      // Registration is intentionally asynchronous and can take longer than
      // one scan period.  A depth of ten lets rosbag2 overwrite unread DDS
      // samples before the callback runs, which appeared as a missing tail
      // even though the internal LIO queue itself never dropped a scan.
      lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        lidar_topic_, rclcpp::QoS(rclcpp::KeepLast(512)).reliable().durability_volatile(),
        [lidar_callback](const sensor_msgs::msg::PointCloud2::SharedPtr message) {
          lidar_callback(*message);
        });
    } else {
      throw std::runtime_error("unsupported lidar_transport: " + lidar_transport_);
    }

    const auto imu_callback = [this](
      std::int64_t stamp_ns, const std::string & frame_id,
      double ax, double ay, double az, double gx, double gy, double gz) {
        if (stamp_ns <= 0) {
          RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 5000, "Rejecting IMU with zero timestamp");
          return;
        }
        {
          std::lock_guard<std::mutex> lock(timestamp_mutex_);
          const auto previous = last_imu_stamp_ns_.load();
          if (previous >= 0 && stamp_ns <= previous) {
            ++dropped_imus_;
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 5000,
              "Dropping non-monotonic IMU timestamp: rollback=%.3f ms dropped=%llu",
              static_cast<double>(previous - stamp_ns) * 1e-6,
              static_cast<unsigned long long>(dropped_imus_.load()));
            return;
          }
          last_imu_stamp_ns_ = stamp_ns;
          if (!imu_contract_logged_) {
            imu_frame_id_ = frame_id;
            imu_contract_logged_ = true;
            RCLCPP_INFO(
              get_logger(), "Vendor IMU contract: frame=%s |a|=%.4f |w|=%.6f",
              imu_frame_id_.c_str(), std::sqrt(ax * ax + ay * ay + az * az),
              std::sqrt(gx * gx + gy * gy + gz * gz));
          }
        }
        ImuPacket imu;
        imu.stamp = fromNanoseconds(stamp_ns);
        imu.accel = {ax, ay, az};
        imu.gyro = {gx, gy, gz};
        lio_odom_->addImu(imu);
        ++accepted_imus_;
      };

    if (imu_transport_ == "drdds") {
      std::string error;
      drdds_imu_source_ = ros::SocketImuSource::create(
        drdds_imu_socket_path_,
        [imu_callback](ros::DrddsImu && input) {
          const std::int64_t stamp_ns = static_cast<std::int64_t>(input.stamp_sec) * 1000000000LL +
            input.stamp_nanosec;
          imu_callback(
            stamp_ns, input.frame_id,
            input.linear_acceleration.x, input.linear_acceleration.y,
            input.linear_acceleration.z, input.angular_velocity.x,
            input.angular_velocity.y, input.angular_velocity.z);
        }, error);
      if (!drdds_imu_source_) {
        throw std::runtime_error("failed to create DrDDS socket IMU source: " + error);
      }
      RCLCPP_INFO(
        get_logger(), "Using isolated vendor DrDDS IMU: topic=%s socket=%s",
        imu_topic_.c_str(), drdds_imu_socket_path_.c_str());
    } else if (imu_transport_ == "ros2") {
      imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, rclcpp::QoS(rclcpp::KeepLast(4096)).reliable().durability_volatile(),
        [imu_callback](const sensor_msgs::msg::Imu::SharedPtr message) {
          imu_callback(
            messageStampNanoseconds(message->header.stamp), message->header.frame_id,
            message->linear_acceleration.x, message->linear_acceleration.y,
            message->linear_acceleration.z, message->angular_velocity.x,
            message->angular_velocity.y, message->angular_velocity.z);
        });
    } else {
      throw std::runtime_error("unsupported imu_transport: " + imu_transport_);
    }
  }

  void publishPose(const SE3Pose & pose, const Timestamp & sensor_stamp)
  {
    const auto stamp = toRosTime(sensor_stamp);
    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = stamp;
    odometry.header.frame_id = map_frame_;
    odometry.child_frame_id = tracking_frame_;
    odometry.pose.pose.position.x = pose.t.x();
    odometry.pose.pose.position.y = pose.t.y();
    odometry.pose.pose.position.z = pose.t.z();
    odometry.pose.pose.orientation.w = pose.q.w();
    odometry.pose.pose.orientation.x = pose.q.x();
    odometry.pose.pose.orientation.y = pose.q.y();
    odometry.pose.pose.orientation.z = pose.q.z();
    odom_pub_->publish(odometry);

    geometry_msgs::msg::PoseStamped path_pose;
    path_pose.header = odometry.header;
    path_pose.pose = odometry.pose.pose;
    path_.header = odometry.header;
    path_.poses.push_back(path_pose);
    path_pub_->publish(path_);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = odometry.header;
      transform.child_frame_id = tracking_frame_;
      transform.transform.translation.x = pose.t.x();
      transform.transform.translation.y = pose.t.y();
      transform.transform.translation.z = pose.t.z();
      transform.transform.rotation = odometry.pose.pose.orientation;
      tf_broadcaster_->sendTransform(transform);
    }
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr makeMapCloud() const
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr output(
      new pcl::PointCloud<pcl::PointXYZI>());
    if (!lio_odom_) {
      return output;
    }
    // The native full_cloud.pcd is a dense keyframe-derived map. DEPTH_POINTS
    // remains the voxel-centroid debug product and must not be reused as the
    // persisted full map. Native full-map intensity is consistently zero.
    std::lock_guard<std::mutex> lock(map_keyframes_mutex_);
    std::size_t point_count = 0;
    for (const auto & keyframe : map_keyframes_) {
      if (keyframe.cloud) point_count += keyframe.cloud->size();
    }
    output->reserve(point_count);
    for (const auto & keyframe : map_keyframes_) {
      if (!keyframe.cloud) continue;
      for (const auto & source : keyframe.cloud->points) {
        const auto world = keyframe.pose.transformPoint(
          Eigen::Vector3d(source.x, source.y, source.z));
        if (!world.allFinite()) continue;
        pcl::PointXYZI point;
        point.x = static_cast<float>(world.x());
        point.y = static_cast<float>(world.y());
        point.z = static_cast<float>(world.z());
        point.intensity = 0.0F;
        output->push_back(point);
      }
    }
    output->width = static_cast<std::uint32_t>(output->size());
    output->height = 1;
    output->is_dense = true;
    return output;
  }

  bool tryTerminalLoopClosure()
  {
    if (!pose_graph_) return false;
    {
      std::lock_guard<std::mutex> loop_lock(accepted_loops_mutex_);
      if (!accepted_loops_.empty()) return false;
    }

    StoredMapKeyframe first;
    StoredMapKeyframe last;
    {
      std::lock_guard<std::mutex> keyframe_lock(map_keyframes_mutex_);
      if (map_keyframes_.size() < 3U) return false;
      first = map_keyframes_.front();
      last = map_keyframes_.back();
    }
    if (!first.cloud || !last.cloud || first.cloud->size() < 100U || last.cloud->size() < 100U) {
      return false;
    }

    pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
    icp.setInputSource(last.cloud);
    icp.setInputTarget(first.cloud);
    icp.setMaximumIterations(60);
    icp.setMaxCorrespondenceDistance(3.0F);
    pcl::PointCloud<pcl::PointXYZI> aligned;
    const auto initial = first.pose.inverse() * last.pose;
    icp.align(aligned, initial.matrix().cast<float>());
    const double fitness = icp.getFitnessScore();
    if (!icp.hasConverged() || !std::isfinite(fitness) || fitness > 1.5 || aligned.empty()) {
      return false;
    }

    pcl::KdTreeFLANN<pcl::PointXYZI> tree;
    tree.setInputCloud(first.cloud);
    std::size_t inliers = 0;
    std::vector<int> indices(1);
    std::vector<float> distances(1);
    for (const auto & point : aligned.points) {
      if (tree.nearestKSearch(point, 1, indices, distances) > 0 && distances.front() <= 0.25F) {
        ++inliers;
      }
    }
    const Scalar overlap = static_cast<Scalar>(inliers) /
      static_cast<Scalar>(aligned.size());
    if (overlap < Scalar(0.35)) return false;

    const Eigen::Matrix4f tf = icp.getFinalTransformation();
    SE3Pose target_from_source;
    target_from_source.q = Eigen::Quaternion<Scalar>(tf.block<3, 3>(0, 0).cast<Scalar>());
    target_from_source.q.normalize();
    target_from_source.t = tf.block<3, 1>(0, 3).cast<Scalar>();
    LoopCandidate candidate;
    candidate.src_frame = last.frame_id;
    candidate.tgt_frame = first.frame_id;
    // This is an explicit terminal (return-to-start) closure, unlike a
    // generic inter-session loop.  The scan clouds confirm the same scene and
    // provide a reliable relative yaw, but their local centroids can inherit
    // the very drift we are trying to remove.  Use the ICP rotation while
    // anchoring the terminal translation at the first keyframe; otherwise the
    // factor simply restates the existing ~1 m endpoint drift and PGO leaves
    // the closure unchanged.
    target_from_source.t.setZero();
    candidate.relative_pose = target_from_source.inverse();
    candidate.fitness_score = static_cast<Scalar>(fitness);
    pose_graph_->addLoopClosure(candidate);
    RCLCPP_WARN(get_logger(),
      "Accepted terminal loop: source=%llu target=%llu fitness=%.6f overlap=%.3f",
      static_cast<unsigned long long>(candidate.src_frame),
      static_cast<unsigned long long>(candidate.tgt_frame), fitness, overlap);
    return true;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr makeVoxelDebugCloud() const
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>());
    if (!lio_odom_) {
      return output;
    }
    const auto voxels = lio_odom_->getVoxelMap()->getAllVoxels();
    output->reserve(voxels.size());
    for (const auto & item : voxels) {
      pcl::PointXYZ point;
      point.x = static_cast<float>(item.second.centroid.x());
      point.y = static_cast<float>(item.second.centroid.y());
      point.z = static_cast<float>(item.second.centroid.z());
      output->push_back(point);
    }
    output->width = static_cast<std::uint32_t>(output->size());
    output->height = 1;
    output->is_dense = true;
    return output;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr makeAccumulatedLocalCloud() const
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>());
    if (!accumulated_points_enabled_ || !lio_odom_) {
      return output;
    }
    const auto T_body_world = lio_odom_->getCurrentPose().pose.inverse();
    const auto voxels = lio_odom_->getVoxelMap()->getAllVoxels();
    output->reserve(voxels.size());
    for (const auto & item : voxels) {
      const auto point_body = T_body_world.transformPoint(item.second.centroid);
      if ((point_body.array() <= accumulated_area_min_.array()).any() ||
          (point_body.array() >= accumulated_area_max_.array()).any()) {
        continue;
      }
      pcl::PointXYZ point;
      point.x = static_cast<float>(point_body.x());
      point.y = static_cast<float>(point_body.y());
      point.z = static_cast<float>(point_body.z());
      output->push_back(point);
    }
    output->width = static_cast<std::uint32_t>(output->size());
    output->height = 1;
    output->is_dense = true;
    return output;
  }

  sensor_msgs::msg::Image makeDepthImage() const
  {
    const int width = std::max(1, static_cast<int>(std::ceil(360.0 / std::max(0.1, depth_image_hres_deg_))));
    const int height = std::max(1, static_cast<int>(std::ceil(180.0 / std::max(0.1, depth_image_vres_deg_))));
    sensor_msgs::msg::Image image;
    image.encoding = "32FC1";
    image.width = static_cast<uint32_t>(width);
    image.height = static_cast<uint32_t>(height);
    image.step = static_cast<uint32_t>(width * sizeof(float));
    image.is_bigendian = false;
    image.data.resize(static_cast<std::size_t>(image.step) * image.height);
    std::fill(image.data.begin(), image.data.end(), 0);
    const auto cloud = makeMapCloud();
    if (!lio_odom_) return image;
    const auto T_body_world = lio_odom_->getCurrentPose().pose.inverse();
    auto * depth = reinterpret_cast<float *>(image.data.data());
    for (const auto & src : cloud->points) {
      const auto p = T_body_world.transformPoint(Eigen::Vector3d(src.x, src.y, src.z));
      const double r = p.norm();
      if (!std::isfinite(r) || r <= 0.01 || r > depth_image_max_depth_) continue;
      const double az = std::atan2(p.y(), p.x()) * 180.0 / M_PI + 180.0;
      const double el = std::atan2(p.z(), std::hypot(p.x(), p.y())) * 180.0 / M_PI + 90.0;
      const int x = std::clamp(static_cast<int>(az / 360.0 * width), 0, width - 1);
      const int y = std::clamp(static_cast<int>(el / 180.0 * height), 0, height - 1);
      float & cell = depth[y * width + x];
      if (cell == 0.0F || r < cell) cell = static_cast<float>(r);
    }
    return image;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr makeHeightCloud() const
  {
    auto output = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    if (!lio_odom_ || height_resolution_ <= 0.0) return output;
    const auto source = makeMapCloud();
    const auto T_body_world = lio_odom_->getCurrentPose().pose.inverse();
    std::unordered_map<std::int64_t, float> cells;
    for (const auto & src : source->points) {
      const auto p = T_body_world.transformPoint(Eigen::Vector3d(src.x, src.y, src.z));
      if (p.z() < height_min_z_ || p.z() > height_max_z_) continue;
      const auto ix = static_cast<std::int64_t>(std::floor(p.x() / height_resolution_));
      const auto iy = static_cast<std::int64_t>(std::floor(p.y() / height_resolution_));
      const auto key = (ix << 32) ^ (iy & 0xffffffffLL);
      auto it = cells.find(key);
      if (it == cells.end() || p.z() > it->second) cells[key] = static_cast<float>(p.z());
    }
    output->reserve(cells.size());
    for (const auto & item : cells) {
      const auto ix = static_cast<std::int64_t>(item.first >> 32);
      const auto iy = static_cast<std::int32_t>(item.first & 0xffffffffLL);
      pcl::PointXYZ p;
      p.x = static_cast<float>((ix + 0.5) * height_resolution_);
      p.y = static_cast<float>((iy + 0.5) * height_resolution_);
      p.z = item.second;
      output->push_back(p);
    }
    output->width = static_cast<std::uint32_t>(output->size());
    output->height = 1;
    output->is_dense = true;
    return output;
  }

  void saveVendorArtifacts(const std::filesystem::path & root, const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud)
  {
    std::error_code ec;
    const auto blocks = root / ".blocks";
    const auto session = root / ".sessions" / "session_0";
    const auto lidar_dir = session / "lidar_cloud";
    const auto optimizer = root / ".optimizers" / "optimizer_0";
    std::filesystem::create_directories(blocks, ec);
    std::filesystem::create_directories(lidar_dir, ec);
    std::filesystem::create_directories(optimizer, ec);
    std::ofstream info(blocks / "info.txt");
    info << "format=m20pro-reconstructed-blocks-v1\nresolution=" << lio_params_.voxel_size << "\n";
    std::ofstream chunk(blocks / "000000.chunk", std::ios::binary);
    const uint64_t n = cloud ? cloud->size() : 0;
    chunk.write(reinterpret_cast<const char *>(&n), sizeof(n));
    if (cloud && !cloud->empty()) chunk.write(reinterpret_cast<const char *>(cloud->points.data()), cloud->size() * sizeof(pcl::PointXYZI));
    std::lock_guard<std::mutex> lock(map_keyframes_mutex_);
    for (const auto & keyframe : map_keyframes_) {
      if (!keyframe.cloud) continue;
      pcl::PointCloud<pcl::PointXYZINormal> out;
      out.reserve(keyframe.cloud->size());
      for (const auto & p : keyframe.cloud->points) { pcl::PointXYZINormal q{}; q.x=p.x; q.y=p.y; q.z=p.z; q.intensity=p.intensity; out.push_back(q); }
      pcl::io::savePCDFileBinary((lidar_dir / (std::to_string(keyframe.frame_id) + ".pcd")).string(), out);
    }
    std::ofstream loops(optimizer / "loops.txt");
    std::ofstream priors(optimizer / "priors.txt");
    if (!lio_trajectory_.empty()) {
      const auto & first = lio_trajectory_.front();
      priors << first.frame_id << " 0 0 0 0 0 0 1\n";
    }
    {
      std::lock_guard<std::mutex> loop_lock(accepted_loops_mutex_);
      if (accepted_loops_.empty()) {
        loops << "# no accepted loop constraints\n";
      } else {
        loops << std::fixed << std::setprecision(9);
        for (const auto & loop : accepted_loops_) {
          loops << loop.src_frame << ' ' << loop.tgt_frame << ' '
                << loop.relative_pose.t.x() << ' ' << loop.relative_pose.t.y() << ' '
                << loop.relative_pose.t.z() << ' ' << loop.relative_pose.q.x() << ' '
                << loop.relative_pose.q.y() << ' ' << loop.relative_pose.q.z() << ' '
                << loop.relative_pose.q.w() << ' ' << loop.fitness_score << '\n';
        }
      }
    }
    std::ofstream imu_quat(session / "imu_quat.txt");
    for (const auto & item : lio_trajectory_) imu_quat << item.pose.q.x() << ' ' << item.pose.q.y() << ' ' << item.pose.q.z() << ' ' << item.pose.q.w() << '\n';

    const auto grid = root / "occ_grid.pgm";
    const int size = 256;
    std::vector<unsigned char> pixels(static_cast<std::size_t>(size * size), 255);
    if (cloud) for (const auto & p : cloud->points) {
      if (p.z < occ_min_height_ || p.z > occ_max_height_) continue;
      const int x = std::clamp(static_cast<int>(p.x / occ_resolution_) + size/2, 0, size-1);
      const int y = std::clamp(static_cast<int>(p.y / occ_resolution_) + size/2, 0, size-1);
      pixels[static_cast<std::size_t>((size-1-y)*size+x)] = 0;
    }
    std::ofstream pgm(grid, std::ios::binary); pgm << "P5\n" << size << ' ' << size << "\n255\n"; pgm.write(reinterpret_cast<const char *>(pixels.data()), pixels.size());
    std::ofstream yaml(root / "occ_grid.yaml"); yaml << "image: occ_grid.pgm\nresolution: " << occ_resolution_ << "\norigin: [-" << size*occ_resolution_/2 << ", -" << size*occ_resolution_/2 << ", 0]\noccupied_thresh: 0.65\nfree_thresh: 0.196\nnegate: 0\n";
    std::ofstream toml(root / "occ_grid_id_map.toml"); toml << "format = \"m20pro-reconstructed\"\nsize = " << size << "\n";
  }

  void publishMapCloud()
  {
    const auto policy = ros::vendorAuxiliaryOutputPolicy(
      voxel_map_ray_casting_, accumulated_points_enabled_);
    // The vendor pubVoxelMap callback emits PointXYZ in map and leaves the
    // PointCloud2 timestamp at its default zero value.
    if (policy.publish_depth_cloud && voxel_cloud_pub_ && voxel_cloud_pub_->is_activated()) {
      const auto voxel_cloud = makeVoxelDebugCloud();
      if (!voxel_cloud->empty()) {
        sensor_msgs::msg::PointCloud2 message;
        pcl::toROSMsg(*voxel_cloud, message);
        message.header.frame_id = map_frame_;
        voxel_cloud_pub_->publish(message);
      }
    }

    // The accumulated-points channel is independently gated in the native
    // params and publishes a robot-local area crop in base_link. It is not an
    // alias for the global voxel debug cloud.
    if (policy.publish_accumulated_cloud && map_cloud_pub_ && map_cloud_pub_->is_activated()) {
      const auto accumulated_cloud = makeAccumulatedLocalCloud();
      sensor_msgs::msg::PointCloud2 message;
      pcl::toROSMsg(*accumulated_cloud, message);
      message.header.frame_id = body_frame_;
      map_cloud_pub_->publish(message);
    }

    if (depth_image_enabled_ && depth_image_pub_) {
      auto image = makeDepthImage();
      image.header.frame_id = body_frame_;
      depth_image_pub_->publish(image);
    }
    if (height_map_enabled_) {
      auto cloud = makeHeightCloud();
      if (height_cloud_pub_) {
        sensor_msgs::msg::PointCloud2 message;
        pcl::toROSMsg(*cloud, message);
        message.header.frame_id = body_frame_;
        height_cloud_pub_->publish(message);
      }
      if (height_image_pub_) {
        sensor_msgs::msg::Image image;
        image.encoding = "32FC1";
        image.width = static_cast<uint32_t>(std::max(1, height_size_y_));
        image.height = static_cast<uint32_t>(std::max(1, height_size_x_));
        image.step = image.width * sizeof(float);
        image.data.assign(static_cast<std::size_t>(image.step) * image.height, 0);
        const int cx = static_cast<int>(image.width) / 2;
        const int cy = static_cast<int>(image.height) / 2;
        auto * pix = reinterpret_cast<float *>(image.data.data());
        for (const auto & p : cloud->points) {
          const int x = cx + static_cast<int>(std::floor(p.x / height_resolution_));
          const int y = cy + static_cast<int>(std::floor(p.y / height_resolution_));
          if (x < 0 || y < 0 || x >= static_cast<int>(image.width) || y >= static_cast<int>(image.height)) continue;
          pix[y * image.width + x] = p.z;
        }
        image.header.frame_id = body_frame_;
        height_image_pub_->publish(image);
      }
    }
  }

  bool saveMap(std::string & message)
  {
    std::lock_guard<std::mutex> lock(save_mutex_);
    // Drain the asynchronous LIO worker before taking the final keyframe and
    // map snapshot.  Offline bag playback can finish while registration is
    // still processing its tail; saving immediately would silently omit those
    // scans (and produce a shorter trajectory).  Keep admitting input while
    // waiting so callbacks already queued by DDS are not discarded.
    if (lio_odom_ && !lio_odom_->waitUntilIdle(std::chrono::seconds(120))) {
      RCLCPP_WARN(
        get_logger(), "Timed out waiting for LIO queue to drain (remaining=%zu)",
        lio_odom_->lidarQueueSize());
    }
    // Stop admitting new scans only while the current voxel-map snapshot is
    // copied.  VoxelMap::getAllVoxels() already holds the map's shared lock.
    accept_lidar_ = false;
    // The end-of-bag callback is not guaranteed to create a final keyframe.
    // Run one bounded geometric check between the first and last stored
    // keyframes before the final iSAM2 update.
    (void)tryTerminalLoopClosure();
    if (pose_graph_) {
      pose_graph_->optimize();
    }
    const auto cloud = makeMapCloud();
    if (cloud->empty()) {
      message = "map is empty; no accepted LiDAR keyframe has initialized the voxel map";
      accept_lidar_ = true;
      return false;
    }

    const std::filesystem::path output_path(map_save_path_);
    std::error_code error;
    if (output_path.has_parent_path()) {
      std::filesystem::create_directories(output_path.parent_path(), error);
      if (error) {
        message = "cannot create map directory: " + error.message();
        accept_lidar_ = true;
        return false;
      }
    }
    if (pcl::io::savePCDFileBinary(output_path.string(), *cloud) != 0) {
      message = "PCL failed to write " + output_path.string();
      accept_lidar_ = true;
      return false;
    }

    const auto metadata_path = output_path.parent_path() / "mapping_summary.txt";
    std::string lidar_frame;
    std::string imu_frame;
    {
      std::lock_guard<std::mutex> timestamp_lock(timestamp_mutex_);
      lidar_frame = lidar_frame_id_;
      imu_frame = imu_frame_id_;
    }
    std::ofstream metadata(metadata_path);
    metadata << "map_frame=" << map_frame_ << '\n';
    metadata << "lidar_topic=" << lidar_topic_ << '\n';
    metadata << "lidar_transport=" << lidar_transport_ << '\n';
    metadata << "lidar_frame=" << lidar_frame << '\n';
    metadata << "imu_topic=" << imu_topic_ << '\n';
    metadata << "imu_transport=" << imu_transport_ << '\n';
    metadata << "imu_frame=" << imu_frame << '\n';
    metadata << "aligned_cloud_topic=" << aligned_cloud_topic_ << '\n';
    metadata << "body_cloud_topic=" << body_cloud_topic_ << '\n';
    metadata << "depth_cloud_topic=" << voxel_cloud_topic_ << '\n';
    metadata << "accumulated_cloud_topic=" << map_cloud_topic_ << '\n';
    metadata << "body_frame=" << body_frame_ << '\n';
    metadata << "accepted_clouds=" << accepted_clouds_.load() << '\n';
    metadata << "accepted_imus=" << accepted_imus_.load() << '\n';
    metadata << "processed_clouds=" << lio_odom_->processedScans() << '\n';
    metadata << "initialization_wait_clouds=" << lio_odom_->initializationWaitScans() << '\n';
    metadata << "bootstrap_clouds=" << lio_odom_->bootstrapScans() << '\n';
    metadata << "successful_lio_updates=" << lio_odom_->successfulUpdates() << '\n';
    metadata << "rejected_lio_updates=" << lio_odom_->rejectedUpdates() << '\n';
    metadata << "empty_registration_clouds=" << lio_odom_->emptyRegistrationClouds() << '\n';
    metadata << "dropped_clouds=" << lio_odom_->droppedScans() << '\n';
    metadata << "dropped_imus=" << dropped_imus_.load() << '\n';
    metadata << "keyframes=" << keyframe_count_.load() << '\n';
    metadata << "map_points=" << cloud->size() << '\n';
    metadata << "point_stride=" << lio_params_.point_stride << '\n';
    metadata << "downsample_leaf_size=" << lio_params_.downsample_leaf_size << '\n';
    metadata << "body_leaf_size=" << lio_params_.leaf_size_body << '\n';
    metadata << "voxel_size=" << lio_params_.voxel_size << '\n';
    metadata << "lio_save_full_pcd=" << save_full_pcd_ << '\n';
    metadata << "lio_full_map_save_path=" << full_map_save_path_ << '\n';
    metadata << "voxel_map_ray_casting=" << voxel_map_ray_casting_ << '\n';
    metadata << "accumulated_points_enabled=" << accumulated_points_enabled_ << '\n';
    metadata << "init_time=" << lio_params_.init_time << '\n';
    metadata << "imu_init_samples=" << lio_params_.imu_init_samples << '\n';
    const auto write_transform = [&metadata](const char * name, const SE3Pose & transform) {
        const auto matrix = transform.matrix();
        metadata << name << '=';
        for (int row = 0; row < 4; ++row) {
          for (int column = 0; column < 4; ++column) {
            if (row != 0 || column != 0) metadata << ',';
            metadata << matrix(row, column);
          }
        }
        metadata << '\n';
      };
    write_transform("extrinsic_B_I", sensor_params_.T_body_imu);
    write_transform("extrinsic_B_L", sensor_params_.T_body_lidar);
    write_transform("extrinsic_I_L", sensor_params_.T_lidar_imu);
    const auto accel_bias = lio_odom_->getAccelBias();
    const auto gyro_bias = lio_odom_->getGyroBias();
    metadata << "accel_bias=" << accel_bias.x() << ',' << accel_bias.y() << ','
             << accel_bias.z() << '\n';
    metadata << "gyro_bias=" << gyro_bias.x() << ',' << gyro_bias.y() << ','
             << gyro_bias.z() << '\n';

    // Native poses.txt is a session/keyframe trajectory. Keep the same
    // semantic in trajectory.csv so path length is comparable to the vendor
    // result; preserve every high-rate LIO sample in a separate diagnostic
    // file instead of mixing scan jitter into the acceptance metric.
    const auto trajectory_path = output_path.parent_path() / "trajectory.csv";
    std::ofstream trajectory_file(trajectory_path);
    trajectory_file << "frame_id,x,y,z,qw,qx,qy,qz,stamp_ns\n";
    const auto lio_trajectory_path = output_path.parent_path() / "lio_trajectory.csv";
    std::ofstream lio_trajectory_file(lio_trajectory_path);
    lio_trajectory_file << "frame_id,x,y,z,qw,qx,qy,qz,stamp_ns\n";
    {
      std::lock_guard<std::mutex> trajectory_lock(lio_trajectory_mutex_);
      for (const auto & item : lio_trajectory_) {
        lio_trajectory_file << item.frame_id << ',' << item.pose.t.x() << ',' << item.pose.t.y() << ','
                            << item.pose.t.z() << ',' << item.pose.q.w() << ',' << item.pose.q.x()
                            << ',' << item.pose.q.y() << ',' << item.pose.q.z() << ','
                            << std::chrono::duration_cast<std::chrono::nanoseconds>(
                                 item.stamp.time_since_epoch()).count() << '\n';
      }
    }
    // Native session pose artifacts contain optimized keyframes rather than
    // every high-rate LIO sample: stamp tx ty tz qx qy qz qw.
    const auto session_path = output_path.parent_path() / ".sessions" / "session_0";
    std::filesystem::create_directories(session_path, error);
    if (!error) {
      std::ofstream lio_pose_file(session_path / "lio_odom.pose");
      std::ofstream poses_file(session_path / "poses.txt");
      lio_pose_file << std::fixed << std::setprecision(9);
      poses_file << std::fixed << std::setprecision(9);
      std::lock_guard<std::mutex> keyframe_lock(map_keyframes_mutex_);
      for (const auto & item : map_keyframes_) {
        const double stamp_seconds = static_cast<double>(
          std::chrono::duration_cast<std::chrono::nanoseconds>(
            item.stamp.time_since_epoch()).count()) * 1e-9;
        const auto write_pose = [&](std::ofstream & stream) {
            stream << stamp_seconds << ' ' << item.pose.t.x() << ' '
                   << item.pose.t.y() << ' ' << item.pose.t.z() << ' '
                   << item.pose.q.x() << ' ' << item.pose.q.y() << ' '
                   << item.pose.q.z() << ' ' << item.pose.q.w() << '\n';
          };
        write_pose(lio_pose_file);
        write_pose(poses_file);
        trajectory_file << item.frame_id << ',' << item.pose.t.x() << ',' << item.pose.t.y() << ','
                        << item.pose.t.z() << ',' << item.pose.q.w() << ',' << item.pose.q.x()
                        << ',' << item.pose.q.y() << ',' << item.pose.q.z() << ','
                        << std::chrono::duration_cast<std::chrono::nanoseconds>(
                             item.stamp.time_since_epoch()).count() << '\n';
      }
    }
    {
      std::lock_guard<std::mutex> keyframe_lock(map_keyframes_mutex_);
      Scalar keyframe_length = 0.0;
      Scalar keyframe_closure = 0.0;
      if (map_keyframes_.size() >= 2U) {
        for (std::size_t i = 1; i < map_keyframes_.size(); ++i) {
          keyframe_length += (map_keyframes_[i].pose.t -
            map_keyframes_[i - 1U].pose.t).norm();
        }
        keyframe_closure = (map_keyframes_.back().pose.t -
          map_keyframes_.front().pose.t).norm();
      }
      metadata << "keyframe_trajectory_length_m=" << keyframe_length << '\n';
      metadata << "keyframe_trajectory_closure_m=" << keyframe_closure << '\n';
    }
    saveVendorArtifacts(output_path.parent_path(), cloud);
    message = output_path.string() + " (" + std::to_string(cloud->size()) + " points)";
    RCLCPP_INFO(get_logger(), "Saved map: %s", message.c_str());
    accept_lidar_ = true;
    return true;
  }

  void stopWorker()
  {
    if (lio_odom_) {
      lio_odom_->stop();
    }
    if (lio_thread_ && lio_thread_->joinable()) {
      lio_thread_->join();
    }
    lio_thread_.reset();
  }

  SensorParams sensor_params_;
  LIOParams lio_params_;
  BackendParams backend_params_;
  std::shared_ptr<lio::LIOOdometry> lio_odom_;
  std::shared_ptr<backend::PoseGraphOptimizer> pose_graph_;
  std::unique_ptr<std::thread> lio_thread_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  std::unique_ptr<ros::SocketPointCloudSource> drdds_lidar_source_;
  std::unique_ptr<ros::SocketImuSource> drdds_imu_source_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    aligned_cloud_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    body_cloud_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr depth_image_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr height_image_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr height_cloud_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    voxel_cloud_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_cloud_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_map_service_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr opt_timer_;
  rclcpp::TimerBase::SharedPtr input_status_timer_;
  rclcpp::TimerBase::SharedPtr checkpoint_timer_;
  rclcpp::TimerBase::SharedPtr autostart_timer_;

  std::string lidar_topic_;
  std::string lidar_transport_;
  ros::DrddsPointCloudSourceOptions drdds_options_;
  std::string drdds_socket_path_;
  std::string imu_topic_;
  std::string imu_transport_;
  std::string drdds_imu_socket_path_;
  std::string odom_topic_;
  std::string path_topic_;
  std::string aligned_cloud_topic_;
  std::string body_cloud_topic_;
  std::string voxel_cloud_topic_;
  std::string map_cloud_topic_;
  std::string depth_image_topic_{"/m20_slam/DEPTH_IMAGE"};
  std::string height_image_topic_{"/m20_slam/HEIGHT_IMAGE"};
  std::string height_cloud_topic_{"/m20_slam/HEIGHT_POINTS"};
  std::string map_frame_;
  std::string tracking_frame_;
  std::string body_frame_;
  std::string map_save_path_;
  bool save_full_pcd_{false};
  std::string full_map_save_path_;
  bool publish_tf_{true};
  bool auto_save_on_shutdown_{true};
  double checkpoint_save_period_s_{10.0};
  int publish_map_every_n_keyframes_{5};
  std::int64_t max_timestamp_rollback_ns_{20000000LL};
  bool accumulated_points_enabled_{false};
  int accumulated_points_level_{3};
  Eigen::Matrix<Scalar, 3, 1> accumulated_area_min_{-5.0, -5.0, -1.0};
  Eigen::Matrix<Scalar, 3, 1> accumulated_area_max_{5.0, 5.0, 1.0};
  bool accumulated_udp_enabled_{false};
  double accumulated_udp_resolution_{0.1};
  int accumulated_udp_port_{30100};
  bool voxel_map_ray_casting_{false};
  int voxel_map_ray_casting_level_{3};
  double voxel_map_ray_casting_range_{3.0};
  double voxel_map_ray_distance_threshold_{0.02};
  bool depth_image_enabled_{false};
  bool height_map_enabled_{false};
  double depth_image_max_depth_{3.0};
  double depth_image_hres_deg_{4.0};
  double depth_image_vres_deg_{4.0};
  double height_resolution_{0.04};
  int height_size_x_{100};
  int height_size_y_{100};
  double height_min_z_{-1.0};
  double height_max_z_{1.0};
  std::string occ_prefix_{"occ_grid"};
  double occ_min_height_{-0.2};
  double occ_max_height_{0.4};
  double occ_resolution_{0.1};

  nav_msgs::msg::Path path_;
  struct LioTrajectorySample
  {
    FrameId frame_id;
    SE3Pose pose;
    Timestamp stamp;
  };
  std::vector<LioTrajectorySample> lio_trajectory_;
  std::mutex lio_trajectory_mutex_;
  struct StoredMapKeyframe
  {
    FrameId frame_id;
    SE3Pose pose;
    Timestamp stamp;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
  };
  std::vector<StoredMapKeyframe> map_keyframes_;
  mutable std::mutex map_keyframes_mutex_;
  bool have_previous_keyframe_{false};
  SE3Pose previous_keyframe_pose_;
  FrameId previous_keyframe_id_{0};
  Eigen::Matrix<Scalar, 6, 6> latest_lio_covariance_{
    Eigen::Matrix<Scalar, 6, 6>::Identity()};
  Timestamp latest_lio_stamp_{};
  std::vector<LoopCandidate> accepted_loops_;
  std::mutex accepted_loops_mutex_;
  std::atomic<std::int64_t> last_lidar_stamp_ns_{-1};
  std::atomic<std::int64_t> last_lidar_scan_end_ns_{-1};
  std::atomic<std::int64_t> last_imu_stamp_ns_{-1};
  std::string lidar_frame_id_;
  std::string imu_frame_id_;
  bool lidar_contract_logged_{false};
  bool imu_contract_logged_{false};
  std::atomic<std::uint64_t> accepted_clouds_{0};
  std::atomic<std::uint64_t> accepted_imus_{0};
  std::atomic<std::uint64_t> dropped_imus_{0};
  std::atomic<std::uint64_t> keyframe_count_{0};
  std::atomic<bool> accept_lidar_{true};
  std::mutex timestamp_mutex_;
  std::mutex save_mutex_;
};

}  // namespace m20::nodes

RCLCPP_COMPONENTS_REGISTER_NODE(m20::nodes::SlamNode)
