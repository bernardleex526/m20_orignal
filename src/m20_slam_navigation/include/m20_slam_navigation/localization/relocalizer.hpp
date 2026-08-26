#pragma once
/**
 * @file relocalizer.hpp
 * @brief Global relocalization against prior map.
 *
 * Supports:
 *  - 2D/3D pose initialization via RViz `/initialpose`
 *  - Global multi-hypothesis NDT matching (kidnapped robot recovery)
 *  - Continuous pose tracking via ESKF + NDT
 */

#include "m20_slam_navigation/common/types.hpp"
#include "m20_slam_navigation/common/params.hpp"
#include "m20_slam_navigation/localization/ndt_matcher.hpp"
#include "m20_slam_navigation/localization/eskf.hpp"
#include "m20_slam_navigation/localization/prior_map_loader.hpp"

#include <functional>
#include <memory>
#include <mutex>

namespace m20::localization {

using RelocalizationCallback = std::function<void(const SE3Pose& pose, bool success)>;

class Relocalizer {
public:
  Relocalizer(const LocalizationParams& localization_params,
              const SensorParams& sensor_params);

  /// Load prior map
  bool loadMap(const std::string& map_path);

  /**
   * @brief Attempt global relocalization.
   *
   * @param cloud             Current LiDAR scan
   * @param initial_guess     Optional initial pose guess (from /initialpose or last known)
   * @return true if relocalization succeeded
   */
  bool relocalize(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                  const PoseWithCovariance& initial_guess);

  /**
   * @brief Update localization with new IMU data (ESKF predict).
   */
  void predict(const ImuPacket& imu);

  /**
   * @brief Update localization with foot odometry (ESKF update).
   */
  void updateOdometry(const FootOdomPacket& odom);

  /**
   * @brief Update localization with NDT matching (ESKF update).
   */
  void updateNDT(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

  /// Get current pose estimate
  PoseWithCovariance getCurrentPose() const;

  /// Set relocalization callback
  void setRelocalizationCallback(RelocalizationCallback cb) { reloc_cb_ = std::move(cb); }

  /// Check if localized
  bool isLocalized() const { return localized_; }

private:
  LocalizationParams              loc_params_;
  SensorParams                    sensor_params_;
  std::unique_ptr<PriorMapLoader> map_loader_;
  std::unique_ptr<NDTMatcher>     ndt_matcher_;
  std::unique_ptr<ESKF>           eskf_;

  RelocalizationCallback          reloc_cb_;
  mutable std::mutex              mutex_;
  bool                            localized_{false};
  SE3Pose                         last_ndt_pose_;  ///< for NDT initialization
};

}  // namespace m20::localization