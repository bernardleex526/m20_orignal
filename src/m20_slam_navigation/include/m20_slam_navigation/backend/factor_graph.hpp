#pragma once
/**
 * @file factor_graph.hpp
 * @brief GTSAM-based factor graph manager for back-end optimization.
 *
 * Manages the iSAM2 incremental non-linear optimizer with factor types:
 *  - BetweenFactor<Pose3>: LIO odometry relative pose constraints.
 *  - PriorFactor<Pose3>:  Prior map relocalization constraints.
 *  - Custom GravityFactor: Roll/pitch constraint from IMU gravity direction.
 *  - BetweenFactor<Pose3>: Loop closure constraints from ScanContext matching.
 *
 * Uses GTSAM's ISAM2 with configurable relinearization and wildfire threshold.
 */

#include "m20_slam_navigation/common/types.hpp"
#include "m20_slam_navigation/common/params.hpp"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <Eigen/Dense>

#include <memory>
#include <mutex>
#include <vector>

namespace m20::backend {

/// Convert our SE3Pose to GTSAM Pose3
inline gtsam::Pose3 toGtsam(const SE3Pose& pose) {
  return gtsam::Pose3(
      gtsam::Rot3(pose.q.cast<double>().matrix()),
      gtsam::Point3(pose.t.x(), pose.t.y(), pose.t.z()));
}

/// Convert GTSAM Pose3 to our SE3Pose
inline SE3Pose fromGtsam(const gtsam::Pose3& pose) {
  SE3Pose out;
  Eigen::Matrix<double, 3, 3> R = pose.rotation().matrix();
  out.q = Eigen::Quaternion<Scalar>(R.cast<Scalar>());
  out.t = pose.translation().matrix().cast<Scalar>();
  return out;
}

/// GTSAM symbol shorthand: X(frame_id)
inline gtsam::Symbol X(FrameId id) { return gtsam::Symbol('x', id); }

/**
 * @brief Gravity prior factor: constrains roll and pitch by penalizing
 *        deviation of estimated gravity from the known global gravity vector.
 *
 * Since the global gravity direction is [0, 0, -g] (z-up world frame),
 * the body-frame gravity measurement g_b = R_wbᵀ · [0, 0, -g] should equal
 * the measured acceleration (minus motion acceleration) at low dynamics.
 *
 * This factor essentially says: "the roll and pitch of R_wb must be consistent
 * with the measured gravity direction, so only (x, y, z, yaw) are free."
 */
class GravityPriorFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
public:
  GravityPriorFactor(gtsam::Key key,
                     const Eigen::Matrix<double, 3, 1>& gravity_body,
                     const gtsam::SharedNoiseModel& noise_model)
      : NoiseModelFactor1<gtsam::Pose3>(noise_model, key),
        gravity_body_(gravity_body) {}

  gtsam::Vector evaluateError(
      const gtsam::Pose3& T_wb,
      boost::optional<gtsam::Matrix&> H = boost::none) const override;

private:
  Eigen::Matrix<double, 3, 1> gravity_body_;
};

class FactorGraph {
public:
  explicit FactorGraph(const BackendParams& params);

  /// Add LIO odometry factor between consecutive frames
  void addOdometryFactor(FrameId prev_id, FrameId curr_id,
                         const SE3Pose& relative_pose,
                         const Eigen::Matrix<Scalar, 6, 6>& information);

  /// Add gravity prior factor for a frame
  void addGravityFactor(FrameId frame_id,
                        const Eigen::Matrix<Scalar, 3, 1>& gravity_body);

  /// Add loop closure factor
  void addLoopClosureFactor(FrameId src_id, FrameId tgt_id,
                            const SE3Pose& relative_pose);

  /// Add prior factor (from relocalization or initial pose)
  void addPriorFactor(FrameId frame_id, const SE3Pose& pose,
                      const Eigen::Matrix<Scalar, 6, 6>& covariance);

  /// Run incremental optimization (iSAM2 update)
  void optimize();

  /// Get optimized pose for a frame
  SE3Pose getPoseEstimate(FrameId frame_id) const;

  /// Get all optimized poses
  std::vector<std::pair<FrameId, SE3Pose>> getAllPoses() const;

  /// Get mutable iSAM2 instance
  gtsam::ISAM2& getISAM2() { return isam2_; }

  /// Clear all factors and reinitialize
  void reset();

  /// Get the current nonlinear factor graph (for degeneracy analysis)
  const gtsam::NonlinearFactorGraph& getGraph() const { return graph_; }
  const gtsam::Values& getInitialValues() const { return initial_values_; }

private:
  BackendParams                      params_;
  gtsam::NonlinearFactorGraph        graph_;
  gtsam::Values                      initial_values_;
  gtsam::ISAM2                       isam2_;
  gtsam::Values                      result_;
  std::mutex                         mutex_;
  bool                               graph_modified_{false};
};

}  // namespace m20::backend