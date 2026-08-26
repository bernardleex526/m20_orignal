#include "m20_slam_navigation/backend/factor_graph.hpp"

#include <gtsam/nonlinear/ISAM2Params.h>

#include <cmath>

namespace m20::backend {

// =============================================================================
// GravityPriorFactor: error = R_wbᵀ · g_world − g_measured_body
// =============================================================================
gtsam::Vector GravityPriorFactor::evaluateError(
    const gtsam::Pose3& T_wb,
    boost::optional<gtsam::Matrix&> H_opt) const {

  // g_world = [0, 0, −9.81] (z-up)
  Eigen::Vector3d g_world(0.0, 0.0, -9.81007);

  // R_wb is world-to-body rotation (inverse of body-to-world)
  // g_body_pred = R_wbᵀ · g_world = R_wb.inverse() · g_world
  gtsam::Rot3 R_wb = T_wb.rotation();
  gtsam::Point3 g_body_pred = R_wb.unrotate(gtsam::Point3(g_world));

  // Error: predicted - measured (both in body frame)
  gtsam::Vector3 error = g_body_pred - gtsam::Point3(gravity_body_);

  if (H_opt) {
    // Jacobian w.r.t. T_wb = [R_wb, t_wb]
    // ∂g_body/∂R = ∂(R⁻¹·g_world)/∂R = [R⁻¹·g_world]×
    // ∂g_body/∂t = 0
    Eigen::Matrix3d G;
    G << 0,                      -g_body_pred.z(),  g_body_pred.y(),
         g_body_pred.z(),   0,                    -g_body_pred.x(),
        -g_body_pred.y(),   g_body_pred.x(),   0;

    *H_opt = (gtsam::Matrix36() << G, Eigen::Matrix3d::Zero()).finished();
  }

  return error;
}

// =============================================================================
// FactorGraph
// =============================================================================
FactorGraph::FactorGraph(const BackendParams& params)
    : params_(params) {

  gtsam::ISAM2Params isam2_params;
  isam2_params.relinearizeSkip = params.isam2_relinearize_skip;
  isam2_params.setRelinearizeThreshold(params.isam2_wildfire_threshold);
  // Enable smart factors for better performance
  isam2_params.enablePartialRelinearizationCheck = true;

  isam2_ = gtsam::ISAM2(isam2_params);
}

void FactorGraph::addOdometryFactor(
    FrameId prev_id, FrameId curr_id,
    const SE3Pose& relative_pose,
    const Eigen::Matrix<Scalar, 6, 6>& information) {

  std::lock_guard<std::mutex> lock(mutex_);

  // Noise model from information matrix
  Eigen::Matrix<double, 6, 6> info_double = information.cast<double>();
  gtsam::noiseModel::Gaussian::shared_ptr noise =
      gtsam::noiseModel::Gaussian::Information(info_double);

  // Between factor
  graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
      X(prev_id), X(curr_id), toGtsam(relative_pose), noise));

  // Initialize new frame pose if needed
  if (!initial_values_.exists(X(curr_id))) {
    gtsam::Pose3 prev_pose;
    if (initial_values_.exists(X(prev_id))) {
      prev_pose = initial_values_.at<gtsam::Pose3>(X(prev_id));
    } else {
      prev_pose = gtsam::Pose3::Identity();
    }
    initial_values_.insert(X(curr_id), prev_pose.compose(toGtsam(relative_pose)));
  }

  graph_modified_ = true;
}

void FactorGraph::addGravityFactor(
    FrameId frame_id,
    const Eigen::Matrix<Scalar, 3, 1>& gravity_body) {

  std::lock_guard<std::mutex> lock(mutex_);

  // Noise: diagonal with gravity_noise_sigma
  gtsam::noiseModel::Diagonal::shared_ptr noise =
      gtsam::noiseModel::Diagonal::Sigmas(
          Eigen::Vector3d::Constant(params_.gravity_noise_sigma));

  graph_.add(GravityPriorFactor(
      X(frame_id), gravity_body.cast<double>(), noise));

  graph_modified_ = true;
}

void FactorGraph::addLoopClosureFactor(
    FrameId src_id, FrameId tgt_id,
    const SE3Pose& relative_pose) {

  std::lock_guard<std::mutex> lock(mutex_);

  // Loop closure noise (typically larger than odometry)
  Eigen::Matrix<double, 6, 6> cov = Eigen::Matrix<double, 6, 6>::Identity();
  cov.block<3, 3>(0, 0) *= params_.loop_closure_noise_rot * params_.loop_closure_noise_rot;
  cov.block<3, 3>(3, 3) *= params_.loop_closure_noise_trans * params_.loop_closure_noise_trans;

  gtsam::noiseModel::Gaussian::shared_ptr noise =
      gtsam::noiseModel::Gaussian::Covariance(cov);

  graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
      X(src_id), X(tgt_id), toGtsam(relative_pose), noise));

  graph_modified_ = true;
}

void FactorGraph::addPriorFactor(
    FrameId frame_id, const SE3Pose& pose,
    const Eigen::Matrix<Scalar, 6, 6>& covariance) {

  std::lock_guard<std::mutex> lock(mutex_);

  gtsam::noiseModel::Gaussian::shared_ptr noise =
      gtsam::noiseModel::Gaussian::Covariance(covariance.cast<double>());

  graph_.add(gtsam::PriorFactor<gtsam::Pose3>(
      X(frame_id), toGtsam(pose), noise));

  if (!initial_values_.exists(X(frame_id))) {
    initial_values_.insert(X(frame_id), toGtsam(pose));
  } else {
    initial_values_.update(X(frame_id), toGtsam(pose));
  }

  graph_modified_ = true;
}

void FactorGraph::optimize() {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!graph_modified_) return;

  try {
    // iSAM2 incremental update
    gtsam::ISAM2Result result = isam2_.update(graph_, initial_values_);

    // Get optimized values
    result_ = isam2_.calculateEstimate();

    // Reset for next batch
    graph_ = gtsam::NonlinearFactorGraph();
    initial_values_.clear();
    graph_modified_ = false;
  } catch (const std::exception& e) {
    // Optimization failed; keep previous estimate
    graph_modified_ = false;
  }
}

SE3Pose FactorGraph::getPoseEstimate(FrameId frame_id) const {
  std::lock_guard<std::mutex> lock(mutex_);

  if (result_.exists(X(frame_id))) {
    return fromGtsam(result_.at<gtsam::Pose3>(X(frame_id)));
  }
  if (initial_values_.exists(X(frame_id))) {
    return fromGtsam(initial_values_.at<gtsam::Pose3>(X(frame_id)));
  }
  return SE3Pose::Identity();
}

std::vector<std::pair<FrameId, SE3Pose>> FactorGraph::getAllPoses() const {
  std::lock_guard<std::mutex> lock(mutex_);

  std::vector<std::pair<FrameId, SE3Pose>> poses;
  for (const auto& key_value : result_) {
    FrameId id = gtsam::Symbol(key_value.key).index();
    SE3Pose pose = fromGtsam(key_value.value.cast<gtsam::Pose3>());
    poses.push_back({id, pose});
  }
  return poses;
}

void FactorGraph::reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  graph_ = gtsam::NonlinearFactorGraph();
  initial_values_.clear();
  gtsam::ISAM2Params isam2_params;
  isam2_params.relinearizeSkip = params_.isam2_relinearize_skip;
  isam2_ = gtsam::ISAM2(isam2_params);
  result_.clear();
  graph_modified_ = false;
}

}  // namespace m20::backend