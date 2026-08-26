#include "m20_slam_navigation/localization/ndt_matcher.hpp"
#include "m20_slam_navigation/common/math_utils.hpp"

#include <pcl/registration/ndt.h>

#include <chrono>
#include <random>

namespace m20::localization {

NDTMatcher::NDTMatcher(const LocalizationParams& params)
    : params_(params) {}

void NDTMatcher::setTargetMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& map) {
  target_map_ = map;
}

NDTResult NDTMatcher::align(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
    const SE3Pose& T_init) {

  NDTResult result;
  result.converged = false;
  result.fitness_score = 1.0;

  if (!target_map_ || target_map_->empty()) return result;

  // Set up PCL NDT
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
  ndt.setMaximumIterations(params_.ndt_max_iterations);
  ndt.setResolution(params_.ndt_resolution);
  ndt.setStepSize(params_.ndt_step_size);
  ndt.setTransformationEpsilon(params_.ndt_epsilon);
  ndt.setOulierRatio(params_.ndt_outlier_ratio);

  ndt.setInputTarget(target_map_);
  ndt.setInputSource(source);

  // Initial guess as Eigen 4x4
  Eigen::Matrix4f T_init_f = T_init.matrix().cast<float>();

  pcl::PointCloud<pcl::PointXYZ> output;
  ndt.align(output, T_init_f);

  if (ndt.hasConverged()) {
    Eigen::Matrix4f T_out = ndt.getFinalTransformation();
    result.T_world_lidar.q = Eigen::Quaternion<Scalar>(
        Eigen::Quaternionf(T_out.block<3, 3>(0, 0)).cast<Scalar>());
    result.T_world_lidar.t = T_out.block<3, 1>(0, 3).cast<Scalar>();
    result.fitness_score = ndt.getFitnessScore();
    result.transformation_probability = ndt.getTransformationProbability();
    result.iterations = ndt.getFinalNumIteration();
    result.converged = true;

    // Information from Hessian (approximate from fitness)
    Scalar info_scale = 1.0 / (ndt.getFitnessScore() + 0.01);
    result.information = Eigen::Matrix<Scalar, 6, 6>::Identity() * info_scale;
  }

  return result;
}

NDTResult NDTMatcher::globalRelocalize(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
    const Eigen::Matrix<Scalar, 3, 1>& search_center,
    Scalar search_radius_xy,
    int num_hypotheses) {

  NDTResult best_result;
  best_result.fitness_score = 1e9;
  best_result.converged = false;

  // Generate hypotheses: grid sampling within search radius
  std::mt19937 rng(42);  // fixed seed for reproducibility
  std::uniform_real_distribution<Scalar> dist_xy(-search_radius_xy, search_radius_xy);
  std::uniform_real_distribution<Scalar> dist_yaw(-math::kPI, math::kPI);

  for (int h = 0; h < num_hypotheses; ++h) {
    SE3Pose hypothesis;
    hypothesis.t.x() = search_center.x() + dist_xy(rng);
    hypothesis.t.y() = search_center.y() + dist_xy(rng);
    hypothesis.t.z() = search_center.z();  // keep z as given
    hypothesis.q = math::yaw_to_quaternion(dist_yaw(rng));

    NDTResult result = align(source, hypothesis);
    if (result.converged && result.fitness_score < best_result.fitness_score) {
      best_result = result;
    }
  }

  return best_result;
}

}  // namespace m20::localization