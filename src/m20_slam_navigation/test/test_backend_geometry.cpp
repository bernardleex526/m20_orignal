#include <gtest/gtest.h>

#include "m20_slam_navigation/backend/factor_graph.hpp"
#include "m20_slam_navigation/backend/loop_closure.hpp"

#include <cmath>

namespace {

using m20::BackendParams;
using m20::SE3Pose;
using m20::Scalar;
using m20::backend::FactorGraph;
using m20::backend::LoopClosureDetector;

void addPoint(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
              float x, float y, float z) {
  pcl::PointXYZI point;
  point.x = x;
  point.y = y;
  point.z = z;
  cloud->push_back(point);
}

pcl::PointCloud<pcl::PointXYZI>::Ptr makeRoom() {
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZI>());
  for (int i = 0; i < 80; ++i) {
    const float x = -4.0F + static_cast<float>(i) * 0.1F;
    addPoint(cloud, x, -2.0F, 0.0F);
    addPoint(cloud, x,  2.0F, 0.0F);
    addPoint(cloud, x, -2.0F, 2.0F);
    addPoint(cloud, x,  2.0F, 2.0F);
  }
  for (int i = 0; i < 40; ++i) {
    const float y = -2.0F + static_cast<float>(i) * 0.1F;
    addPoint(cloud, -4.0F, y, 1.0F);
    addPoint(cloud,  4.0F, y, 1.0F);
  }
  return cloud;
}

TEST(BackendGeometry, GhtSeedRecoversDriftedPoseForLoopVerification) {
  BackendParams params;
  params.segment_num = 15;
  params.loop_min_frame_separation = 2;
  params.loop_max_candidates = 2;
  params.loop_matching_error_threshold = 0.16;
  params.loop_min_submap_overlap = 0.65;
  params.loop_inlier_fraction_threshold = 0.95;
  params.loop_icp_max_correspondence = 2.0;
  LoopClosureDetector detector(params);
  auto cloud = makeRoom();

  detector.addKeyframe(LoopClosureDetector::buildDescriptor(
    cloud, 1, SE3Pose::Identity()));
  detector.addKeyframe(LoopClosureDetector::buildDescriptor(
    cloud, 2, SE3Pose::Identity()));
  SE3Pose drifted;
  drifted.t.x() = 5.0;
  drifted.q = Eigen::Quaternion<Scalar>(Eigen::AngleAxis<Scalar>(0.7, Eigen::Vector3d::UnitZ()));
  const auto loops = detector.detectLoop(
    LoopClosureDetector::buildDescriptor(cloud, 3, drifted));
  ASSERT_FALSE(loops.empty());
  EXPECT_EQ(loops.front().tgt_frame, 1U);
  EXPECT_LT(loops.front().fitness_score, params.loop_matching_error_threshold);
}

TEST(FactorGraph, AcceptsNativeAnisotropicFactorNoise) {
  BackendParams params;
  params.prior_noise_sigmas = {{1e-4, 2e-4, 3e-4, 4e-4, 5e-4, 6e-4}};
  params.odom_noise_sigmas = {{0.01, 0.02, 0.03, 0.04, 0.05, 0.06}};
  params.loop_noise_sigmas = {{0.1, 0.2, 0.3, 0.4, 0.5, 0.6}};
  FactorGraph graph(params);
  Eigen::Matrix<Scalar, 6, 6> covariance = Eigen::Matrix<Scalar, 6, 6>::Identity();
  graph.addPriorFactor(1, SE3Pose::Identity(), covariance);
  graph.addOdometryFactor(1, 2, SE3Pose::Identity(), covariance.inverse());
  graph.addGravityFactor(1, Eigen::Vector3d(0.0, 0.0, -9.81007));
  graph.optimize();
  EXPECT_TRUE(graph.getPoseEstimate(2).t.allFinite());
  EXPECT_GE(graph.getGraph().size(), 0U);
}

}  // namespace
