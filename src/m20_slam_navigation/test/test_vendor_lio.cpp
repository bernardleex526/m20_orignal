#include <gtest/gtest.h>

#include "m20_slam_navigation/lio/vendor_lio_eskf.hpp"

#include <chrono>

namespace {

using m20::ImuPacket;
using m20::LIOParams;
using m20::SE3Pose;
using m20::SensorParams;
using m20::Scalar;
using m20::composeVendorLidarInImuExtrinsic;
using m20::lio::VendorLioEskf;
using m20::lio::VoxelMap;

pcl::PointCloud<pcl::PointXYZI>::Ptr makePlane(Scalar z) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
  for (int ix = -30; ix <= 30; ++ix) {
    for (int iy = -30; iy <= 30; ++iy) {
      pcl::PointXYZI point;
      point.x = static_cast<float>(ix * 0.03);
      point.y = static_cast<float>(iy * 0.03);
      point.z = static_cast<float>(z);
      cloud->push_back(point);
    }
  }
  return cloud;
}

LIOParams vendorParams() {
  LIOParams params;
  params.voxel_size = 0.15;
  params.max_voxels = 100000;
  params.esti_plane_threshold = 0.1;
  params.deepest_level = 2;
  params.plane_level = 2;
  params.top_level = 1;
  params.max_iterations = 3;
  params.lidar_cov = 0.001;
  return params;
}

TEST(VendorVoxelMap, FindsHierarchicalPlane) {
  const auto params = vendorParams();
  auto map = std::make_shared<VoxelMap>(params);
  map->insertCloud(makePlane(0.0), SE3Pose::Identity());

  VoxelMap::PlaneMatch match;
  ASSERT_TRUE(map->findPlane(Eigen::Vector3d(0.02, -0.01, 0.05), match));
  EXPECT_NEAR(std::abs(match.normal.z()), 1.0, 1e-6);
  EXPECT_NEAR(std::abs(match.residual), 0.05, 1e-6);
  EXPECT_GE(match.support_points, 5U);
  EXPECT_GE(match.level, params.top_level);
  EXPECT_LE(match.level, params.plane_level);
}

TEST(VendorVoxelMap, UsesCenterAndSixFaceNeighboursOnly) {
  m20::LIOParams params;
  params.voxel_size = 0.16;
  params.deepest_level = 0;
  params.plane_level = 0;
  params.top_level = 0;
  params.esti_plane_threshold = 0.1;
  auto map = std::make_shared<m20::lio::VoxelMap>(params);

  pcl::PointCloud<pcl::PointXYZI>::Ptr diagonal_plane(
    new pcl::PointCloud<pcl::PointXYZI>());
  for (int x = 0; x < 8; ++x) {
    for (int y = 0; y < 8; ++y) {
      pcl::PointXYZI point;
      point.x = 0.18F + static_cast<float>(x) * 0.01F;
      point.y = 0.18F + static_cast<float>(y) * 0.01F;
      point.z = 0.0F;
      diagonal_plane->push_back(point);
    }
  }
  map->insertCloud(diagonal_plane, m20::SE3Pose::Identity());

  m20::lio::VoxelMap::PlaneMatch match;
  EXPECT_FALSE(map->findPlane(Eigen::Vector3d(0.02, 0.02, 0.02), match));
}

TEST(VendorExtrinsic, ComposesInverseBodyImuWithBodyLidar) {
  SE3Pose T_body_imu;
  T_body_imu.q = Eigen::Quaterniond(
    Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()));
  T_body_imu.t = Eigen::Vector3d(1.0, -2.0, 0.5);
  SE3Pose T_body_lidar;
  T_body_lidar.q = Eigen::Quaterniond(
    Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitY()));
  T_body_lidar.t = Eigen::Vector3d(-0.4, 0.3, 1.2);

  const auto T_imu_lidar = composeVendorLidarInImuExtrinsic(
    T_body_imu, T_body_lidar);
  const auto reconstructed = T_body_imu * T_imu_lidar;
  EXPECT_LT((reconstructed.t - T_body_lidar.t).norm(), 1e-12);
  EXPECT_LT(reconstructed.q.angularDistance(T_body_lidar.q), 1e-12);
}

TEST(VendorLioEskf, IteratedPointPlaneCorrectsHeight) {
  SensorParams sensors;
  sensors.T_lidar_imu = SE3Pose::Identity();
  const auto params = vendorParams();
  auto map = std::make_shared<VoxelMap>(params);
  map->insertCloud(makePlane(0.0), SE3Pose::Identity());

  VendorLioEskf filter(sensors, params);
  SE3Pose predicted;
  predicted.t.z() = 0.05;
  filter.initialize(predicted, Eigen::Vector3d::Zero(),
                    Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  const auto result = filter.update(makePlane(0.0), map);

  EXPECT_TRUE(result.converged);
  EXPECT_GT(result.correspondences, 100);
  EXPECT_LT(std::abs(filter.state().pose.t.z()), 0.02);
  EXPECT_TRUE(filter.state().covariance.allFinite());
}

TEST(VendorLioEskf, InitializationCovarianceMatchesVendorBlocks) {
  SensorParams sensors;
  const auto params = vendorParams();
  VendorLioEskf filter(sensors, params);
  filter.initialize(SE3Pose::Identity(), Eigen::Vector3d::Zero(),
                    Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

  const auto diagonal = filter.state().covariance.diagonal();
  EXPECT_EQ(filter.state().kErrorDim, 23);
  EXPECT_NEAR(diagonal.segment<3>(0).mean(), 1.0, 1e-12);
  EXPECT_NEAR(diagonal.segment<3>(3).mean(), 1.0, 1e-12);
  EXPECT_NEAR(diagonal.segment<3>(6).mean(), 1e-5, 1e-12);
  EXPECT_NEAR(diagonal.segment<3>(9).mean(), 1e-5, 1e-12);
  EXPECT_NEAR(diagonal.segment<3>(12).mean(), 1.0, 1e-12);
  EXPECT_NEAR(diagonal.segment<3>(15).mean(), 1e-4, 1e-12);
  EXPECT_NEAR(diagonal.segment<3>(18).mean(), 1e-3, 1e-12);
  EXPECT_NEAR(diagonal.segment<2>(21).mean(), 1e-5, 1e-12);
}

TEST(VendorLioEskf, UsesS2GravityWithFixedNorm) {
  SensorParams sensors;
  const auto params = vendorParams();
  VendorLioEskf filter(sensors, params);
  filter.initialize(SE3Pose::Identity(), Eigen::Vector3d::Zero(),
                    Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  EXPECT_NEAR(filter.state().gravity.norm(), params.gravity.norm(), 1e-12);
  EXPECT_EQ(filter.state().covariance.rows(), 23);
}

TEST(VendorLioEskf, StationaryCovariancePropagationIsFinite) {
  SensorParams sensors;
  const auto params = vendorParams();
  VendorLioEskf filter(sensors, params);
  filter.initialize(SE3Pose::Identity(), Eigen::Vector3d::Zero(),
                    Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

  const auto start = std::chrono::steady_clock::now();
  std::vector<ImuPacket> measurements;
  for (int index = 0; index <= 20; ++index) {
    ImuPacket imu;
    imu.stamp = start + std::chrono::milliseconds(index * 5);
    imu.accel = Eigen::Vector3d(0.0, 0.0, 9.81007);
    measurements.push_back(imu);
  }
  filter.propagateCovariance(measurements);
  EXPECT_TRUE(filter.state().covariance.allFinite());
  EXPECT_GT(filter.state().covariance.trace(), 0.0);
}

}  // namespace
