#pragma once

#include "m20_slam_navigation/ros/drdds_pointcloud_source.hpp"
#include "m20_slam_navigation/ros/drdds_imu_source.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace m20::ros
{

constexpr std::uint32_t kPointCloudWireMagic = 0x4D323043U;  // M20C
constexpr std::uint32_t kPointCloudWireVersion = 1;
constexpr std::uint32_t kMaxPointCloudWireBytes = 64U * 1024U * 1024U;
constexpr std::uint32_t kImuWireMagic = 0x4D323049U;  // M20I
constexpr std::uint32_t kImuWireVersion = 1;
constexpr std::uint32_t kMaxImuWireBytes = 4096U;

std::vector<std::uint8_t> serializePointCloud(const DrddsPointCloud & cloud);
bool deserializePointCloud(
  const std::vector<std::uint8_t> & bytes, DrddsPointCloud & cloud, std::string & error);
std::vector<std::uint8_t> serializeImu(const DrddsImu & imu);
bool deserializeImu(
  const std::vector<std::uint8_t> & bytes, DrddsImu & imu, std::string & error);

}  // namespace m20::ros
