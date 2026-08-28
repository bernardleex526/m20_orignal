#pragma once

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

namespace m20::ros
{

enum class StampAction
{
  ACCEPT,
  CLAMP,
  DROP,
};

struct StampResult
{
  StampAction action{StampAction::ACCEPT};
  std::int64_t stamp_ns{0};
  std::int64_t rollback_ns{0};
};

inline StampResult makeMonotonicStamp(
  std::int64_t current_ns, std::int64_t last_ns,
  std::int64_t max_clamp_rollback_ns = 20000000LL)
{
  StampResult result;
  result.stamp_ns = current_ns;
  if (last_ns < 0 || current_ns > last_ns) {
    return result;
  }

  result.rollback_ns = last_ns - current_ns;
  if (result.rollback_ns > max_clamp_rollback_ns ||
      last_ns == std::numeric_limits<std::int64_t>::max()) {
    result.action = StampAction::DROP;
    return result;
  }

  result.action = StampAction::CLAMP;
  result.stamp_ns = last_ns + 1;
  return result;
}

struct CloudAdaptResult
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud{
    new pcl::PointCloud<pcl::PointXYZI>()};
  std::vector<double> point_time_offsets;
  std::vector<std::uint16_t> rings;
  std::int64_t scan_start_ns{0};
  std::int64_t scan_end_ns{0};
  bool rewrote_header_stamp{false};
  std::string error;
};

inline const sensor_msgs::msg::PointField * findField(
  const sensor_msgs::msg::PointCloud2 & cloud, const std::string & name)
{
  const auto it = std::find_if(
    cloud.fields.begin(), cloud.fields.end(),
    [&name](const auto & field) {return field.name == name;});
  return it == cloud.fields.end() ? nullptr : &*it;
}

template<typename T>
inline T readField(const std::uint8_t * point, std::uint32_t offset)
{
  T value{};
  std::memcpy(&value, point + offset, sizeof(T));
  return value;
}

inline CloudAdaptResult adaptM20Cloud(const sensor_msgs::msg::PointCloud2 & input)
{
  CloudAdaptResult result;
  const auto * x = findField(input, "x");
  const auto * y = findField(input, "y");
  const auto * z = findField(input, "z");
  const auto * intensity = findField(input, "intensity");
  const auto * ring = findField(input, "ring");
  const auto * timestamp = findField(input, "timestamp");

  if (!(x && y && z && intensity && ring && timestamp)) {
    result.error = "M20 point cloud requires x,y,z,intensity,ring,timestamp fields";
    return result;
  }
  if (input.is_bigendian) {
    result.error = "big-endian PointCloud2 is not supported";
    return result;
  }
  if (x->datatype != sensor_msgs::msg::PointField::FLOAT32 ||
      y->datatype != sensor_msgs::msg::PointField::FLOAT32 ||
      z->datatype != sensor_msgs::msg::PointField::FLOAT32 ||
      intensity->datatype != sensor_msgs::msg::PointField::FLOAT32 ||
      ring->datatype != sensor_msgs::msg::PointField::UINT16 ||
      timestamp->datatype != sensor_msgs::msg::PointField::FLOAT64) {
    result.error =
      "M20 field types must be x/y/z/intensity=float32, ring=uint16, timestamp=float64";
    return result;
  }

  const auto fits = [&input](const auto * field, std::size_t size) {
      return static_cast<std::size_t>(field->offset) + size <= input.point_step;
    };
  if (!(fits(x, 4) && fits(y, 4) && fits(z, 4) && fits(intensity, 4) &&
        fits(ring, 2) && fits(timestamp, 8))) {
    result.error = "PointCloud2 field offset exceeds point_step";
    return result;
  }

  const std::size_t total_points =
    static_cast<std::size_t>(input.width) * static_cast<std::size_t>(input.height);
  if (total_points == 0) {
    result.error = "PointCloud2 is empty";
    return result;
  }
  if (input.row_step < input.point_step * input.width ||
      input.data.size() < static_cast<std::size_t>(input.row_step) * input.height) {
    result.error = "PointCloud2 data or row_step is incomplete";
    return result;
  }

  double min_timestamp = std::numeric_limits<double>::infinity();
  double max_timestamp = -std::numeric_limits<double>::infinity();
  for (std::uint32_t row = 0; row < input.height; ++row) {
    for (std::uint32_t col = 0; col < input.width; ++col) {
      const auto offset = static_cast<std::size_t>(row) * input.row_step +
        static_cast<std::size_t>(col) * input.point_step;
      const double value = readField<double>(input.data.data() + offset, timestamp->offset);
      if (std::isfinite(value)) {
        min_timestamp = std::min(min_timestamp, value);
        max_timestamp = std::max(max_timestamp, value);
      }
    }
  }
  if (!std::isfinite(min_timestamp) || !std::isfinite(max_timestamp)) {
    result.error = "timestamp field has no finite value";
    return result;
  }

  const double header_time = static_cast<double>(input.header.stamp.sec) +
    static_cast<double>(input.header.stamp.nanosec) * 1e-9;
  double base_time = header_time;
  if (!std::isfinite(header_time) || min_timestamp - header_time < -0.02 ||
      max_timestamp - header_time > 0.5) {
    base_time = min_timestamp;
    result.rewrote_header_stamp = true;
  }

  result.cloud->reserve(total_points);
  result.point_time_offsets.reserve(total_points);
  result.rings.reserve(total_points);
  for (std::uint32_t row = 0; row < input.height; ++row) {
    for (std::uint32_t col = 0; col < input.width; ++col) {
      const auto offset = static_cast<std::size_t>(row) * input.row_step +
        static_cast<std::size_t>(col) * input.point_step;
      const auto * point = input.data.data() + offset;
      pcl::PointXYZI output;
      output.x = readField<float>(point, x->offset);
      output.y = readField<float>(point, y->offset);
      output.z = readField<float>(point, z->offset);
      output.intensity = readField<float>(point, intensity->offset);
      const double absolute_time = readField<double>(point, timestamp->offset);
      if (!std::isfinite(output.x) || !std::isfinite(output.y) || !std::isfinite(output.z) ||
          !std::isfinite(absolute_time)) {
        continue;
      }
      result.cloud->push_back(output);
      result.point_time_offsets.push_back(std::max(0.0, absolute_time - base_time));
      result.rings.push_back(readField<std::uint16_t>(point, ring->offset));
    }
  }
  if (result.cloud->empty()) {
    result.error = "PointCloud2 contains no finite points";
    return result;
  }

  result.cloud->width = static_cast<std::uint32_t>(result.cloud->size());
  result.cloud->height = 1;
  result.cloud->is_dense = true;
  result.scan_start_ns = static_cast<std::int64_t>(std::llround(base_time * 1e9));
  const double duration = *std::max_element(
    result.point_time_offsets.begin(), result.point_time_offsets.end());
  if (!std::isfinite(duration) || duration > 0.5) {
    result.error = "point timestamp span is invalid or exceeds 0.5 seconds";
    return result;
  }
  result.scan_end_ns = result.scan_start_ns +
    static_cast<std::int64_t>(std::llround(duration * 1e9));
  return result;
}

}  // namespace m20::ros
