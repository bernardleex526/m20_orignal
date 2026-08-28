#include "m20_slam_navigation/ros/pointcloud_wire.hpp"

#include <cstring>
#include <limits>
#include <type_traits>

namespace m20::ros
{
namespace
{

template<typename T>
void append(std::vector<std::uint8_t> & output, const T & value)
{
  static_assert(std::is_trivially_copyable_v<T>);
  const auto * begin = reinterpret_cast<const std::uint8_t *>(&value);
  output.insert(output.end(), begin, begin + sizeof(T));
}

void appendString(std::vector<std::uint8_t> & output, const std::string & value)
{
  append(output, static_cast<std::uint32_t>(value.size()));
  output.insert(output.end(), value.begin(), value.end());
}

template<typename T>
bool read(const std::vector<std::uint8_t> & input, std::size_t & offset, T & value)
{
  static_assert(std::is_trivially_copyable_v<T>);
  if (offset > input.size() || input.size() - offset < sizeof(T)) {
    return false;
  }
  std::memcpy(&value, input.data() + offset, sizeof(T));
  offset += sizeof(T);
  return true;
}

bool readString(
  const std::vector<std::uint8_t> & input, std::size_t & offset, std::string & value)
{
  std::uint32_t size = 0;
  if (!read(input, offset, size) || offset > input.size() || input.size() - offset < size) {
    return false;
  }
  value.assign(reinterpret_cast<const char *>(input.data() + offset), size);
  offset += size;
  return true;
}

}  // namespace

std::vector<std::uint8_t> serializePointCloud(const DrddsPointCloud & cloud)
{
  std::vector<std::uint8_t> output;
  output.reserve(cloud.data.size() + cloud.fields.size() * 32U + cloud.frame_id.size() + 64U);
  append(output, kPointCloudWireVersion);
  append(output, cloud.stamp_sec);
  append(output, cloud.stamp_nanosec);
  appendString(output, cloud.frame_id);
  append(output, cloud.height);
  append(output, cloud.width);
  append(output, static_cast<std::uint32_t>(cloud.fields.size()));
  for (const auto & field : cloud.fields) {
    appendString(output, field.name);
    append(output, field.offset);
    append(output, field.datatype);
    append(output, field.count);
  }
  append(output, static_cast<std::uint8_t>(cloud.is_bigendian));
  append(output, cloud.point_step);
  append(output, cloud.row_step);
  append(output, static_cast<std::uint8_t>(cloud.is_dense));
  append(output, static_cast<std::uint32_t>(cloud.data.size()));
  output.insert(output.end(), cloud.data.begin(), cloud.data.end());
  return output;
}

bool deserializePointCloud(
  const std::vector<std::uint8_t> & bytes, DrddsPointCloud & cloud, std::string & error)
{
  std::size_t offset = 0;
  std::uint32_t version = 0;
  std::uint32_t field_count = 0;
  std::uint8_t flag = 0;
  if (!read(bytes, offset, version) || version != kPointCloudWireVersion ||
    !read(bytes, offset, cloud.stamp_sec) || !read(bytes, offset, cloud.stamp_nanosec) ||
    !readString(bytes, offset, cloud.frame_id) || !read(bytes, offset, cloud.height) ||
    !read(bytes, offset, cloud.width) || !read(bytes, offset, field_count) || field_count > 128U)
  {
    error = "invalid point-cloud wire header";
    return false;
  }
  cloud.fields.clear();
  cloud.fields.reserve(field_count);
  for (std::uint32_t index = 0; index < field_count; ++index) {
    DrddsPointField field;
    if (!readString(bytes, offset, field.name) || !read(bytes, offset, field.offset) ||
      !read(bytes, offset, field.datatype) || !read(bytes, offset, field.count))
    {
      error = "truncated point-cloud field table";
      return false;
    }
    cloud.fields.push_back(std::move(field));
  }
  std::uint32_t data_size = 0;
  if (!read(bytes, offset, flag)) {
    error = "truncated point-cloud endian flag";
    return false;
  }
  cloud.is_bigendian = flag != 0;
  if (!read(bytes, offset, cloud.point_step) || !read(bytes, offset, cloud.row_step) ||
    !read(bytes, offset, flag) || !read(bytes, offset, data_size) ||
    data_size > kMaxPointCloudWireBytes || offset > bytes.size() || bytes.size() - offset != data_size)
  {
    error = "invalid point-cloud data length";
    return false;
  }
  cloud.is_dense = flag != 0;
  cloud.data.assign(bytes.begin() + static_cast<std::ptrdiff_t>(offset), bytes.end());
  error.clear();
  return true;
}

std::vector<std::uint8_t> serializeImu(const DrddsImu & imu)
{
  std::vector<std::uint8_t> output;
  output.reserve(imu.frame_id.size() + 320U);
  append(output, kImuWireVersion);
  append(output, imu.stamp_sec);
  append(output, imu.stamp_nanosec);
  appendString(output, imu.frame_id);
  append(output, imu.orientation.x);
  append(output, imu.orientation.y);
  append(output, imu.orientation.z);
  append(output, imu.orientation.w);
  for (const auto value : imu.orientation_covariance) append(output, value);
  append(output, imu.angular_velocity.x);
  append(output, imu.angular_velocity.y);
  append(output, imu.angular_velocity.z);
  for (const auto value : imu.angular_velocity_covariance) append(output, value);
  append(output, imu.linear_acceleration.x);
  append(output, imu.linear_acceleration.y);
  append(output, imu.linear_acceleration.z);
  for (const auto value : imu.linear_acceleration_covariance) append(output, value);
  return output;
}

bool deserializeImu(
  const std::vector<std::uint8_t> & bytes, DrddsImu & imu, std::string & error)
{
  std::size_t offset = 0;
  std::uint32_t version = 0;
  if (!read(bytes, offset, version) || version != kImuWireVersion ||
    !read(bytes, offset, imu.stamp_sec) || !read(bytes, offset, imu.stamp_nanosec) ||
    !readString(bytes, offset, imu.frame_id) ||
    !read(bytes, offset, imu.orientation.x) || !read(bytes, offset, imu.orientation.y) ||
    !read(bytes, offset, imu.orientation.z) || !read(bytes, offset, imu.orientation.w))
  {
    error = "invalid IMU wire header";
    return false;
  }
  for (auto & value : imu.orientation_covariance) {
    if (!read(bytes, offset, value)) {error = "truncated IMU orientation covariance"; return false;}
  }
  if (!read(bytes, offset, imu.angular_velocity.x) ||
    !read(bytes, offset, imu.angular_velocity.y) ||
    !read(bytes, offset, imu.angular_velocity.z))
  {
    error = "truncated IMU angular velocity";
    return false;
  }
  for (auto & value : imu.angular_velocity_covariance) {
    if (!read(bytes, offset, value)) {error = "truncated IMU angular covariance"; return false;}
  }
  if (!read(bytes, offset, imu.linear_acceleration.x) ||
    !read(bytes, offset, imu.linear_acceleration.y) ||
    !read(bytes, offset, imu.linear_acceleration.z))
  {
    error = "truncated IMU linear acceleration";
    return false;
  }
  for (auto & value : imu.linear_acceleration_covariance) {
    if (!read(bytes, offset, value)) {error = "truncated IMU acceleration covariance"; return false;}
  }
  if (offset != bytes.size()) {
    error = "invalid IMU wire data length";
    return false;
  }
  error.clear();
  return true;
}

}  // namespace m20::ros
