#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace m20::ros
{

struct DrddsPointField
{
  std::string name;
  std::uint32_t offset{0};
  std::uint8_t datatype{0};
  std::uint32_t count{0};
};

// Kept independent of ROS 2 and vendor generated headers because both expose
// a sensor_msgs::msg::PointCloud2 name with different C++ representations.
struct DrddsPointCloud
{
  std::int32_t stamp_sec{0};
  std::uint32_t stamp_nanosec{0};
  std::string frame_id;
  std::uint32_t height{0};
  std::uint32_t width{0};
  std::vector<DrddsPointField> fields;
  bool is_bigendian{false};
  std::uint32_t point_step{0};
  std::uint32_t row_step{0};
  std::vector<std::uint8_t> data;
  bool is_dense{false};
};

struct DrddsPointCloudSourceOptions
{
  std::string topic{"/LIDAR/POINTS"};
  int domain_id{0};
  bool use_shm{false};
  std::string topic_prefix{"rt"};
  std::string network_name;
};

class DrddsPointCloudSource
{
public:
  using Callback = std::function<void(DrddsPointCloud &&)>;

  virtual ~DrddsPointCloudSource() = default;
  virtual int matchedPublishers() const = 0;
  virtual bool updatedWithin(std::uint16_t milliseconds) const = 0;

  static std::unique_ptr<DrddsPointCloudSource> create(
    const DrddsPointCloudSourceOptions & options, Callback callback, std::string & error);
  static bool available();
};

}  // namespace m20::ros
