#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

namespace m20::ros
{

struct DrddsVector3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct DrddsQuaternion
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double w{1.0};
};

struct DrddsImu
{
  std::int32_t stamp_sec{0};
  std::uint32_t stamp_nanosec{0};
  std::string frame_id;
  DrddsQuaternion orientation;
  std::array<double, 9> orientation_covariance{};
  DrddsVector3 angular_velocity;
  std::array<double, 9> angular_velocity_covariance{};
  DrddsVector3 linear_acceleration;
  std::array<double, 9> linear_acceleration_covariance{};
};

struct DrddsImuSourceOptions
{
  std::string topic{"/IMU_YESENSE"};
  int domain_id{0};
  bool use_shm{false};
  std::string topic_prefix{"rt"};
  std::string network_name;
};

class DrddsImuSource
{
public:
  using Callback = std::function<void(DrddsImu &&)>;

  virtual ~DrddsImuSource() = default;
  virtual int matchedPublishers() const = 0;
  virtual bool updatedWithin(std::uint16_t milliseconds) const = 0;

  static std::unique_ptr<DrddsImuSource> create(
    const DrddsImuSourceOptions & options, Callback callback, std::string & error);
  static bool available();
};

}  // namespace m20::ros
