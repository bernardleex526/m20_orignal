#pragma once

#include "m20_slam_navigation/ros/drdds_imu_source.hpp"

#include <functional>
#include <memory>
#include <string>

namespace m20::ros
{

class SocketImuSource
{
public:
  using Callback = std::function<void(DrddsImu &&)>;
  virtual ~SocketImuSource() = default;

  static std::unique_ptr<SocketImuSource> create(
    const std::string & socket_path, Callback callback, std::string & error);
};

}  // namespace m20::ros
