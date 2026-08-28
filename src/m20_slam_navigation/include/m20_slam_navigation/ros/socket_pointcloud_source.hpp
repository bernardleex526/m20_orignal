#pragma once

#include "m20_slam_navigation/ros/drdds_pointcloud_source.hpp"

#include <functional>
#include <memory>
#include <string>

namespace m20::ros
{

class SocketPointCloudSource
{
public:
  using Callback = std::function<void(DrddsPointCloud &&)>;
  virtual ~SocketPointCloudSource() = default;

  static std::unique_ptr<SocketPointCloudSource> create(
    const std::string & socket_path, Callback callback, std::string & error);
};

}  // namespace m20::ros
