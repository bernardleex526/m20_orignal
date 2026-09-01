#include "m20_slam_navigation/backend/pose_graph_optimizer.hpp"

#include <pcl/io/pcd_io.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace
{

struct VendorFrame
{
  double stamp{0.0};
  m20::FrameId id{0};
  m20::SE3Pose pose;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud{new pcl::PointCloud<pcl::PointXYZI>()};
};

std::vector<m20::SE3Pose> readPoses(const fs::path & path, std::vector<double> & stamps)
{
  std::ifstream stream(path);
  if (!stream) {
    throw std::runtime_error("cannot open vendor poses: " + path.string());
  }
  std::vector<m20::SE3Pose> poses;
  std::string line;
  std::size_t line_number = 0;
  while (std::getline(stream, line)) {
    ++line_number;
    if (line.empty()) {
      continue;
    }
    std::istringstream fields(line);
    double tx, ty, tz, qx, qy, qz, qw, stamp;
    std::string trailing;
    if (!(fields >> stamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw) ||
      (fields >> trailing))
    {
      throw std::runtime_error(
              "invalid vendor pose at line " + std::to_string(line_number));
    }
    const double values[]{stamp, tx, ty, tz, qx, qy, qz, qw};
    if (!std::all_of(std::begin(values), std::end(values), [](double value) {
        return std::isfinite(value);
      }))
    {
      throw std::runtime_error(
              "non-finite vendor pose at line " + std::to_string(line_number));
    }
    m20::SE3Pose pose;
    pose.t = {tx, ty, tz};
    pose.q = Eigen::Quaternion<m20::Scalar>(qw, qx, qy, qz);
    if (pose.q.norm() < 1e-9) {
      throw std::runtime_error(
              "zero vendor quaternion at line " + std::to_string(line_number));
    }
    pose.q.normalize();
    stamps.push_back(stamp);
    poses.push_back(pose);
  }
  if (poses.empty()) {
    throw std::runtime_error("vendor poses.txt is empty");
  }
  return poses;
}

std::vector<fs::path> keyframePaths(const fs::path & directory)
{
  if (!fs::is_directory(directory)) {
    throw std::runtime_error("vendor lidar_cloud directory is missing: " + directory.string());
  }
  std::vector<std::pair<m20::FrameId, fs::path>> numbered;
  for (const auto & entry : fs::directory_iterator(directory)) {
    if (!entry.is_regular_file() || entry.path().extension() != ".pcd") {
      continue;
    }
    const auto stem = entry.path().stem().string();
    std::size_t consumed = 0;
    m20::FrameId id = 0;
    try {
      id = static_cast<m20::FrameId>(std::stoull(stem, &consumed));
    } catch (const std::exception &) {
      throw std::runtime_error("non-numeric vendor keyframe name: " + entry.path().string());
    }
    if (consumed != stem.size()) {
      throw std::runtime_error("non-numeric vendor keyframe name: " + entry.path().string());
    }
    numbered.emplace_back(id, entry.path());
  }
  std::sort(numbered.begin(), numbered.end(), [](const auto & lhs, const auto & rhs) {
    return lhs.first < rhs.first;
  });
  std::vector<fs::path> paths;
  paths.reserve(numbered.size());
  for (const auto & item : numbered) {
    paths.push_back(item.second);
  }
  return paths;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr loadVendorCloud(const fs::path & path)
{
  pcl::PointCloud<pcl::PointXYZINormal> native;
  if (pcl::io::loadPCDFile(path.string(), native) != 0) {
    throw std::runtime_error("cannot load vendor keyframe: " + path.string());
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
  cloud->reserve(native.size());
  for (const auto & input : native) {
    if (!std::isfinite(input.x) || !std::isfinite(input.y) ||
      !std::isfinite(input.z) || !std::isfinite(input.intensity))
    {
      continue;
    }
    pcl::PointXYZI point;
    point.x = input.x;
    point.y = input.y;
    point.z = input.z;
    point.intensity = input.intensity;
    cloud->push_back(point);
  }
  cloud->width = static_cast<std::uint32_t>(cloud->size());
  cloud->height = 1;
  cloud->is_dense = true;
  return cloud;
}

double pathLength(const std::vector<VendorFrame> & frames)
{
  double length = 0.0;
  for (std::size_t index = 1; index < frames.size(); ++index) {
    length += (frames[index].pose.t - frames[index - 1].pose.t).norm();
  }
  return length;
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 3) {
    std::cerr << "Usage: " << argv[0] << " VENDOR_MAP_DIR OUTPUT_DIR\n";
    return 2;
  }
  try {
    const fs::path input = fs::weakly_canonical(argv[1]);
    const fs::path output = fs::absolute(argv[2]);
    const fs::path session = input / ".sessions" / "session_0";
    std::vector<double> stamps;
    const auto poses = readPoses(session / "poses.txt", stamps);
    const auto cloud_paths = keyframePaths(session / "lidar_cloud");
    if (poses.size() != cloud_paths.size()) {
      throw std::runtime_error(
              "vendor pose/cloud count mismatch: poses=" + std::to_string(poses.size()) +
              " clouds=" + std::to_string(cloud_paths.size()));
    }

    std::vector<VendorFrame> frames;
    frames.reserve(poses.size());
    for (std::size_t index = 0; index < poses.size(); ++index) {
      VendorFrame frame;
      frame.stamp = stamps[index];
      frame.id = static_cast<m20::FrameId>(std::stoull(cloud_paths[index].stem().string()));
      frame.pose = poses[index];
      frame.cloud = loadVendorCloud(cloud_paths[index]);
      frames.push_back(std::move(frame));
    }

    m20::BackendParams params;
    params.enable_loop_closure = true;
    m20::backend::PoseGraphOptimizer optimizer(params);
    std::vector<m20::LoopCandidate> loops;
    optimizer.setLoopClosureCallback([&loops](const m20::LoopCandidate & loop) {
      loops.push_back(loop);
    });

    Eigen::Matrix<m20::Scalar, 6, 6> prior_covariance =
      Eigen::Matrix<m20::Scalar, 6, 6>::Zero();
    Eigen::Matrix<m20::Scalar, 6, 6> odom_information =
      Eigen::Matrix<m20::Scalar, 6, 6>::Zero();
    for (int axis = 0; axis < 6; ++axis) {
      const auto prior_sigma = params.prior_noise_default_sigmas[axis];
      const auto odom_sigma = params.odom_noise_sigmas[axis];
      prior_covariance(axis, axis) = prior_sigma * prior_sigma;
      odom_information(axis, axis) = 1.0 / (odom_sigma * odom_sigma);
    }
    optimizer.addPriorPose(frames.front().id, frames.front().pose, prior_covariance);
    optimizer.addKeyframe(frames.front().id, frames.front().pose, frames.front().cloud);
    for (std::size_t index = 1; index < frames.size(); ++index) {
      const auto relative = frames[index - 1].pose.inverse() * frames[index].pose;
      optimizer.addOdometry(
        frames[index - 1].id, frames[index].id, relative, odom_information);
      optimizer.addKeyframe(frames[index].id, frames[index].pose, frames[index].cloud);
    }
    optimizer.optimize();

    const auto optimized = optimizer.getTrajectory();
    std::map<m20::FrameId, m20::SE3Pose> optimized_by_id;
    for (const auto & item : optimized) {
      optimized_by_id.emplace(item.first, item.second);
    }
    for (auto & frame : frames) {
      const auto found = optimized_by_id.find(frame.id);
      if (found != optimized_by_id.end()) {
        frame.pose = found->second;
      }
    }

    fs::create_directories(output);
    std::ofstream trajectory(output / "optimized_poses.txt");
    trajectory << std::fixed << std::setprecision(9);
    pcl::PointCloud<pcl::PointXYZI> map;
    for (const auto & frame : frames) {
      trajectory << frame.stamp << ' ' << frame.pose.t.x() << ' ' << frame.pose.t.y() << ' '
                 << frame.pose.t.z() << ' ' << frame.pose.q.x() << ' ' << frame.pose.q.y()
                 << ' ' << frame.pose.q.z() << ' ' << frame.pose.q.w() << '\n';
      for (const auto & input_point : *frame.cloud) {
        const Eigen::Vector3d local(input_point.x, input_point.y, input_point.z);
        const auto world = frame.pose.transformPoint(local);
        pcl::PointXYZI output_point;
        output_point.x = static_cast<float>(world.x());
        output_point.y = static_cast<float>(world.y());
        output_point.z = static_cast<float>(world.z());
        output_point.intensity = input_point.intensity;
        map.push_back(output_point);
      }
    }
    map.width = static_cast<std::uint32_t>(map.size());
    map.height = 1;
    map.is_dense = true;
    pcl::io::savePCDFileBinary((output / "optimized_full_cloud.pcd").string(), map);

    std::ofstream loop_file(output / "loops.txt");
    for (const auto & loop : loops) {
      loop_file << loop.src_frame << ' ' << loop.tgt_frame << ' '
                << loop.relative_pose.t.x() << ' ' << loop.relative_pose.t.y() << ' '
                << loop.relative_pose.t.z() << ' ' << loop.relative_pose.q.x() << ' '
                << loop.relative_pose.q.y() << ' ' << loop.relative_pose.q.z() << ' '
                << loop.relative_pose.q.w() << ' ' << loop.fitness_score << '\n';
    }

    const double length = pathLength(frames);
    const double start_end = (frames.back().pose.t - frames.front().pose.t).norm();
    std::ofstream summary(output / "adapter_summary.txt");
    summary << "source=" << input.string() << '\n'
            << "keyframes=" << frames.size() << '\n'
            << "input_points=" << map.size() << '\n'
            << "optimized_poses=" << optimized.size() << '\n'
            << "accepted_loops=" << loops.size() << '\n'
            << std::setprecision(9)
            << "path_length_m=" << length << '\n'
            << "start_end_distance_m=" << start_end << '\n';
    std::cout << "Vendor map adapted: keyframes=" << frames.size()
              << " points=" << map.size() << " loops=" << loops.size()
              << " output=" << output << std::endl;
    return 0;
  } catch (const std::exception & exception) {
    std::cerr << "Vendor map adaptation failed: " << exception.what() << '\n';
    return 1;
  }
}
