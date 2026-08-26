cmake_minimum_required(VERSION 3.16)
project(m20_slam_navigation LANGUAGES CXX)

# Default to C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# --- Dependencies -------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters registration io features search surface kdtree octree)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)

# --- Compiler flags -----------------------------------------------------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
  # Optimizations for embedded/NUC targets
  add_compile_options(-O3 -march=native -mtune=native)
  # OpenMP for parallelized VGICP
  find_package(OpenMP)
  if(OpenMP_CXX_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  endif()
endif()

# --- Headers ------------------------------------------------------------------
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${GTSAM_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

# --- Source files -------------------------------------------------------------
set(SOURCES
  # Common
  src/common/math_utils.cpp
  src/common/params.cpp

  # LIO Front-End
  src/lio/imu_processor.cpp
  src/lio/point_cloud_deskewer.cpp
  src/lio/voxel_map.cpp
  src/lio/fast_vgicp.cpp
  src/lio/lio_odometry.cpp

  # Back-End Factor Graph
  src/backend/factor_graph.cpp
  src/backend/degeneracy_detector.cpp
  src/backend/loop_closure.cpp
  src/backend/pose_graph_optimizer.cpp

  # Prior Map Relocalization
  src/localization/ndt_matcher.cpp
  src/localization/prior_map_loader.cpp
  src/localization/relocalizer.cpp
  src/localization/eskf.cpp

  # Terrain Traversability
  src/terrain/elevation_grid.cpp
  src/terrain/slope_analyzer.cpp
  src/terrain/roughness_analyzer.cpp
  src/terrain/step_detector.cpp
  src/terrain/traversability_map.cpp

  # Global Planner
  src/planning/hybrid_astar.cpp
  src/planning/motion_primitives.cpp
  src/planning/spline_optimizer.cpp
  src/planning/global_planner_node.cpp

  # Local Controller
  src/control/dwa_planner.cpp
  src/control/line_planner.cpp
  src/control/local_controller_node.cpp

  # Top-level Nodes
  src/nodes/slam_node.cpp
  src/nodes/localization_node.cpp
  src/nodes/navigation_node.cpp
)

# --- Shared library -----------------------------------------------------------
add_library(${PROJECT_NAME} SHARED ${SOURCES})

target_link_libraries(${PROJECT_NAME}
  rclcpp::rclcpp
  rclcpp_components::component
  rclcpp_lifecycle::rclcpp_lifecycle
  ${PCL_LIBRARIES}
  Eigen3::Eigen
  gtsam::gtsam
  ${OpenMP_CXX_LIBRARIES}
)
target_compile_definitions(${PROJECT_NAME} PRIVATE
  M20_SLAM_BUILDING_LIBRARY
)

# Thread safety (TSAN optional)
option(BUILD_TSAN "Build with ThreadSanitizer" OFF)
if(BUILD_TSAN)
  target_compile_options(${PROJECT_NAME} PRIVATE -fsanitize=thread -fno-omit-frame-pointer)
  target_link_options(${PROJECT_NAME} PRIVATE -fsanitize=thread)
endif()

# --- Install ------------------------------------------------------------------
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

# Install config and launch
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# --- Linting ------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# --- Export -------------------------------------------------------------------
ament_auto_package()
ament_export_dependencies(${dependencies})
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})