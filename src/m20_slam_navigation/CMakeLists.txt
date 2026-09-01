cmake_minimum_required(VERSION 3.16)
project(m20_slam_navigation LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters registration io features search surface kdtree octree)
find_package(Eigen3 REQUIRED)
find_package(OpenMP QUIET)
find_package(Threads REQUIRED)

# The robot also installs a ROS package named "drdds" under /opt/ros/foxy,
# but the vendor C++ channel API used by slam_ddsnode is the separate package
# under /usr/local. Prefer that exact SDK when it is present.
set(M20_VENDOR_DRDDS_CMAKE_DIR "/usr/local/lib/cmake/drdds" CACHE PATH
  "Directory containing the robot vendor drddsConfig.cmake")
if(EXISTS "${M20_VENDOR_DRDDS_CMAKE_DIR}/drddsConfig.cmake")
  set(drdds_DIR "${M20_VENDOR_DRDDS_CMAKE_DIR}" CACHE PATH "" FORCE)
endif()
find_package(drdds QUIET CONFIG)
find_path(M20_VENDOR_DRDDS_INCLUDE_DIR
  NAMES drdds/core/drdds_core.h
  HINTS ${drdds_INCLUDE_DIRS} /usr/local/include)
find_path(M20_VENDOR_DRIDL_INCLUDE_DIR
  NAMES dridl/dr_msgs/msg/PointCloud2PubSubTypes.h
  HINTS ${drdds_INCLUDE_DIRS} /usr/local/include)

set(M20_VENDOR_DRDDS_FOUND FALSE)
if(drdds_FOUND AND M20_VENDOR_DRDDS_INCLUDE_DIR AND M20_VENDOR_DRIDL_INCLUDE_DIR)
  set(M20_VENDOR_DRDDS_FOUND TRUE)
  set(M20_REQUIRED_DRIDL_HEADERS
    dridl/dr_msgs/msg/PointCloud2PubSubTypes.h
    dridl/dr_msgs/msg/ImuPubSubTypes.h
    dridl/dr_msgs/msg/MotionInfoPubSubTypes.h
    dridl/dr_msgs/msg/NavCmdPubSubTypes.h
    dridl/dr_msgs/msg/PlannerStatusPubSubTypes.h
    dridl/dr_msgs/msg/StdMsgInt32PubSubTypes.h
    dridl/dr_srvs/srv/PoseStampedToInt32PubSubTypes.h
    dridl/dr_srvs/srv/StdSrvInt32PubSubTypes.h
    dridl/dr_srvs/srv/set_paramPubSubTypes.h)
  foreach(required_header IN LISTS M20_REQUIRED_DRIDL_HEADERS)
    if(NOT EXISTS "${M20_VENDOR_DRIDL_INCLUDE_DIR}/${required_header}")
      set(M20_VENDOR_DRDDS_FOUND FALSE)
      message(STATUS "M20 vendor DrDDS disabled: missing ${required_header}")
    endif()
  endforeach()
endif()

# Some M20Pro images contain a relocated GTSAM whose GTSAMConfig.cmake still
# points at the vendor build tree under /root/aarch64.  Resolve the installed
# public header and shared library directly so the package also works with a
# normal distro GTSAM installation.
find_path(M20_GTSAM_INCLUDE_DIR NAMES gtsam/geometry/Pose3.h)
find_library(M20_GTSAM_LIBRARY NAMES gtsam)
if(NOT M20_GTSAM_INCLUDE_DIR OR NOT M20_GTSAM_LIBRARY)
  message(FATAL_ERROR "GTSAM headers or library not found")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(M20_ROS_DEPENDENCIES
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  lifecycle_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  std_srvs
  std_msgs
  builtin_interfaces
  visualization_msgs
  pcl_conversions
)

set(M20_CORE_SOURCES
  src/common/math_utils.cpp
  src/common/params.cpp
  src/lio/imu_processor.cpp
  src/lio/point_cloud_deskewer.cpp
  src/lio/voxel_map.cpp
  src/lio/vendor_lio_eskf.cpp
  src/lio/vendor_output_contract.cpp
  src/lio/lio_odometry.cpp
  src/backend/factor_graph.cpp
  src/backend/degeneracy_detector.cpp
  src/backend/loop_closure.cpp
  src/backend/pose_graph_optimizer.cpp
)

option(
  BUILD_EXPERIMENTAL_NAVIGATION
  "Build the M20Pro native-contract localization/planning/controller modules"
  ON
)

if(BUILD_EXPERIMENTAL_NAVIGATION)
  list(APPEND M20_CORE_SOURCES
    src/localization/ndt_matcher.cpp
    src/localization/prior_map_loader.cpp
    src/localization/relocalizer.cpp
    src/localization/eskf.cpp
    src/terrain/elevation_grid.cpp
    src/terrain/slope_analyzer.cpp
    src/terrain/roughness_analyzer.cpp
    src/terrain/step_detector.cpp
    src/terrain/traversability_map.cpp
    src/planning/hybrid_astar.cpp
    src/planning/motion_primitives.cpp
    src/planning/spline_optimizer.cpp
    src/planning/global_planner_node.cpp
    src/control/dwa_planner.cpp
    src/control/line_planner.cpp
    src/control/local_controller_node.cpp
  )
endif()

add_library(m20_slam_core SHARED ${M20_CORE_SOURCES})
target_include_directories(m20_slam_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
  ${M20_GTSAM_INCLUDE_DIR}
)
ament_target_dependencies(m20_slam_core ${M20_ROS_DEPENDENCIES})
target_link_libraries(m20_slam_core ${PCL_LIBRARIES} Eigen3::Eigen ${M20_GTSAM_LIBRARY})
if(OpenMP_CXX_FOUND)
  target_link_libraries(m20_slam_core OpenMP::OpenMP_CXX)
endif()

add_library(m20_drdds_input SHARED src/ros/drdds_pointcloud_source.cpp)
target_include_directories(m20_drdds_input PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
if(M20_VENDOR_DRDDS_FOUND)
  target_compile_definitions(m20_drdds_input PRIVATE M20_HAS_DRDDS=1)
  target_include_directories(m20_drdds_input SYSTEM PRIVATE
    ${drdds_INCLUDE_DIRS} ${M20_VENDOR_DRDDS_INCLUDE_DIR} ${M20_VENDOR_DRIDL_INCLUDE_DIR})
  if(TARGET drdds::drdds)
    target_link_libraries(m20_drdds_input PRIVATE drdds::drdds)
  else()
    target_link_directories(m20_drdds_input PRIVATE /usr/local/lib)
    target_link_libraries(m20_drdds_input PRIVATE ${drdds_LIBRARIES})
  endif()
  message(STATUS "M20 vendor DrDDS point-cloud and IMU input enabled")
else()
  message(STATUS "M20 vendor DrDDS point-cloud and IMU input disabled (drdds package not found)")
endif()

# Native navigation message/service adapter.  It is a no-op on a workstation
# without the vendor SDK and becomes the exact DrDDS /NAV_CMD, status,
# /MOTION_INFO, /planner_mode and service bridge on an M20Pro image.
add_library(m20_native_navigation_bridge SHARED src/ros/native_navigation_bridge.cpp)
target_include_directories(m20_native_navigation_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(m20_native_navigation_bridge PRIVATE Eigen3::Eigen ${PCL_LIBRARIES})
if(M20_VENDOR_DRDDS_FOUND)
  target_compile_definitions(m20_native_navigation_bridge PRIVATE M20_HAS_DRDDS=1)
  target_include_directories(m20_native_navigation_bridge SYSTEM PRIVATE
    ${drdds_INCLUDE_DIRS} ${M20_VENDOR_DRDDS_INCLUDE_DIR} ${M20_VENDOR_DRIDL_INCLUDE_DIR})
  if(TARGET drdds::drdds)
    target_link_libraries(m20_native_navigation_bridge PRIVATE drdds::drdds)
  else()
    target_link_directories(m20_native_navigation_bridge PRIVATE /usr/local/lib)
    target_link_libraries(m20_native_navigation_bridge PRIVATE ${drdds_LIBRARIES})
  endif()
endif()

add_library(m20_pointcloud_wire SHARED src/ros/pointcloud_wire.cpp)
target_include_directories(m20_pointcloud_wire PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(m20_socket_input SHARED src/ros/socket_pointcloud_source.cpp)
target_include_directories(m20_socket_input PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(m20_socket_input PRIVATE m20_pointcloud_wire Threads::Threads pthread)

add_executable(m20_drdds_receiver src/ros/drdds_receiver_main.cpp)
target_include_directories(m20_drdds_receiver PRIVATE include)
target_link_libraries(
  m20_drdds_receiver PRIVATE m20_drdds_input m20_pointcloud_wire Threads::Threads pthread)

function(add_m20_component target source plugin executable)
  add_library(${target} SHARED ${source})
  target_include_directories(${target} PRIVATE include ${PCL_INCLUDE_DIRS})
  ament_target_dependencies(${target} ${M20_ROS_DEPENDENCIES})
  target_link_libraries(${target} m20_slam_core ${PCL_LIBRARIES})
  rclcpp_components_register_node(${target}
    PLUGIN "${plugin}"
    EXECUTABLE ${executable}
  )
endfunction()

add_m20_component(
  slam_node_component src/nodes/slam_node.cpp
  "m20::nodes::SlamNode" slam_node)
target_link_libraries(slam_node_component m20_socket_input)

set(M20_INSTALL_TARGETS
  m20_slam_core
  m20_drdds_input
  m20_pointcloud_wire
  m20_socket_input
  m20_native_navigation_bridge
  slam_node_component
)
if(BUILD_EXPERIMENTAL_NAVIGATION)
  add_m20_component(
    localization_node_component src/nodes/localization_node.cpp
    "m20::nodes::LocalizationNode" localization_node)
  add_m20_component(
    navigation_node_component src/nodes/navigation_node.cpp
    "m20::nodes::NavigationNode" navigation_node)
  target_link_libraries(navigation_node_component m20_native_navigation_bridge)
  if(M20_VENDOR_DRDDS_FOUND)
    target_compile_definitions(navigation_node_component PRIVATE M20_HAS_DRDDS=1)
    target_include_directories(navigation_node_component SYSTEM PRIVATE
      ${drdds_INCLUDE_DIRS} ${M20_VENDOR_DRDDS_INCLUDE_DIR} ${M20_VENDOR_DRIDL_INCLUDE_DIR})
  endif()
  list(APPEND M20_INSTALL_TARGETS localization_node_component navigation_node_component)
endif()

install(
  TARGETS ${M20_INSTALL_TARGETS}
  EXPORT export_m20_slam_navigation
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS m20_drdds_receiver RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(FILES config/m20_mapping.yaml DESTINATION share/${PROJECT_NAME}/config)
if(BUILD_EXPERIMENTAL_NAVIGATION)
  install(FILES
    config/localization_params.yaml
    config/native_navigation.yaml
    config/native_global_planner.yaml
    config/native_local_planner.yaml
    config/native_passable_area.yaml
    config/native_pcl_pass_grid.yaml
    config/native_global_topics.yaml
    config/native_lidar_params.yaml
    config/native_body_params.yaml
    DESTINATION share/${PROJECT_NAME}/config)
  install(FILES
    launch/navigation_system.launch.py
    launch/localization_system.launch.py
    launch/m20_full_system.launch.py
    DESTINATION share/${PROJECT_NAME}/launch)
endif()
install(FILES launch/slam_system.launch.py DESTINATION share/${PROJECT_NAME}/launch)
install(FILES
  rviz/slam_view.rviz
  rviz/navigation_view.rviz
  DESTINATION share/${PROJECT_NAME}/rviz)
install(PROGRAMS scripts/start_mapping.sh DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_m20_cloud_adapter test/test_m20_cloud_adapter.cpp)
  if(TARGET test_m20_cloud_adapter)
    target_include_directories(test_m20_cloud_adapter PRIVATE include ${PCL_INCLUDE_DIRS})
    ament_target_dependencies(test_m20_cloud_adapter sensor_msgs)
    target_link_libraries(test_m20_cloud_adapter ${PCL_LIBRARIES})
  endif()
  ament_add_gtest(test_sensor_wire test/test_sensor_wire.cpp)
  if(TARGET test_sensor_wire)
    target_include_directories(test_sensor_wire PRIVATE include)
    target_link_libraries(test_sensor_wire m20_pointcloud_wire)
  endif()
  ament_add_gtest(test_vendor_lio test/test_vendor_lio.cpp)
  if(TARGET test_vendor_lio)
    target_include_directories(test_vendor_lio PRIVATE include ${PCL_INCLUDE_DIRS})
    target_link_libraries(test_vendor_lio m20_slam_core ${PCL_LIBRARIES})
  endif()
  ament_add_gtest(test_vendor_output_contract test/test_vendor_output_contract.cpp)
  if(TARGET test_vendor_output_contract)
    target_include_directories(test_vendor_output_contract PRIVATE include ${PCL_INCLUDE_DIRS})
    ament_target_dependencies(test_vendor_output_contract sensor_msgs pcl_conversions)
    target_link_libraries(test_vendor_output_contract m20_slam_core ${PCL_LIBRARIES})
  endif()
  ament_add_gtest(test_backend_geometry test/test_backend_geometry.cpp)
  if(TARGET test_backend_geometry)
    target_include_directories(test_backend_geometry PRIVATE include ${PCL_INCLUDE_DIRS})
    ament_target_dependencies(test_backend_geometry rclcpp)
    target_link_libraries(test_backend_geometry m20_slam_core ${PCL_LIBRARIES})
  endif()
  if(BUILD_EXPERIMENTAL_NAVIGATION)
    ament_add_gtest(test_localization_eskf test/test_localization_eskf.cpp)
    if(TARGET test_localization_eskf)
      target_include_directories(test_localization_eskf PRIVATE include ${PCL_INCLUDE_DIRS})
      target_link_libraries(test_localization_eskf m20_slam_core ${PCL_LIBRARIES})
    endif()
    ament_add_gtest(test_native_navigation_contract test/test_native_navigation_contract.cpp)
    if(TARGET test_native_navigation_contract)
      target_include_directories(test_native_navigation_contract PRIVATE include ${PCL_INCLUDE_DIRS})
      ament_target_dependencies(test_native_navigation_contract rclcpp)
      target_link_libraries(test_native_navigation_contract m20_slam_core ${PCL_LIBRARIES})
    endif()
  endif()
endif()

ament_export_targets(export_m20_slam_navigation HAS_LIBRARY_TARGET)
ament_export_dependencies(${M20_ROS_DEPENDENCIES})
ament_export_include_directories(include)
ament_export_libraries(m20_slam_core)
ament_package()
