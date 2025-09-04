cmake_minimum_required(VERSION 3.8)
project(mrs_octomap_server)

# Compiler options
add_compile_options(-O3 -DOCTOMAP_NODEBUGOUT)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(octomap REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(OpenMP REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(mrs_lib REQUIRED)
find_package(mrs_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(octomap_ros REQUIRED)
find_package(mrs_octomap_tools REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Messages (ROS2)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PoseWithSize.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

set(LIBRARIES MrsOctomapServer_Server)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# Server library
add_library(MrsOctomapServer_Server
  src/octomap_server.cpp
  src/conversions.cpp
)

ament_target_dependencies(MrsOctomapServer_Server
  Eigen3
  octomap
  PCL
  geometry_msgs
  laser_geometry
  mrs_lib
  mrs_msgs
  octomap_msgs
  octomap_ros
  mrs_octomap_tools
  pcl_conversions
  pcl_msgs
  pcl_ros
  rclcpp
  sensor_msgs
  std_msgs
)

target_include_directories(MrsOctomapServer_Server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(MrsOctomapServer_Server
  ${PCL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)

# Install
install(TARGETS ${LIBRARIES}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)


ament_export_include_directories(include)
ament_export_libraries(${LIBRARIES})
ament_export_dependencies(Eigen3 octomap PCL)

ament_package()