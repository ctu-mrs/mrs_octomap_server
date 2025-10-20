cmake_minimum_required(VERSION 3.20.0)
project(mrs_octomap_server)

# set the correct standards
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 1 == use rclcpp::Timer implementation
# 0 == use MRS's thread timer implementation
set(USE_ROS_TIMER 0)

if(${USE_ROS_TIMER})
  MESSAGE(WARNING "[mrs_octomap_server]: Compiling with ROS Timers. This can cause high CPU load in runtime.")
endif()

# set the compile options to show code warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(ENABLE_COVERAGE)
  message(WARNING "building for coverage, the performance might be limited")

  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage -fprofile-arcs")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -fprofile-arcs")

  add_definitions("-O1")
  add_definitions("-g")
endif()

set(DEPENDENCIES
  ament_cmake
  rclcpp
  rclcpp_components
  Eigen3
  sensor_msgs
  std_srvs
  std_msgs
  geometry_msgs
  laser_geometry
  mrs_lib
  mrs_msgs
  octomap
  octomap_msgs
  PCL
  pcl_conversions
  pcl_ros
  mrs_modules_msgs
)

set(LIBRARIES
  MrsOctomapServer_OctomapServer
)

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${mrs_lib_INCLUDE_DIRS}
  ${mrs_modules_msgs_INCLUDE_DIRS}
  ${mrs_msgs_INCLUDE_DIRS}
)

## --------------------------------------------------------------
## |                           compile                          |
## --------------------------------------------------------------

add_library(MrsOctomapServer_OctomapServer SHARED
  src/octomap_server.cpp
  src/conversions.cpp
)

ament_target_dependencies(MrsOctomapServer_OctomapServer
  ${DEPENDENCIES}
)

rclcpp_components_register_nodes(MrsOctomapServer_OctomapServer PLUGIN "mrs_octomap_server::OctomapServer" EXECUTABLE MrsOctomapServer_OctomapServer)

target_compile_definitions(MrsOctomapServer_OctomapServer PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

## --------------------------------------------------------------
## |                           install                          |
## --------------------------------------------------------------

ament_export_libraries(
  ${LIBRARIES}
)

install(
  TARGETS ${LIBRARIES}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION .
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY tmux
  USE_SOURCE_PERMISSIONS
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  ${DEPENDENCIES}
)

ament_package()
