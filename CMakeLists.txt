cmake_minimum_required(VERSION 3.5)
project(mrs_octomap_server)

# set the correct standards
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler options
add_compile_options(-O3 -DOCTOMAP_NODEBUGOUT)


# 1 == use rclcpp::Timer implementation
# 0 == use MRS's thread timer implementation
set(USE_ROS_TIMER 0)
#set(USE_ROS_TIMER 1)


if(${USE_ROS_TIMER})
  MESSAGE(WARNING "[mrs_uav_autostart]: Compiling with ROS Timers. This can cause high CPU load in runtime.")
endif()



# Find dependencies
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
  MrsOctomapServer
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
## |                           Compile                          |
## --------------------------------------------------------------

add_library(MrsOctomapServer SHARED
  src/octomap_server.cpp
  src/conversions.cpp
)

ament_target_dependencies(MrsOctomapServer
  ${DEPENDENCIES}
)


## components
rclcpp_components_register_nodes(MrsOctomapServer PLUGIN "mrs_octomap_server::OctomapServer" EXECUTABLE MrsOctomapServer)

target_compile_definitions(MrsOctomapServer PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

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

install(DIRECTORY include/
  DESTINATION .)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  )

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)

ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  ${DEPENDENCIES}
)

ament_package()
