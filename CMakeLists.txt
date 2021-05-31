cmake_minimum_required(VERSION 3.5)
project(mrs_octomap_server)

set(PACKAGE_DEPENDENCIES
  roscpp
  nodelet
  sensor_msgs
  std_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  pcl_conversions
  octomap_ros
  octomap_msgs
  mrs_lib
  laser_geometry
  )

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options("-fPIC")
# add_compile_options(${PCL_COMPILE_OPTIONS})

find_package(OpenMP REQUIRED)

find_package(octomap REQUIRED)
add_definitions(-DOCTOMAP_NODEBUGOUT)

# include Eigen3
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES MrsOctomapConversions MrsOctomapServer
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
  DEPENDS OCTOMAP
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  )

# MrsOctomapConversions

add_library(MrsOctomapConversions SHARED
  src/conversions.cpp
  )

add_dependencies(MrsOctomapConversions
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(MrsOctomapConversions
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  )

# MrsOctomapServer

add_library(MrsOctomapServer SHARED
  src/octomap_server.cpp
  )

add_dependencies(MrsOctomapServer
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(MrsOctomapServer
  MrsOctomapConversions
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  )