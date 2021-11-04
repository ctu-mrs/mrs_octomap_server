cmake_minimum_required(VERSION 3.5)
project(mrs_octomap_server)

# Override CXX flags inherited from workspace, if precompiled PCL binaries from debian repos are used
if (DEFINED ENV{PCL_CROSS_COMPILATION})
  set(PCL_CROSS_COMPILATION $ENV{PCL_CROSS_COMPILATION})
else()
  set(PCL_CROSS_COMPILATION "false")
endif()
if(${PCL_CROSS_COMPILATION} STREQUAL "false")
  message("Using precompiled PCL binaries from debian repos. Overriding catkin workspace CMAKE_CXX_FLAGS.")
  set(CMAKE_CXX_FLAGS "-std=c++17")
else()
  message("Using custom-built PCL binaries. Inheriting all CMAKE_CXX_FLAGS from catkin workspace.")
endif()

set(PACKAGE_DEPENDENCIES
  roscpp
  nodelet
  sensor_msgs
  std_msgs
  nav_msgs
  geometry_msgs
  pcl_ros
  pcl_conversions
  octomap_ros
  octomap_msgs
  mrs_lib
  laser_geometry
  mrs_msgs
  )

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})
find_package(PCL REQUIRED COMPONENTS)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options("-fPIC")
add_compile_options("${PCL_COMPILE_OPTIONS}")

message("PCL Compile options: '${PCL_COMPILE_OPTIONS}', pes")
message("CXX flags: '${CMAKE_CXX_FLAGS}', pes")

find_package(OpenMP REQUIRED)

find_package(octomap REQUIRED)
add_definitions(-DOCTOMAP_NODEBUGOUT)

# include Eigen3
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES MrsOctomapServer
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

## Add link directories
link_directories(
  ${PCL_LIBRARY_DIRS}
)

# This causes adds the -march=native gcc flag that prevents ditributed compilation
add_definitions(
  ${PCL_DEFINITIONS}
  )

# MrsOctomapServer

add_library(MrsOctomapServer SHARED
  src/octomap_server.cpp
  src/conversions.cpp
  )

add_dependencies(MrsOctomapServer
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(MrsOctomapServer
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  )

target_include_directories(MrsOctomapServer SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
