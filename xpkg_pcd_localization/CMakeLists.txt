cmake_minimum_required(VERSION 3.0.2)
project(xpkg_pcd_localization)
add_compile_options(-std=c++11)
SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_ros
  pcl_conversions
  
)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)

catkin_package(
)

include_directories(
  ../
  ./
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
)


add_executable(pcd_localization
  pcd_localization/main.cc
  pcd_localization/data_interface.cc
  pcd_localization/pcd_localization.cc
)

target_link_libraries(pcd_localization
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)