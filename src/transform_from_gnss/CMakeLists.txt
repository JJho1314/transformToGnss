cmake_minimum_required(VERSION 3.0.2)
project(transform_from_gnss)

set(CMAKE_BUILD_TYPE "Release")

set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread -std=c++0x -std=c++14 -fexceptions")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs  
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  pcl_ros
  tf
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  include
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
  DEPENDS EIGEN3 
  INCLUDE_DIRS
)

add_executable(transform_from_gnss src/main.cpp)
target_link_libraries(transform_from_gnss ${catkin_LIBRARIES} ${PYTHON_LIBRARIES}  ${GeographicLib_LIBRARIES}) 
target_include_directories(transform_from_gnss PRIVATE ${PYTHON_INCLUDE_DIRS})
