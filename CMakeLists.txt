cmake_minimum_required(VERSION 2.8.3)
project(lqr_controller)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  rosflight_msgs
  dynamic_reconfigure
)

find_package(Eigen3 REQUIRED)

add_subdirectory(lib/geometry)
add_subdirectory(lib/lin_alg_tools)

generate_dynamic_reconfigure_options(
  cfg/Figure8.cfg)

catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs rosflight_msgs
  dynamic_reconfigure
)


###########
## Build ##
###########

include_directories(
  include
  lib/geometry/include
  lib/lin_alg_tools/include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(lqr_controller
  src/lqr.cpp
  src/lqr_node.cpp)
add_dependencies(lqr_controller ${catkin_EXPORTED_TARGETS}
  %{PROJECT_NAME}_gencfg)
target_link_libraries(lqr_controller ${catkin_LIBRARIES} lin_alg_tools)

