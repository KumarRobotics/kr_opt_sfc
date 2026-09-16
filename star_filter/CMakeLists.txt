cmake_minimum_required(VERSION 3.10)
project(star_filter)

set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC")


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  gcopter
  opt_sfc
)

find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    include
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  geometry_msgs gcopter opt_sfc)

# one safe-flight-corridor polytope
add_executable(single
  src/single.cpp
)
target_link_libraries(single
  ${catkin_LIBRARIES}
  OpenMP::OpenMP_CXX
  )

set_property(TARGET single
  PROPERTY CXX_STANDARD 17)

# online receding-horizon star-convex-set + kernel roadmap (RobustStarManager)
add_executable(sconvex
  src/sconvex.cpp
)
target_link_libraries(sconvex
  ${catkin_LIBRARIES}
  OpenMP::OpenMP_CXX
  )

set_property(TARGET sconvex
  PROPERTY CXX_STANDARD 17)
