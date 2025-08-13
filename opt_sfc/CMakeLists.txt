cmake_minimum_required(VERSION 3.10)
project(opt_sfc)

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
)

find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED)
find_package(HDF5 REQUIRED COMPONENTS C CXX)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${HDF5_INCLUDE_DIRS}
    include
)

catkin_package(
  CATKIN_DEPENDS
  geometry_msgs gcopter)

add_executable(sfc
  src/sfc.cpp
)
target_link_libraries(sfc
  ${catkin_LIBRARIES}
  ${OMPL_LIBRARIES}
  ${HDF5_CXX_LIBRARIES} ${HDF5_LIBRARIES}
  )

set_property(TARGET sfc 
  PROPERTY CXX_STANDARD 17)

  