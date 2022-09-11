cmake_minimum_required(VERSION 3.0.2)
project(gnss_imu_fusion)


SET(CMAKE_CXX_COMPILER /usr/bin/g++)
SET(CMAKE_CXX_FLAGS "-std=c++0x")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)


# Boost
find_package(Boost 1.58 REQUIRED COMPONENTS system serialization filesystem thread date_time regex timer chrono)
include_directories(${Boost_INCLUDE_DIR})
message("Boost_INCLUDE_DIR: " ${Boost_INCLUDE_DIR})

# Eigen 
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
message("EIGEN3_INCLUDE_DIR: " ${EIGEN3_INCLUDE_DIR})

# TBB
find_package(TBB 4.4 COMPONENTS tbb tbbmalloc)
include_directories(${TBB_INCLUDE_DIR})
message("TBB_INCLUDE_DIR: " ${TBB_INCLUDE_DIR})

# GTSAM
find_package(GTSAM REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})
message("GTSAM_INCLUDE_DIR: " ${GTSAM_INCLUDE_DIR})



catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs tf
)


## Your package locations should be listed before other locations
include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
)


file(GLOB src_files
    "${PROJECT_SOURCE_DIR}/src/*.cpp"
)
add_executable(FusionNode src/FusionNode.cpp ${src_files})


message("gtsam_LIBRARIES: " ${gtsam_LIBRARIES})
target_link_libraries(FusionNode
    gtsam
    ${catkin_LIBRARIES}
    ${gtsam_LIBRARIES}
    ${Boost_LIBRARIES}
)

