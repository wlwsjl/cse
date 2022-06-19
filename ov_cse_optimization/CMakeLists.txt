cmake_minimum_required(VERSION 2.8.3)
project(ov_cse_optimization)

set(CMAKE_BUILD_TYPE "RelWithDebInfo") # Release
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rosbag
    std_msgs
    geometry_msgs
    nav_msgs
    tf
    cv_bridge
    )

find_package(OpenCV REQUIRED)

message(WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")

find_package(Ceres REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

catkin_package()

add_executable(estimator_node
    src/estimator_node.cpp
    src/estimator.cpp
    src/parameters.cpp
    src/factor/pose_local_parameterization.cpp
    src/factor/marginalization_factor.cpp
    )

target_link_libraries(estimator_node ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})

add_executable(estimator_batch
    src/estimator_batch.cpp
    src/estimator.cpp
    src/parameters.cpp
    src/factor/pose_local_parameterization.cpp
    src/factor/marginalization_factor.cpp
    )

target_link_libraries(estimator_batch ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})
