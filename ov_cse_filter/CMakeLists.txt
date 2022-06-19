cmake_minimum_required(VERSION 2.8.12)
project(ov_cse_filter)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

add_definitions(-std=c++11)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rosbag
    geometry_msgs
    nav_msgs
    sensor_msgs
)

include_directories(
    include/dynamic_estimator
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

catkin_package(
    CATKIN_DEPENDS roscpp rosbag geometry_msgs nav_msgs sensor_msgs message_runtime
    DEPENDS EIGEN3
)

set(ESTIMATOR_SOURCE_FILES
    src/dynamic_estimator.cpp
    src/v_tools.cpp
)


set(ESTIMATOR_HEADER_FILES
    include/dynamic_estimator/dynamic_estimator.h
    include/dynamic_estimator/v_tools.h
)


###########
## Build ##
###########
add_library(dynamic_estimator_lib ${ESTIMATOR_SOURCE_FILES})
target_link_libraries(dynamic_estimator_lib ${catkin_LIBRARIES} ${EIGEN_LIBS})

add_executable(dynamic_estimator  src/dynamic_estimator_node.cpp)
add_dependencies(dynamic_estimator ${catkin_EXPORTED_TARGETS} )
target_link_libraries(dynamic_estimator dynamic_estimator_lib ${catkin_LIBRARIES} ${EIGEN_LIBS})

add_executable(dynamic_estimator_batch  src/dynamic_estimator_batch.cpp)
add_dependencies(dynamic_estimator_batch ${catkin_EXPORTED_TARGETS} )
target_link_libraries(dynamic_estimator_batch dynamic_estimator_lib ${catkin_LIBRARIES} ${EIGEN_LIBS})
