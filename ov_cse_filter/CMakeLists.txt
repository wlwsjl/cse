cmake_minimum_required(VERSION 2.8.12)
project(ov_cse_filter)

# set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

add_definitions(-std=c++11)

find_package(Eigen3 REQUIRED)
find_package(OpenCV 3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rosbag
    geometry_msgs
    nav_msgs
    sensor_msgs
    cv_bridge
    image_transport
    message_generation
)

include_directories(
    include/dynamic_estimator
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

add_message_files(
    FILES
    PointId.msg
    PointIdArray.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(
    CATKIN_DEPENDS roscpp rosbag geometry_msgs nav_msgs sensor_msgs cv_bridge image_transport message_runtime
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
target_link_libraries(dynamic_estimator dynamic_estimator_lib ${catkin_LIBRARIES} ${EIGEN_LIBS})

add_executable(message_handler  src/message_handler.cpp)
target_link_libraries(message_handler ${catkin_LIBRARIES})

add_executable(sub_pose  src/sub_pose.cpp)
target_link_libraries(sub_pose ${catkin_LIBRARIES})

add_executable(sub_gps  src/sub_gps.cpp)
target_link_libraries(sub_gps ${catkin_LIBRARIES})

add_executable(detector  src/detector.cpp)
target_link_libraries(detector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${EIGEN_LIBS})

add_executable(detector_global  src/detector_global.cpp)
target_link_libraries(detector_global ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${EIGEN_LIBS})

add_executable(pos_from_py  src/pos_from_py.cpp)
target_link_libraries(pos_from_py ${catkin_LIBRARIES} ${EIGEN_LIBS})