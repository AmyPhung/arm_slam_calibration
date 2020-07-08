cmake_minimum_required(VERSION 3.0.2)
project(min_variance_calibration)

find_package(Boost REQUIRED system thread)

find_package(catkin REQUIRED
  COMPONENTS
    geometry_msgs
    kdl_parser
    min_variance_calibration_msgs
    message_filters
    roscpp
    rospy
    rosbag
    sensor_msgs
    std_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    geometry_msgs
    kdl_parser
    min_variance_calibration_msgs
    message_filters
    roscpp
    rospy
    rosbag
    sensor_msgs
    std_msgs
  DEPENDS
    Boost
  LIBRARIES
    min_variance_calibration
)

include_directories(include ${catkin_INCLUDE_DIRS}
                            ${Boost_INCLUDE_DIRS}
)

include(~/dlib/dlib/cmake)



add_library(min_variance_calibration src/ParameterManager.cpp
                                     src/Utils.cpp
                                     src/Optimizer.cpp
                                     src/ChainModel.cpp)

target_link_libraries(min_variance_calibration ${catkin_LIBRARIES}
                                               ${Boost_LIBRARIES}
                                               dlib)


add_executable(RunCalibrationServer src/RunCalibrationServer.cpp)
target_link_libraries(RunCalibrationServer min_variance_calibration
                                           ${catkin_LIBRARIES})

add_executable(ProjectPointsServer src/ProjectPointsServer.cpp)
target_link_libraries(ProjectPointsServer min_variance_calibration
                                          ${catkin_LIBRARIES})
