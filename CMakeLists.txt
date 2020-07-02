cmake_minimum_required(VERSION 3.0.2)
project(joint_calibration)

find_package(Boost REQUIRED system thread)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    rosbag
    message_filters
    sensor_msgs
    std_msgs
    geometry_msgs
    message_generation
    kdl_parser
)

add_message_files(
  FILES
  PointLabeled.msg
  PointGroup.msg
  CalibrationData.msg
  ParameterInfo.msg
  FreeParameters.msg
  OptimizationParameters.msg
  CalibrationMsg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime
  DEPENDS Boost
  LIBRARIES joint_calibration
)

include_directories(include ${catkin_INCLUDE_DIRS}
                            ${Boost_INCLUDE_DIRS}
)

include(~/dlib/dlib/cmake)



add_library(joint_calibration src/ParameterManager.cpp
                              src/Utils.cpp
                              src/Optimizer.cpp
                              src/ChainModel.cpp)

target_link_libraries(joint_calibration ${catkin_LIBRARIES}
                                        ${Boost_LIBRARIES}
                                        dlib)


add_executable(Calibrate src/Calibrate.cpp)
target_link_libraries(Calibrate joint_calibration
                                ${catkin_LIBRARIES})
