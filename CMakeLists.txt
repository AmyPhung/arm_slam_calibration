cmake_minimum_required(VERSION 3.0.2)
project(joint_calibration)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    message_filters
    sensor_msgs
    robot_calibration
    robot_calibration_msgs
    std_msgs
    geometry_msgs
    message_generation
)

add_message_files(
  FILES
  PointLabeled.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
