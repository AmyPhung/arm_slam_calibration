cmake_minimum_required(VERSION 3.0.2)
project(joint_calibration)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    message_filters
    apriltag_ros
    apriltag_msgs
    sensor_msgs
    robot_calibration
    robot_calibration_msgs
)

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(calibrate src/calibrate.cpp)
target_link_libraries(calibrate ${catkin_LIBRARIES})
