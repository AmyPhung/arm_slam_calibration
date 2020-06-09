cmake_minimum_required(VERSION 3.0.2)
project(arm_slam_calibration)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(calibrate src/calibrate.cpp)
target_link_libraries(calibrate ${catkin_LIBRARIES})