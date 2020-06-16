# Joint Calibration

## Build Status
[![Build Status](https://api.travis-ci.com/AmyPhung/joint_calibration.svg?branch=master)](https://travis-ci.com/github/AmyPhung/joint_calibration)


## Build Dependencies
+ https://github.com/mikeferguson/robot_calibration/tree/master/robot_calibration
## Exec Dependencies:
+ https://bitbucket.org/droplabumich/easy_handeye.git


## Setup
+ `cd ~/catkin_ws/src`
+ `git clone https://github.com/AmyPhung/joint_calibration`
+ `cd ~/catkin_ws/`
+ `rosdep install -iry --from-paths src`

## Usage
### Making the ground-truth tf tree
+ `roscore`
+ need to set sim time to true before starting `rosparam set use_sim_time true`
+ `rqt_bag --clock` start running bag
+ Launch arm (needed for arm frames) `roslaunch titan_arm_moveit_config demo.launch` Wait till it loads
+ `roslaunch joint_calibration set_ground_truth.launch`


### Debugging ground-truth code:
+ Launch arm (needed for arm frames) `roslaunch titan_arm_moveit_config demo.launch`
+ Start apriltag detection `roslaunch tagslam apriltag_detector_node.launch`
+ Start tagslam `roslaunch tagslam tagslam.launch`

## Testing virtual-sensors
+ `roscore`
+ need to set sim time to true before starting `rosparam set use_sim_time true`
+ `rqt_bag --clock` start running bag
+ Launch arm (needed for arm frames) `roslaunch titan_arm_moveit_config demo.launch` Wait till it loads
+ `roslaunch joint_calibration virtual_sensor_node.launch`

## Record calibration dataset
+ `roscore`
+ need to set sim time to true before starting `rosparam set use_sim_time true`
+ `rqt_bag --clock` start running bag
+ Launch arm (needed for arm frames) `roslaunch titan_arm_moveit_config demo.launch` Wait till it loads
+ `roslaunch joint_calibration capture_calibration_data.launch`


## Calibrate
+ `roslaunch joint_calibration calibrate.launch`


## Play bag data from cmd line
+ `rosbag play merged.bag --clock --topics /camera/fisheye/camera_info /camera/fisheye/image_raw /joint_states`
+ For testing`rosbag play merged.bag /joint_states:=/fake_joint_states --clock --topics /camera/fisheye/camera_info /camera/fisheye/image_raw /joint_states`
+ `rosrun joint_calibration fake_joints.py`

## Debugging
+ `rosrun tf view_frames`
+ `killall -9 rviz`
+ `killall -9 roscore`
+ `killall -9 rosmaster`
+ `rosmsg show PointLabeled`
+ known bug: when ROS time jumps backwards (i.e. when the bag restarts), the tfs don't like that - when this happens, just close everything and re-run

## TODO:
+ Document launch files, scripts
+ Add node args
+ Add args for sim time
+ fix build
