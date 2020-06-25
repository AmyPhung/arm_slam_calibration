# Joint Calibration

## Build Status
[![Build Status](https://api.travis-ci.com/AmyPhung/joint_calibration.svg?branch=master)](https://travis-ci.com/github/AmyPhung/joint_calibration)


## Build Dependencies
+ https://github.com/davisking/dlib.git (clone to home directory)


## Setup
+ `cd ~/catkin_ws/src`
+ `git clone https://github.com/AmyPhung/joint_calibration`
+ `cd ~/catkin_ws/`
+ `rosdep install -iry --from-paths src`

## Usage
### Recording calibration dataset
+ `roscore`
+ need to set sim time to true before starting `rosparam set use_sim_time true`
+ `rqt_bag --clock` start running bag
+ Launch arm (needed for arm frames) `roslaunch titan_arm_moveit_config demo.launch` Wait till it loads
+ `roslaunch joint_calibration bringup.launch`
+ `roslaunch joint_calibration capture_calibration_data.launch`
+ Wait till virtual tags show up in RVIZ before collecting points

### Calibrate
+ Restart the roscore
    + Note: need to make sure there are no stray params in the /calibrate/ namespace (this might happen if the launch file is used twice with the same roscore)
+ `roslaunch joint_calibration calibrate.launch`

### Previewing results
+ Add ceres output to `joint_state_republisher.py`
+ Start core processes
```
roscore
rosparam set use_sim_time true
roslaunch titan_arm_moveit_config demo.launch
roslaunch joint_calibration bringup.launch
roslaunch joint_calibration connect_tf_trees.launch
```
+ Play back rosbag, remap joint_states to a different topic
```
rosbag play merged.bag /joint_states:=/original_joint_states --clock --topics /camera/fisheye/camera_info /camera/fisheye/image_raw /joint_states
```
+ Start joint republisher
```
rosrun joint_calibration joint_state_republisher.py
```

### Other launch files
+ `bringup.launch` - Starts up essential processes for joint_calibration
+ `connect_tf_trees.launch` - Publish tf between fisheye -> virtual tags and base_link -> ground truth tags to conenct everything to one tf tree

## Misc useful notes

### Play bag data from cmd line
+ `rosbag play merged.bag --clock --topics /camera/fisheye/camera_info /camera/fisheye/image_raw /joint_states`
+ For testing`rosbag play merged.bag /joint_states:=/original_joint_states --clock --topics /camera/fisheye/camera_info /camera/fisheye/image_raw /joint_states`


### Debugging
+ `rosrun tf view_frames`
+ `killall -9 rviz`
+ `killall -9 roscore`
+ `killall -9 rosmaster`
+ `rosmsg show PointLabeled`
+ known bug: when ROS time jumps backwards (i.e. when the bag restarts), the tfs don't like that - when this happens, just close everything and re-run

### TODO:
+ Document launch files, scripts
+ Add node args
+ Add args for sim time
+ create shared functions
+ redo parameter loading
+ fix build
