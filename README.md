# Minimum Variance Calibration

## Build Status
[![Build Status](https://api.travis-ci.com/AmyPhung/min_variance_calibration.svg?branch=master)](https://travis-ci.com/github/AmyPhung/min_variance_calibration)

## Build Dependencies
+ https://github.com/davisking/dlib.git (clone to home directory)

## Setup
+ `cd ~/catkin_ws/src`
+ `git clone https://github.com/AmyPhung/min_variance_calibration`
+ `cd ~/catkin_ws/`
+ `rosdep install -iry --from-paths src`
+ `catkin build`

## Usage
+ Requires properly formatted initial params yaml file, bag containing calibration data & robot_description
+ `rosrun min_variance_calibration RunCalibrationServer`
+ `rosrun min_variance_calibration calibration_bridge.py`

### Debugging
+ `rosrun tf view_frames`
+ `killall -9 rviz`
+ `killall -9 roscore`
+ `killall -9 rosmaster`
+ `rosmsg show PointLabeled`
+ known bug: when ROS time jumps backwards (i.e. when the bag restarts), the tfs don't like that - when this happens, just close everything and re-run
+ To debug docker build
    + `sudo docker ps -a`
    + `sudo docker commit 6934ada98de6` (change number based on ps -a output)
    + `sudo docker run -it 7015687976a4 bash -il` (change number based on commit output)
    + `source /opt/ros/melodic/setup.bash` (needed to run catkin_make)
    + after navigating to correct directory `catkin_make`
    + `Ctrl + d` to logout


### TODO:
+ Document launch files, scripts
+ Add node args
+ Add args for sim time/make sure it works realtime
+ create shared functions
+ synchronize calibration data collection
+ create launch files
+ document data formats
