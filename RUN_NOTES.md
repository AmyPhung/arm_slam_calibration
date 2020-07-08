## Easy Handeye Calibration
+ make sure tf in URDF is commented out
+ roscore
+ rosparam set /use_sim_time true
+ rqt_bag --clock
+ roslaunch titan_arm_moveit_config demo.launch
+ roslaunch tagslam apriltag_detector_node.launch
+ roslaunch tagslam tagslam.launch
+ roslaunch easy_handeye calibrate_nui.launch
+ Add output tf to URDF file


rosbag record /calibration_data /camera/fisheye/camera_info /camera/fisheye/image_raw /capture_calibration /joint_states /tf /tf_static /virtual_sensor_base /virtual_sensor_fisheye
