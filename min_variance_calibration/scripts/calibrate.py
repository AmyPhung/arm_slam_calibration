#!/usr/bin/env python

import calibration_bridge as bridge

import rospy
import rosbag
import yaml
from min_variance_calibration_msgs.msg import OptimizationParameters

if __name__ == "__main__":
    rospy.init_node("calibration_bridge")

    # Load initial parameters from yaml file
    filename = rospy.get_param('~initial_param_yaml')
    initial_params = bridge.loadFromYAML(filename, yaml.SafeLoader)

    # Load calibration data and robot description from bagfile
    bagfile = rospy.get_param('~data_bagfile')
    bag = rosbag.Bag(bagfile)

    calibration_data = None
    robot_description = None

    for topic, msg, t in bag.read_messages(topics=['robot_description']):
        robot_description = msg
    for topic, msg, t in bag.read_messages(topics=['calibration_data']):
        calibration_data = msg
    bag.close()

    # Load optimization params from ROS parameter server
    opt_params = OptimizationParameters()
    opt_params.rho_start = rospy.get_param('~rho_start', 10)
    opt_params.rho_end = rospy.get_param('~rho_end', 1e-6)
    opt_params.npt = len(initial_params.values()) + 2
    opt_params.max_f_evals = rospy.get_param('~max_f_evals', 10000)

    # Pass data to calibration server
    result = bridge.runCalibration(initial_params, calibration_data,
        robot_description, opt_params)
    rospy.loginfo("Starting Variance: " + str(result.starting_variance))
    rospy.loginfo("Ending Variance: " + str(result.ending_variance))
    bridge.printParams(result.params)
