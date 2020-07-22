#!/usr/bin/env python

import rospy
import rosbag
import tf
from min_variance_calibration_msgs.msg import OptimizationParameters
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState


# Python Libraries
from matplotlib.mlab import griddata
import matplotlib.pyplot as plt
import numpy.ma as ma
from numpy.random import uniform, seed
import numpy as np
import yaml
import time
import copy
import pandas as pd

# Custom Libraries
import calibration_bridge as bridge

joint_list = ["shoulder_yaw", "shoulder_pitch", "forearm_pitch", "wrist_pitch", "wrist_yaw"]
offset_increment = 5 # in degrees


"""
add error to joint angle
run calibration
get initial variance = the net effect
compute variance for incorrect


pandas dataframe

joint_name degrees_offset variance
"""

if __name__ == "__main__":
    rospy.init_node("sweep_joint_errors")

    # Load data and optimization params ----------------------------------------
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
    opt_params.max_f_evals = rospy.get_param('~max_f_evals', 20000)

    # --------------------------------------------------------------------------
    # Create ground truth params
    gt_params = bridge.convertToMsg(initial_params)

    # Initialize lists (will be saved in dataframe later)
    df_joints = []
    df_offsets = []
    df_variance = []

    for joint in joint_list:
        for offset in range(0, 180, offset_increment):
            # Add error to joint
            offset_params = bridge.addOffsetToJoint(gt_params, joint, offset)

            # Compute variance
            gt_result = bridge.runCalibration(gt_params, calibration_data,
                robot_description, opt_params)
            offset_result = bridge.runCalibration(offset_params, calibration_data,
                robot_description, opt_params)

            rospy.loginfo("Offset Variance: " + str(offset_result.starting_variance))

            df_joints.append(joint)
            df_offsets.append(offset)
            df_variance.append(offset_result.starting_variance)

    # Create dataframe
    df_data = [df_joints, df_offsets, df_variance]
    print(df_data)
    output_df = pd.DataFrame(df_data, index=['joint_name',
                                             'degrees_offset',
                                             'variance']).T

    output_df.to_csv("joint_results.csv")
    print(output_df)
