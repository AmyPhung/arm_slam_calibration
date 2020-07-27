#!/usr/bin/env python
"""
Sensitivity test for joint errors

Run a sweep of joint offset errors and compute resulting objective function
variance, end effector accuracy, and end effector precision. Saves results to
a CSV file
"""

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
import helper_functions as hf

joint_list = ["shoulder_yaw", "shoulder_pitch", "forearm_pitch", "wrist_pitch", "wrist_yaw"]
offset_increment = 5 # in degrees

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
    df_ee_accuracy = []
    df_ee_precision = []


    for offset in range(0, 361, offset_increment):
        offset_params = copy.deepcopy(gt_params)
        # TODO: Change this
        # Assume shoulder yaw joint is correct
        offset_params.params[0].value = 65322
        offset_params.params[0].min = 0
        offset_params.params[0].max = 2000000
        offset_params.params[0].uncertainty = 0.000000001
        # Assume shoulder pitch joint is correct
        offset_params.params[1].value = 3792
        offset_params.params[1].min = 0
        offset_params.params[1].max = 2000000
        offset_params.params[1].uncertainty = 0.000000001

        # Assume shoulder pitch joint is correct
        offset_params.params[2].value = 59777
        offset_params.params[2].min = 0
        offset_params.params[2].max = 2000000
        offset_params.params[2].uncertainty = 0.000000001

        # Assume shoulder pitch joint is correct
        offset_params.params[3].value = 64437
        offset_params.params[3].min = 0
        offset_params.params[3].max = 2000000
        offset_params.params[3].uncertainty = 0.000000001

        # Assume shoulder pitch joint is correct
        offset_params.params[4].value = 65324
        offset_params.params[4].min = 0
        offset_params.params[4].max = 2000000
        offset_params.params[4].uncertainty = 0.000000001

        print(offset)
        for joint in joint_list:
            # Add error to joint
            offset_params = bridge.addOffsetToJoint(offset_params, joint, offset)

        print(offset_params)
        # Compute variance
        gt_result = bridge.runCalibration(gt_params, calibration_data,
            robot_description, opt_params)
        offset_result = bridge.runCalibration(offset_params, calibration_data,
            robot_description, opt_params)

        rospy.loginfo("Offset Variance: " + str(offset_result.starting_variance))


        # -----------------
        # Compute end effector positions
        effector_frame = String()
        effector_frame.data = "fisheye"
        output_frame = String()
        output_frame.data = "base_link"

        # TODO: remove hardcoded single tag
        joint_states = calibration_data.point_groups[0].joint_states

        gt_ee_positions = bridge.getEndEffectorPosition(joint_states,
            gt_params, robot_description, effector_frame, output_frame)

        offset_ee_positions = bridge.getEndEffectorPosition(joint_states,
            offset_params, robot_description, effector_frame, output_frame)

        # -----------------
        # Compute accuracy and precision
        acc, prec = hf.computeMetrics(gt_ee_positions.output_poses.poses,
                                      offset_ee_positions.output_poses.poses)


        df_joints.append("all")
        df_offsets.append(offset)
        df_variance.append(offset_result.starting_variance)
        df_ee_accuracy.append(acc)
        df_ee_precision.append(prec)

    # Create dataframe
    df_data = [df_joints, df_offsets, df_variance,
               df_ee_accuracy, df_ee_precision]
    print(df_data)
    output_df = pd.DataFrame(df_data, index=['joint_name',
                                             'degrees_offset',
                                             'variance',
                                             'end_effector_accuracy',
                                             'end_effector_precision']).T

    path = "/home/amy/whoi_ws/src/min_variance_calibration/min_variance_calibration/results/sweep_joint_errors/"
    output_df.to_csv(path + "joint_results.csv")
    print(output_df)
