#!/usr/bin/env python

# ROS
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

# Help find custom libraries
import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0,parentdir)

# Custom Libraries
import calibration_bridge as bridge
import helper_functions as hf


def resetShoulderValues(params):
    # TODO: Change this
    # Assume shoulder yaw joint is correct
    noisy_params.params[0].value = 65322
    noisy_params.params[0].min = 65321.9999999
    noisy_params.params[0].max = 65322.0000001
    noisy_params.params[0].uncertainty = 0.000000001
    # Assume shoulder pitch joint is correct
    noisy_params.params[1].value = 3792
    noisy_params.params[1].min = 3791.999999
    noisy_params.params[1].max = 3792.0000001
    noisy_params.params[1].uncertainty = 0.000000001

    # Assume shoulder pitch joint is correct
    noisy_params.params[2].value = 59777
    noisy_params.params[2].min = 59776.999999
    noisy_params.params[2].max = 59777.0000001
    noisy_params.params[2].uncertainty = 0.000000001

    # Assume shoulder pitch joint is correct
    noisy_params.params[3].value = 64437
    noisy_params.params[3].min = 64436.999999
    noisy_params.params[3].max = 64437.0000001
    noisy_params.params[3].uncertainty = 0.000000001

    # Assume shoulder pitch joint is correct
    noisy_params.params[4].value = 65324
    noisy_params.params[4].min = 65323.999999
    noisy_params.params[4].max = 65324.0000001
    noisy_params.params[4].uncertainty = 0.000000001


    # # Assume fisheye roll orientation is correct
    # params.params[13].value = 0
    # params.params[13].min = -0.0000001
    # params.params[13].max = 0.0000001
    # params.params[13].uncertainty = 0.000000001
    # # Assume fisheye pitch orientation is correct
    # params.params[15].value = 0
    # params.params[15].min = -0.0000001
    # params.params[15].max = 0.0000001
    # params.params[15].uncertainty = 0.000000001

    # params = copy.deepcopy(gt_params)
    # params.params[13].value = 4
    # params.params[14].value = 4
    # params.params[15].value = 4

if __name__ == "__main__":
    rospy.init_node("sweep_noise_levels")

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

    # Create arrays to save data
    xs, ys, acc_arr, prec_arr, results_arr = [], [], [], [], []

    # MODIFY THESE VALUES FOR TESTING
    # Iterate through various levels of parameter and measurement noise
    for x in np.linspace(0.000001, 0.6, 4): # Parameter error (%)
        for y in np.linspace(0, 0.3, 4): # Measurement noise (meters)
            for _ in range(1): # Run multiple times to get averages
                print(x)
                print(y)
                noisy_params = bridge.add_param_noise(gt_params, x)
                resetShoulderValues(noisy_params)
                calibration_data = bridge.add_measurement_noise(calibration_data, y)

                # ---------------
                # Pass data to calibration server
                result = bridge.runCalibration(noisy_params, calibration_data,
                    robot_description, opt_params)

                # Handle case of no convergence
                if result == None:
                    xs.append(x), ys.append(y)
                    acc_arr.append(np.nan), prec_arr.append(np.nan)
                    results_arr.append(np.nan)
                    continue

                rospy.loginfo("Starting Variance: " + str(result.starting_variance))
                rospy.loginfo("Ending Variance: " + str(result.ending_variance))
                bridge.printParams(result.params)

                # -----------------
                # Compute end effector positions
                effector_frame = String()
                effector_frame.data = "fisheye"
                output_frame = String()
                output_frame.data = "base_link"

                # TODO: remove hardcoded single tag
                joint_states = calibration_data.point_groups[0].joint_states

                gt_end_effector_positions = bridge.getEndEffectorPosition(joint_states,
                    gt_params, robot_description, effector_frame, output_frame)

                # initial_end_effector_positions = bridge.getEndEffectorPosition(joint_states,
                #     noisy_params, robot_description, effector_frame, output_frame)

                final_end_effector_positions = bridge.getEndEffectorPosition(joint_states,
                    result.params, robot_description, effector_frame, output_frame)

                # -----------------
                # Compute accuracy and precision
                acc, prec = hf.computeMetrics(gt_end_effector_positions.output_poses.poses,
                                           final_end_effector_positions.output_poses.poses)
                # -----------------
                # Save results
                xs.append(x), ys.append(y)
                acc_arr.append(acc), prec_arr.append(prec)
                results_arr.append(result.ending_variance)

    # --------------------------------------------------------------------------
    # Save results to csv file
    results_file = rospy.get_param('~output_file')
    csv_arr = np.array([xs, ys, acc_arr, prec_arr, results_arr])
    np.savetxt(results_file, csv_arr, delimiter=',',
        header="param_noise, measurement_noise, accuracy, precision, output_variance", comments="")
