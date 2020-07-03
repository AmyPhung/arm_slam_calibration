#!/usr/bin/env python

# TODO: add this as a dependency
import numpy as np
import copy

import calibration_bridge as bridge

import rospy
import rosbag
import yaml
from min_variance_calibration_msgs.msg import OptimizationParameters

def add_param_noise(initial_params, noise):
    """ Adds gaussian noise to initial parameters
    Args:
        noise (float): Approximate percent error to apply (between 0 and 1) """
    output = copy.deepcopy(initial_params)

    params = output.keys()

    for p in params:
        # Apply noise
        scaled_noise = float(abs(noise * output[p]["initial_value"]))

        # Compute noisy param within expected uncertainty levels
        noisy_param = np.random.normal(output[p]["initial_value"], scaled_noise)

        # Write results to output
        output[p]["initial_value"] = noisy_param
        output[p]["uncertainty"] = scaled_noise
    return output

def add_measurement_noise(calibration_data, noise):
    """ Adds gaussian noise to measurements in calibration data
    Args:
        noise (float): Approximate error to apply (in meters, will be applied
            per-axis) """
    output = copy.deepcopy(calibration_data)

    # Iterate through all point groups and points
    for pg in output.point_groups:
        for pt in pg.observations:
            pt.point.x = np.random.normal(pt.point.x, noise)
            pt.point.y = np.random.normal(pt.point.y, noise)
            pt.point.z = np.random.normal(pt.point.z, noise)

    return output

if __name__ == "__main__":
    rospy.init_node("sweep_noise_levels")

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
    opt_params.max_f_evals = 100000 # rospy.get_param('~max_f_evals', 10)

    # Load sweep noise levels from ROS parameter server
    # TODO: Make these ROS params
    # TODO: handle 0s
    param_noise_start = 0.0001 # Starting percentage
    param_noise_end = 1   # Ending percentage
    param_n_steps = 3    # Number of points
    measurement_noise_start = 0.0001 # Starting noise (in meters)
    measurement_noise_end = 0.5   # Ending noise (in meters)
    measurement_n_steps = 3     # Number of points

    # Iterate through all combinations of specified noise levels
    x = np.linspace(param_noise_start,
                    param_noise_end,
                    param_n_steps)
    y = np.linspace(measurement_noise_start,
                    measurement_noise_end,
                    measurement_n_steps)

    xv, yv = np.meshgrid(x, y)
    results = np.zeros(xv.shape)

    for i in range(len(xv)):
        for j in range(len(yv)):
            noisy_params = add_param_noise(initial_params, xv[i][j])
            noisy_measurements = add_measurement_noise(calibration_data, yv[i][j])

            # print("here!")
            # print(xv[i][j])
            # print(yv[i][j])
            result = bridge.runCalibration(noisy_params, noisy_measurements,
                    robot_description, opt_params)


            # results[i][j] = i + j
            # print(results[i][j])

    # Save results to csv file
    results_file = rospy.get_param('~output_file')
    csv_arr = np.array([xv.flatten(), yv.flatten(), results.flatten()])
    np.savetxt(results_file, csv_arr, delimiter=',',
        header="param_noise, measurement_noise, output_variance", comments="")


    #
    # # Delete later - just for checking
    # noisy_params = add_param_noise(initial_params, 0.1)
    # noisy_measurements = add_measurement_noise(calibration_data, 0.01)
    #
    # params = noisy_params.keys()
    #
    # # print(calibration_data.point_groups[0].observations)
    # print(noisy_measurements.point_groups[0].observations)
    #
    #
    # # print(initial_params[params[0]]["initial_value"])
    # # print(noisy_params[params[0]]["initial_value"])
    # #
    # # print(xv.shape)
    # # print(yv.shape)
    # # print(results.shape)
    #
    # # for p_noise in param_noise:
    # #     for m_noise in measurement_noise:
    # #         # Pass data to calibration server
    # #         result = bridge.runCalibration(initial_params, calibration_data,
    # #             robot_description, opt_params)
    # #         rospy.loginfo("Starting Variance: " + str(result.starting_variance))
    # #         rospy.loginfo("Ending Variance: " + str(result.ending_variance))
    # #         bridge.printParams(result.params)






"""
def runTestCalibration(self, initial_params, calibration_data,
    robot_description, optimization_params,
    param_noise, measurement_noise):
    # Adjust values
    # Run calibration
    # TODO: implement this
    pass
"""
