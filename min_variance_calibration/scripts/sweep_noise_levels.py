#!/usr/bin/env python

# TODO: add this as a dependency
import numpy as np
from matplotlib.mlab import griddata
import matplotlib.pyplot as plt
import numpy.ma as ma
from numpy.random import uniform, seed
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
    opt_params.rho_end = rospy.get_param('rho_end', 1e-6)
    opt_params.npt = len(initial_params.values()) + 2
    opt_params.max_f_evals = rospy.get_param('max_f_evals', 10000)

    # Load sweep noise levels from ROS parameter server
    # TODO: Make these ROS params
    param_noise_start = 0
    param_noise_end = 1
    param_n_steps = 10
    measurement_noise_start = 0
    measurement_noise_end = 1
    measurement_n_steps = 5

    # Iterate through all combinations of specified noise levels
    x = np.linspace(param_noise_start,
                    param_noise_end,
                    param_n_steps)
    y = np.linspace(measurement_noise_start,
                    measurement_noise_end,
                    measurement_n_steps)

    xv, yv = np.meshgrid(x, y)
    results = np.zeros(xv.shape)

    # for i in range(len(xv)):
    #     for j in range(len(yv)):
    #         noisy_params = add_param_noise(initial_params, xv[i][j])
    #         noisy_measurements = add_measurement_noise(calibration_data, yv[i][j])
    #
    #         # print("here!")
    #         # print(xv[i][j])
    #         # print(yv[i][j])
    #         results[i][j] = i + j
    #         # print(results[i][j])


    # Delete later - just for checking
    noisy_params = add_param_noise(initial_params, 0.1)
    noisy_measurements = add_measurement_noise(calibration_data, 0.01)

    params = noisy_params.keys()

    # print(calibration_data.point_groups[0].observations)
    print(noisy_measurements.point_groups[0].observations)


    # print(initial_params[params[0]]["initial_value"])
    # print(noisy_params[params[0]]["initial_value"])
    #
    # print(xv.shape)
    # print(yv.shape)
    # print(results.shape)

    # for p_noise in param_noise:
    #     for m_noise in measurement_noise:
    #         # Pass data to calibration server
    #         result = bridge.runCalibration(initial_params, calibration_data,
    #             robot_description, opt_params)
    #         rospy.loginfo("Starting Variance: " + str(result.starting_variance))
    #         rospy.loginfo("Ending Variance: " + str(result.ending_variance))
    #         bridge.printParams(result.params)








    # # Save data to output file so it doesn't need re-running
    # # make up some randomly distributed data
    # seed(1234)
    # npts = 200
    # x = uniform(-2,2,npts)
    # y = uniform(-2,2,npts)
    # z = np.random.random(len(x))
    # # define grid.
    # xi = np.linspace(-2.1,2.1,100)
    # yi = np.linspace(-2.1,2.1,100)
    # # grid the data.
    # zi = griddata(x, y, z, xi, yi, interp='linear')
    #
    # # contour the gridded data, plotting dots at the randomly spaced data points.
    # CS = plt.contour(xi,yi,zi,15,linewidths=0.5,colors='k')
    # CS = plt.contourf(xi,yi,zi,15,cmap=plt.cm.jet)
    # plt.colorbar() # draw colorbar
    # # plot data points.
    # plt.scatter(x,y,marker='o',c='b',s=5)
    # plt.xlim(-2,2)
    # plt.ylim(-2,2)
    # plt.title('griddata test (%d points)' % npts)
    # plt.show()

"""
def runTestCalibration(self, initial_params, calibration_data,
    robot_description, optimization_params,
    param_noise, measurement_noise):
    # Adjust values
    # Run calibration
    # TODO: implement this
    pass
"""
