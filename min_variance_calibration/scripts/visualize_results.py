#!/usr/bin/env python

# ROS
import rospy
import rosbag
from min_variance_calibration_msgs.msg import OptimizationParameters
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud

# Python Libraries
from matplotlib.mlab import griddata
import matplotlib.pyplot as plt
import numpy.ma as ma
from numpy.random import uniform, seed
import numpy as np
import yaml
import time

# Custom Libraries
import calibration_bridge as bridge

if __name__ == "__main__":
    rospy.init_node("visualize_results")

    # Run calibration ----------------------------------------------------------
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

    # Pass data to calibration server
    result = bridge.runCalibration(initial_params, calibration_data,
        robot_description, opt_params)
    rospy.loginfo("Starting Variance: " + str(result.starting_variance))
    rospy.loginfo("Ending Variance: " + str(result.ending_variance))
    bridge.printParams(result.params)

    # --------------------------------------------------------------------------

    output_frame = String()
    output_frame.data = "fisheye"

    ground_truth_projection = bridge.projectPoints(calibration_data,
        result.params, robot_description, output_frame)

    pcl_pub = rospy.Publisher('/projected_points', PointCloud, queue_size=10)
    rospy.sleep(1) # For some reason won't publish without this
    # pcl_pub.publish(ground_truth_projection.output_points)
    # # rospy.sleep(2)
    # print(ground_truth_projection.output_points)

    # --------------------------------------------------------------------------

    effector_frame = String()
    effector_frame.data = "fisheye"

    output_frame = String()
    output_frame.data = "base_link"

    # TODO: remove hardcoded single tag
    joint_states = calibration_data.point_groups[0].joint_states

    end_effector_positions = bridge.getEndEffectorPosition(joint_states,
        result.params, robot_description, effector_frame, output_frame)

    gt_pos_pub = rospy.Publisher('/ground_truth_positions', PointCloud, queue_size=10)
    rospy.sleep(1) # For some reason won't publish without this
    gt_pos_pub.publish(end_effector_positions.output_positions)
    # rospy.sleep(2)
    print(end_effector_positions.output_positions)





    ### UNCOMMENT TO VIEW CONTOUR GRAPH
    # filename = "/home/amy/whoi_ws/src/min_variance_calibration/min_variance_calibration/results/results.csv"
    #
    # (x, y, z) = np.loadtxt(open(filename, "rb"), delimiter=",", skiprows=1)
    #
    # # z = z - y # Uncomment for normalization
    # print(x)
    # print(x.min())
    #
    # # Save data to output file so it doesn't need re-running
    # # make up some randomly distributed data
    # # seed(1234)
    # npts = 200
    # # x = uniform(-2,2,npts)
    # # y = uniform(-2,2,npts)
    # # z = np.random.random(len(x))
    #
    # # define grid.
    # xi = np.linspace(x.min(),x.max(),100)
    # yi = np.linspace(y.min(),y.max(),100)
    # # grid the data.
    # zi = griddata(x, y, z, xi, yi, interp='linear')
    #
    # # contour the gridded data, plotting dots at the randomly spaced data points.
    # CS = plt.contour(xi,yi,zi,20,linewidths=0.5,colors='k')
    # CS = plt.contourf(xi,yi,zi,20,cmap=plt.cm.jet)
    # plt.colorbar() # draw colorbar
    # # plot data points.
    # plt.scatter(x,y,marker='o',c='b',s=5)
    # # plt.xlim(-2,2)
    # # plt.ylim(-2,2)
    # plt.xlabel("Parameter noise (percentage)")
    # plt.ylabel("Measurement noise (meters)")
    # plt.title('griddata test (%d points)' % npts)
    # plt.show()
