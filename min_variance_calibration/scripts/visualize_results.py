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
import matplotlib.pyplot as plt
import numpy.ma as ma
from numpy.random import uniform, seed
import numpy as np
import yaml
import time
import copy

# Custom Libraries
import calibration_bridge as bridge


def computeDistance(point1, point2):
    """ Compute distance between 2 ROS points
    Args:
        point1 (geometry_msgs/Point): 3 dimensional point
        point2 (geometry_msgs/Point): 3 dimensional point in same frame
    Returns:
        distance (float): Distance between two points
    """
    p1 = np.array([point1.x, point1.y, point1.z])
    p2 = np.array([point2.x, point2.y, point2.z])

    squared_dist = np.sum((p1-p2)**2, axis=0)
    dist = np.sqrt(squared_dist)
    return dist

def computeMetrics(gt_positions, computed_positions):
    """ Compute accuracy and precision between computed points and ground
    truth points

    Args:
        gt_positions (Pose[]): List of ground truth poses
        computed_positions (Pose[]): List of computed poses

    Returns:
        accuracy (double): Average error between points
        variance (double): Variance of error (assuming low variance = consistent offsett)
    """
    errors = []

    for gt_pos, com_pos in zip(gt_positions, computed_positions):
        # print(gt_pos.position)
        # print(com_pos.position)
        errors.append(computeDistance(gt_pos.position, com_pos.position))

    errors = np.array(errors)
    return errors.mean(), errors.var()

if __name__ == "__main__":
    rospy.init_node("visualize_results")

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
    # Create noisy and clean set
    gt_params = bridge.convertToMsg(initial_params)
    noisy_params = bridge.add_param_noise(gt_params, 0.000001)

    # TODO: Change this
    # Assume shoulder yaw joint is correct
    noisy_params.params[0].value = 65322
    # noisy_params.params[0].min = 65321.9999999
    # noisy_params.params[0].max = 65322.0000001
    noisy_params.params[0].uncertainty = 0.000000001
    # Assume shoulder pitch joint is correct
    noisy_params.params[1].value = 3792
    # noisy_params.params[1].min = 3791.999999
    # noisy_params.params[1].max = 3792.0000001
    noisy_params.params[1].uncertainty = 0.000000001

    # Assume shoulder pitch joint is correct
    noisy_params.params[2].value = 59777
    # noisy_params.params[2].min = 59776.999999
    # noisy_params.params[2].max = 59777.0000001
    noisy_params.params[2].uncertainty = 0.000000001

    # Assume shoulder pitch joint is correct
    noisy_params.params[3].value = 64437
    # noisy_params.params[3].min = 64436.999999
    # noisy_params.params[3].max = 64437.0000001
    noisy_params.params[3].uncertainty = 0.000000001

    # Assume shoulder pitch joint is correct
    noisy_params.params[4].value = 65324
    # noisy_params.params[4].min = 65323.999999
    # noisy_params.params[4].max = 65324.0000001
    noisy_params.params[4].uncertainty = 0.000000001


    # # Assume fisheye roll orientation is correct
    # noisy_params.params[13].value = 0
    # noisy_params.params[13].min = -0.0000001
    # noisy_params.params[13].max = 0.0000001
    # noisy_params.params[13].uncertainty = 0.000000001
    # # Assume fisheye pitch orientation is correct
    # noisy_params.params[15].value = 0
    # noisy_params.params[15].min = -0.0000001
    # noisy_params.params[15].max = 0.0000001
    # noisy_params.params[15].uncertainty = 0.000000001

    # noisy_params = copy.deepcopy(gt_params)
    # noisy_params.params[13].value = 4
    # noisy_params.params[14].value = 4
    # noisy_params.params[15].value = 4

    joint_list = ["shoulder_yaw", "shoulder_pitch", "forearm_pitch", "wrist_pitch", "wrist_yaw"]
    for joint in joint_list:
        # Add error to joint
        noisy_params = bridge.addOffsetToJoint(noisy_params, joint, 180)

    # --------------------------------------------------------------------------

    calibration_data = bridge.add_measurement_noise(calibration_data, 0)

    # --------------------------------------------------------------------------
    # Pass data to calibration server
    result = bridge.runCalibration(noisy_params, calibration_data,
        robot_description, opt_params)
    rospy.loginfo("Starting Variance: " + str(result.starting_variance))
    rospy.loginfo("Ending Variance: " + str(result.ending_variance))
    bridge.printParams(result.params)

    # --------------------------------------------------------------------------
    # Publish projected points based on ground truth params

    output_frame = String()
    output_frame.data = "fisheye"

    ground_truth_projection = bridge.projectPoints(calibration_data,
        result.params, robot_description, output_frame)
    uncalibrated_projection = bridge.projectPoints(calibration_data,
        noisy_params, robot_description, output_frame)

    pcl_pub = rospy.Publisher('/projected_points', PointCloud, queue_size=10)
    rospy.sleep(1) # For some reason won't publish without this
    pcl_pub.publish(ground_truth_projection.output_points)
    # rospy.sleep(2)
    # print(ground_truth_projection.output_points)

    no_cal_pcl_pub = rospy.Publisher('/uncalibrated_projected_points', PointCloud, queue_size=10)
    rospy.sleep(1) # For some reason won't publish without this
    no_cal_pcl_pub.publish(uncalibrated_projection.output_points)
    # rospy.sleep(2)
    # print(ground_truth_projection.output_points)

    # # --------------------------------------------------------------------------
    # # Publish end effector positions

    effector_frame = String()
    effector_frame.data = "fisheye"

    output_frame = String()
    output_frame.data = "base_link"

    # TODO: remove hardcoded single tag
    joint_states = calibration_data.point_groups[0].joint_states

    gt_end_effector_positions = bridge.getEndEffectorPosition(joint_states,
        gt_params, robot_description, effector_frame, output_frame)

    initial_end_effector_positions = bridge.getEndEffectorPosition(joint_states,
        noisy_params, robot_description, effector_frame, output_frame)

    final_end_effector_positions = bridge.getEndEffectorPosition(joint_states,
        result.params, robot_description, effector_frame, output_frame)

    acc, prec = computeMetrics(gt_end_effector_positions.output_poses.poses,
                               final_end_effector_positions.output_poses.poses)
    print("Accuracy: " + str(acc))
    print("Precision: " + str(prec))

    gt_pos_pub = rospy.Publisher('/ground_truth_positions', PoseArray, queue_size=10)
    rospy.sleep(1) # For some reason won't publish without this
    gt_pos_pub.publish(gt_end_effector_positions.output_poses)
    # rospy.sleep(2)
    # print(gt_end_effector_positions.output_positions)

    initial_pos_pub = rospy.Publisher('/initial_positions', PoseArray, queue_size=10)
    rospy.sleep(1) # For some reason won't publish without this
    initial_pos_pub.publish(initial_end_effector_positions.output_poses)
    # rospy.sleep(2)
    # print(initial_end_effector_positions.output_positions)

    final_pos_pub = rospy.Publisher('/final_positions', PoseArray, queue_size=10)
    rospy.sleep(1) # For some reason won't publish without this
    final_pos_pub.publish(final_end_effector_positions.output_poses)
    # rospy.sleep(2)
    # print(final_end_effector_positions.output_positions)

    # --------------------------------------------------------------------------
    # Draw lines to connect fisheye with measurements

    marker_pub = rospy.Publisher("/visualization_marker",
    Marker, queue_size=10)

    marker = Marker()
    marker.type = marker.LINE_LIST
    marker.header.frame_id = "base_link"
    marker.scale.x = 0.001
    marker.color.a = 1

    # Draw lines for initial estimate
    for effector, measurement in zip(initial_end_effector_positions.output_poses.poses, uncalibrated_projection.output_points.points):
        # Add effector position
        effector_pt = Point()
        effector_pt.x = effector.position.x
        effector_pt.y = effector.position.y
        effector_pt.z = effector.position.z
        marker.points.append(effector_pt)

        # Add measurement position
        measurement_pt = Point()
        measurement_pt.x = measurement.x
        measurement_pt.y = measurement.y
        measurement_pt.z = measurement.z
        marker.points.append(measurement_pt)

    # Draw lines for final estimate
    for effector, measurement in zip(final_end_effector_positions.output_poses.poses, ground_truth_projection.output_points.points):
        # Add effector position
        effector_pt = Point()
        effector_pt.x = effector.position.x
        effector_pt.y = effector.position.y
        effector_pt.z = effector.position.z
        marker.points.append(effector_pt)

        # Add measurement position
        measurement_pt = Point()
        measurement_pt.x = measurement.x
        measurement_pt.y = measurement.y
        measurement_pt.z = measurement.z
        marker.points.append(measurement_pt)

    # print(uncalibrated_projection.output_points.points)
    rospy.sleep(1)
    marker_pub.publish(marker)
    rospy.sleep(1)

    # Visualize point-by-point movements ----------------------------------------
    js_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

    # Visualize camera as a marker (comment out visual in URDF first) ----------------------------------------
    camera_visual_pub = rospy.Publisher("/camera_visualization_marker",
        Marker, queue_size=10)

    # Display 1 calibration point at a time
    for state in joint_states:
        # Update visual ----------------
        # Available Params:
            # gt_params
            # noisy_params
            # result.params

        current_angles = bridge.convertJointStates(state, result.params)
        js_pub.publish(current_angles)

        # Create Marker ----------------
        marker = Marker()
        marker.type = marker.CUBE
        # NOTE: Computed corrections are made in fisheye frame
        marker.header.frame_id = "fisheye"
        # print(result.params.params[10].name)

        marker.pose.position.x = 0#result.params.params[10].value# + -0.124
        marker.pose.position.y = 0#result.params.params[11].value# + -0.010
        marker.pose.position.z = 0#result.params.params[12].value# + 0.000

        quaternion = tf.transformations.quaternion_from_euler(0,#result.params.params[13].value,# + 2.356,
                                                              0,#result.params.params[14].value,# + 1.571,
                                                              0)#result.params.params[15].value)# + 0.393)
        marker.pose.orientation.x = quaternion[0]
        marker.pose.orientation.y = quaternion[1]
        marker.pose.orientation.z = quaternion[2]
        marker.pose.orientation.w = quaternion[3]

        marker.action = marker.ADD

        marker.scale.x = 0.1
        marker.scale.y = 0.05
        marker.scale.z = 0.2
        marker.color.a = 0.5

        camera_visual_pub.publish(marker)

        # Wait for user ----------------
        print "Press Enter to continue...."
        raw_input()
