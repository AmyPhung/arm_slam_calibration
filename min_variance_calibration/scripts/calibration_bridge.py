#!/usr/bin/env python
import rospy

from min_variance_calibration_msgs.msg import ParameterInfo
from min_variance_calibration_msgs.msg import FreeParameters
from min_variance_calibration_msgs.msg import OptimizationParameters
from min_variance_calibration_msgs.srv import RunCalibration
from min_variance_calibration_msgs.srv import ProjectPoints
from min_variance_calibration_msgs.srv import GetEndEffectorPosition
from sensor_msgs.msg import JointState

# TODO: Add this to dependencies list
import yaml
import copy
from collections import OrderedDict
import numpy as np
import math

def loadFromYAML(filename, Loader=yaml.Loader, object_pairs_hook=OrderedDict):
    """ Load in parameter info from yaml file

    Args:
        filename (str): yaml file location
    Returns:
        params (OrderedDict): Parameter info saved as a dictionary
    """
    class OrderedLoader(Loader):
        pass
    def construct_mapping(loader, node):
        loader.flatten_mapping(node)
        return object_pairs_hook(loader.construct_pairs(node))
    OrderedLoader.add_constructor(
        yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
        construct_mapping)
    with open(filename) as file:
        params = yaml.load(file, OrderedLoader)
    return params

def convertToMsg(param_dict):
    """ Load in parameter info from yaml file

    Args:
        param_dict (OrderedDict): Parameter info saved as a dictionary
    Returns:
        param_msg (FreeParameters): Parameter info saved as a ROS message
    """

    free_params_msg = FreeParameters()

    for param in param_dict.keys():
        param_msg = ParameterInfo()
        param_msg.name = param
        param_msg.value = param_dict[param]["initial_value"]
        param_msg.min = param_dict[param]["lower_limit"]
        param_msg.max = param_dict[param]["upper_limit"]
        param_msg.uncertainty = param_dict[param]["uncertainty"]
        param_msg.scaling = 1

        free_params_msg.params.append(param_msg)

    return free_params_msg
    # TODO: implement this, modify run calibration to use this

def preconditionParams(initial_params, rho_start):
    """ Apply scaling term to all parameters so that all parameters
    will need roughly the same amount of adjustment

    Args:
        initial_params (FreeParameters): Initial parameter info saved as a ROS
            message
        rho_start (float): Optimization parameter that should equal ~1/10
            the greatest expected change - will be used to scale params
    Returns:
        conditioned_params (FreeParameters): Parameter info scaled and
            saved as ROS message
    """
    conditioned_params = FreeParameters()

    for param in initial_params.params:
        # 10 comes from the fact that rho_start should be 1/10 the
        # uncertainty value. Decimal forces float value
        scaling = (10.0 * rho_start) / param.uncertainty

        param_msg = ParameterInfo()
        param_msg.name        = param.name
        param_msg.value       = param.value * scaling
        param_msg.min         = param.min * scaling
        param_msg.max         = param.max * scaling
        param_msg.uncertainty = param.uncertainty * scaling
        param_msg.scaling     = scaling

        conditioned_params.params.append(param_msg)

    return conditioned_params

def printParams(free_params):
    for param in free_params.params:
        print(param.name + ': ' + str(param.value))


def runCalibration(initial_params, calibration_data,
    robot_description, opt_params):
    """ Provides a convenient interface to the calibration service
    Args:
        initial_params (FreeParameters): Initial parameter info saved as a ROS
            message
        calibration_data (CalibrationData): ROS message containing
            measurements
        robot_description (String): ROS message containing robot
            description
        opt_params (OptimizationParameters): ROS message
            containing optimization parameters
    """
    rospy.wait_for_service('/run_calibration')

    free_params = preconditionParams(initial_params, opt_params.rho_start)

    try:
        run_calibration = rospy.ServiceProxy('/run_calibration',
                                             RunCalibration)

        print(calibration_data)
        result = run_calibration(free_params, opt_params, calibration_data,
                                 robot_description)
        return result
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def projectPoints(input_data, params, robot_description, output_frame):
    """ Provides a convenient interface to the calibration service
    Args:
        input_data (CalibrationData): ROS message containing
            measurements to be projected
        params (FreeParameters): Parameters to use for projection
        robot_description (String): ROS message containing robot
            description
        output_frame (String): ROS message containing desired output
            frame name
    """
    rospy.wait_for_service('/project_points')

    try:
        project_points = rospy.ServiceProxy('/project_points',
                                            ProjectPoints)
        projected_points = project_points(input_data, params,
                                          robot_description, output_frame)
        return projected_points
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def getEndEffectorPosition(joint_states, params, robot_description,
    effector_frame, output_frame):
    """ Provides a convenient interface to the end effector position service
    Args:
        joint_states (list): List of joint angles to use
        params (FreeParameters): Parameters to use for projection
        robot_description (String): ROS message containing robot
            description
        effector_frame (String): ROS message containing end effector frame
        output_frame (String): ROS message containing desired output frame
    """
    rospy.wait_for_service('/get_end_effector_position')

    try:
        get_end_effector_position = rospy.ServiceProxy('/get_end_effector_position',
            GetEndEffectorPosition)
        end_effector_positions = get_end_effector_position(joint_states, params,
            robot_description, effector_frame, output_frame)
        return end_effector_positions
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def convertJointStates(sensor_input, params):
    """ Converts sensor input to joint angles and formats output as message

    Args:
        sensor_input (list): List of sensor inputs to use
        params (FreeParameters): Parameters to use for conversion
    """
    output = JointState()
    output.name = sensor_input.name
    output.header.stamp = rospy.get_rostime()

    for i, pos in enumerate(sensor_input.position):
        if i >= 5: # Handle wrist rotate
            output.position.append(pos)
        else:
            # Assumes parameters are in order, last joint removed
            offset = params.params[i].value
            scaling = params.params[i+5].value

            if scaling == 0:
                angle = 0
            else:
                angle = ((pos - offset) / scaling) * (math.pi / 180)

            output.position.append(angle)

    return output


def add_param_noise(initial_params, noise):
    """ Adds gaussian noise to initial parameters
    Args:
        initial_params (FreeParameters): Ground truth prameter values
        noise (float): Approximate noise level to apply (between 0 and 0.5) -
            will take a percentage of the expected range as noise
    Returns:
        output (FreeParameters): Parameter values with added noise"""
    output = copy.deepcopy(initial_params)

    for param in output.params:
        param_range = param.max - param.min

        # Apply noise
        scaled_noise = float(abs(noise * param_range / 2))

        # Compute noisy param within expected uncertainty levels
        noisy_param = np.random.normal(param.value, scaled_noise)

        # Constrain parameter value
        if noisy_param > param.max:
            noisy_param = param.max
            scaled_noise = param.max - param.value
        elif noisy_param < param.min:
            noisy_param = param.min
            scaled_noise = param.value - param.min

        # Write results to output
        param.value = noisy_param
        param.uncertainty = scaled_noise
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

def convertDegreesToTicks(num_degrees, param_scaling):
    """ Convert input degree offset to number of ticks """
    # TODO: take into account parameter scaling terms (post-preconditioning)
    ticks = num_degrees * param_scaling.value
    return ticks

def addOffsetToJoint(initial_params, joint_name, deg_offset):
    """ Add an offset to a particular joint

    Args:
        initial_params (FreeParameters): Ground truth prameter values
        joint_name (string): Name of joint to add offset to
        offset (float): Degrees of offset to add to the joint name

    Returns:
        output (FreeParameters): Parameter values with added joint offset
    """
    output = copy.deepcopy(initial_params)

    offset = [param for param in output.params if param.name == joint_name + "_offset"]
    scaling = [param for param in output.params if param.name == joint_name + "_scaling"]

    if not offset or not scaling:
        print(joint_name + " not found in parameter list in addOffsetToJoint")
        return output

    # Convert degrees to ticks
    ticks = convertDegreesToTicks(deg_offset, scaling[0])
    offset[0].value += ticks

    return output
