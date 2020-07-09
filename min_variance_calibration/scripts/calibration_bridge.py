#!/usr/bin/env python
import rospy


from min_variance_calibration_msgs.msg import ParameterInfo
from min_variance_calibration_msgs.msg import FreeParameters
from min_variance_calibration_msgs.msg import OptimizationParameters
from min_variance_calibration_msgs.srv import RunCalibration
from min_variance_calibration_msgs.srv import ProjectPoints
from min_variance_calibration_msgs.srv import GetEndEffectorPosition

# TODO: Add this to dependencies list
import yaml
from collections import OrderedDict

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

def preconditionParams(initial_params, rho_start):
    """ Apply scaling term to all parameters so that all parameters
    will need roughly the same amount of adjustment

    Args:
        initial_params (OrderedDict): Initial parameter info saved in a dict
        rho_start (float): Optimization parameter that should equal ~1/10
            the greatest expected change - will be used to scale params
    Returns:
        conditioned_params (FreeParameters): Parameter info scaled and
            saved as ROS-compatible message
    """
    conditioned_params = FreeParameters()

    for param in initial_params.keys():
        print(param)
        print(initial_params[param]["uncertainty"])
        # 10 comes from the fact that rho_start should be 1/10 the
        # uncertainty value. Decimal forces float value
        scaling = (10.0 * rho_start) / initial_params[param]["uncertainty"]

        param_msg = ParameterInfo()
        param_msg.name = param
        param_msg.value = initial_params[param]["initial_value"] * scaling
        param_msg.min = initial_params[param]["lower_limit"] * scaling
        param_msg.max = initial_params[param]["upper_limit"] * scaling
        param_msg.uncertainty = initial_params[param]["uncertainty"] * scaling
        param_msg.scaling = scaling

        conditioned_params.params.append(param_msg)

    return conditioned_params

def printParams(free_params):
    for param in free_params.params:
        print(param.name + ': ' + str(param.value))


def runCalibration(initial_params, calibration_data,
    robot_description, opt_params):
    """ Provides a convenient interface to the calibration service
    Args:
        initial_params (dict): Initial parameter info saved in a dict
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
