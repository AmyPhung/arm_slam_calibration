#!/usr/bin/env python
import rospy
import rosbag

from min_variance_calibration_msgs.msg import ParameterInfo
from min_variance_calibration_msgs.msg import FreeParameters
from min_variance_calibration_msgs.msg import OptimizationParameters
from min_variance_calibration_msgs.srv import RunCalibration

# TODO: Add this to dependencies list
import yaml
import time
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


class CalibrationBridge():
    """ Provides a convenient interface to the calibration service """
    def __init__(self):
        pass

    def runCalibration(self, initial_params, calibration_data,
        robot_description, opt_params):
        """
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

    def runTestCalibration(self, initial_params, calibration_data,
        robot_description, optimization_params,
        param_noise, measurement_noise):
        # Adjust values
        # Run calibration
        # TODO: implement this
        pass

if __name__ == "__main__":
    rospy.init_node("calibration_bridge")

    # Load initial parameters from yaml file
    filename = rospy.get_param('~initial_param_yaml')
    initial_params = loadFromYAML(filename, yaml.SafeLoader)

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
    cal_bridge = CalibrationBridge()
    result = cal_bridge.runCalibration(initial_params, calibration_data,
        robot_description, opt_params)
    rospy.loginfo("Starting Variance: " + str(result.starting_variance))
    rospy.loginfo("Ending Variance: " + str(result.ending_variance))
    printParams(result.params)
