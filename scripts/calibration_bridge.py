#!/usr/bin/env python
import rospy
import rosbag

from joint_calibration.msg import ParameterInfo
from joint_calibration.msg import FreeParameters
from joint_calibration.msg import CalibrationMsg
from joint_calibration.msg import OptimizationParameters

# TODO: Add this to dependencies list
import yaml
import time

def loadFromYAML(filename):
    """ Load in parameter info from yaml file

    Args:
        filename (str): yaml file location
    Returns:
        params (dict): Parameter info saved as a dictionary
    """
    with open(filename) as file:
        params = yaml.load(file)
    return params

import yaml
from collections import OrderedDict

def ordered_load(filename, Loader=yaml.Loader, object_pairs_hook=OrderedDict):
    class OrderedLoader(Loader):
        pass
    def construct_mapping(loader, node):
        loader.flatten_mapping(node)
        return object_pairs_hook(loader.construct_pairs(node))
    OrderedLoader.add_constructor(
        yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
        construct_mapping)
    return yaml.load(filename, OrderedLoader)



def preconditionParams(initial_params, rho_start):
    """ Apply scaling term to all parameters so that all parameters
    will need roughly the same amount of adjustment

    Args:
        initial_params (dict): Initial parameter info saved in a dict
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

    print(conditioned_params)
    return conditioned_params

# Create function restoreParams

class CalibrationBridge():
    """
    publisher /calibrate
    subscriber /calibration_results

    runCalibration()
    runTestCalibration()
    sweepNoiseLevels()

    resultsCB()
        restoreParams()
        displayResults (just write it out here)

    TODO: add ros parameters to control modes
    """
    def __init__(self):
        rospy.init_node("CalibrationBridge")
        self.update_rate = rospy.Rate(10)

        self.calibration_pub = rospy.Publisher("/calibrate",
            CalibrationMsg, queue_size=10)

        # self.sensor_fisheye_sub = rospy.Subscriber("/calibration_results",
        #     PointLabeled, self.sensor_fisheye_cb)
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
        free_params = preconditionParams(initial_params, opt_params.rho_start)

        cal_msg = CalibrationMsg()
        cal_msg.free_params = free_params
        cal_msg.opt_params = opt_params
        cal_msg.data = calibration_data
        cal_msg.robot_description = robot_description
        time.sleep(1) # For some reason the publisher doesn't work without this
        self.calibration_pub.publish(cal_msg)

    def runTestCalibration(self, initial_params, calibration_data,
        robot_description, optimization_params,
        param_noise, measurement_noise):
        # Adjust values
        # Run calibration
        pass

if __name__ == "__main__":


    # usage example:
    filename = '/home/amy/whoi_ws/src/joint_calibration/config/initial_params.yaml'
    with open(filename) as file:
        initial_params = ordered_load(file, yaml.SafeLoader)
    # print(a.keys())

    # initial_params = loadFromYAML('/home/amy/whoi_ws/src/joint_calibration/config/initial_params.yaml')
    calibration_data = None
    robot_description = None

    bag = rosbag.Bag('/home/amy/whoi_ws/src/joint_calibration/bags/calibration_data.bag')
    for topic, msg, t in bag.read_messages(topics=['robot_description']):
        robot_description = msg
    for topic, msg, t in bag.read_messages(topics=['calibration_data']):
        calibration_data = msg
    bag.close()

    opt_params = OptimizationParameters()
    opt_params.rho_start = 10
    opt_params.rho_end = 1e-6
    opt_params.npt = len(initial_params.values()) + 2
    opt_params.max_f_evals = 10000

    cal_bridge = CalibrationBridge()
    cal_bridge.runCalibration(initial_params, calibration_data,
        robot_description, opt_params)