#!/usr/bin/env python
import rospy

from joint_calibration.msg import ParameterInfo
from joint_calibration.msg import FreeParameters

# TODO: Add this to dependencies list
import yaml

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
    """

if __name__ == "__main__":
    params = loadFromYAML('/home/amy/whoi_ws/src/joint_calibration/config/initial_params.yaml')
    rho_start = 10

    a = preconditionParams(params, rho_start)
    print(a)
