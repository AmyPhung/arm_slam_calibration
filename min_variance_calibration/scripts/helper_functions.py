#!/usr/bin/env python
import numpy as np


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
