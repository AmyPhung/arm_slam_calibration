#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState


def state_cb(msg):
    # From new optimizer
    offsets =  [3.24032,
                -4.1243,
                3.55843,
                -4.05532,
                -1.87703,
                0]

    scaling = [-0.337734,
               -0.0307891,
                0.0827869,
                0.0348737,
               -0.0507747,
               0]
    # From robot_calibration
    # scaling = [0.0683768,
    #           -0.175106,
    #           -0.077308,
    #           -0.010065,
    #           -0.032861,
    #           0]
    #
    #
    # offsets = [ -0.716194, #-0.830455,
    #            -0.963132, #-0.909553,
    #            1.38354, #0.721249,
    #            -0.94549, #-0.458984,
    #            0.10511, #-0.457727,
    #            0]
    #
    msg.position = [i*j + k for i,j,k in zip(scaling, msg.position, offsets)]
    state_pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("JointRepublisher")

    state_sub  = rospy.Subscriber("/original_joint_states",
        JointState, state_cb)

    state_pub = rospy.Publisher("/joint_states",
        JointState, queue_size=10)

    rospy.spin()
