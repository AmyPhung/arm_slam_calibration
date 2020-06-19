#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState


def state_cb(msg):
    adjustments = [-0.0628366,
                   -0.232881,
                   0.348819,
                   -0.522432,
                   -0.569195,
                   0]

    msg.position = [i + j for i, j in zip(adjustments, msg.position)]
    state_pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("JointRepublisher")

    state_sub  = rospy.Subscriber("/original_joint_states",
        JointState, state_cb)

    state_pub = rospy.Publisher("/joint_states",
        JointState, queue_size=10)

    rospy.spin()
