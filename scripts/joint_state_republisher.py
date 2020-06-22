#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState


def state_cb(msg):
    scaling = [0.0380094,
              -0.286683,
              -0.121535,
              -0.0561879,
              -0.126367,
              0]


    offsets = [-0.830455,
               -0.909553,
               0.721249,
               -0.458984,
               -0.457727,
               0]

    msg.position = [i*j + k for i,j,k in zip(scaling, msg.position, offsets)]
    state_pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("JointRepublisher")

    state_sub  = rospy.Subscriber("/original_joint_states",
        JointState, state_cb)

    state_pub = rospy.Publisher("/joint_states",
        JointState, queue_size=10)

    rospy.spin()
