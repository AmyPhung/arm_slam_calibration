#!/usr/bin/env python
import roslib
import rospy
import tf2_ros
from std_msgs.msg import Bool

import numpy as np
import math


def is_tag_frame(frame):
    """ Returns True if input frame is a tag frame (starts with tag_),
    False otherwise

    Args:
        frame (str): Frame name

    Returns:
        is_tag (bool): Whether or not the input frame is a tag frame """

    if frame[0:4] == "tag_":
        return True
    else:
        return False


# TODO: Make these settable args in a launch file
BASE_FRAME = "base_link"
FRAME2a = "fisheye"
FRAME2b = "cam0"
TIMEOUT = 5 # number of seconds to wait for tf

class GroundTruthCreator():
    def __init__(self):
        rospy.init_node("GroundTruthCreator")
        self.update_rate = rospy.Rate(1)

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0))
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.capture_sub  = rospy.Subscriber("/capture_ground_truth", Bool,
                                             self.capture_callback)

        self.tag_frames = []
        self.complete = False

    def capture_callback(self, msg):
        if msg.data == True:
            self.save_tag_tfs()
            self.complete = True

    def list_tag_frames(self):
        """ Filters through tf frames and returns a list only containing frames
        that represent AprilTags """
        frame_list = self.tf_buffer._getFrameStrings()
        return filter(is_tag_frame, frame_list)


    def save_tag_tfs(self):
        """ Save transforms from base frame to each of the tags in a yaml
        file """
        rospy.loginfo("Saving tag transforms")

        tags_dict = {}

        for tag in self.tag_frames:
            virtual_tag = "virtual_" + tag

            virtual_tf = self.tf_buffer.lookup_transform("base_link", virtual_tag,
                                                        rospy.Time(0))
            print(virtual_tf)
        #
        # print(self.tf_buffer._getFrameStrings())
        #
        # tags_dict = {}
        #
        # for tag in self.tag_frames:
        #     virtual_name = "/virtual_" + tag
        #     # tag_584
        #
        #     # try:
        #     (trans, rot) = self.tf_buffer.lookupTransform(virtual_name, "base_link",
        #                                             rospy.Time(0))
        #     print((trans,rot))
        #         # (trans, rot) = self.listener.lookupTransform(virtual_name,
        #         #     BASE_FRAME, rospy.Time(0))
        #         # tags_dict[virtual_name] = (trans, rot)
        #         # print(tags_dict)
        #     # except: #(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #     #     # print("Frame %s couldn't be found", virtual_name)
        #     #     continue
        #
        #     # try:
        #     #     (trans, rot) = self.listener.waitForTransform(virtual_name,
        #     #         BASE_FRAME, rospy.Time(0), rospy.Duration(TIMEOUT))
        #     #     tags_dict[virtual_name] = (trans, rot)
        #     #
        #     # except: #(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #     #     print("Frame %s couldn't be found", virtual_name)
        #     #     continue
        #
        # # print(tags_dict)

    def broadcast_tag_tfs(self):
        """ Broadcasts tag locations with respect to base frame """
        for tag in self.tag_frames:
            try:
                tag_tf = self.tf_buffer.lookup_transform(tag, FRAME2b,
                                                        rospy.Time(0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                print("Frame %s couldn't be found", tag)
                continue

            new_tag_frame = "virtual_" + tag

            # Change tag tf names & re-broadcast
            tag_tf.header.frame_id = FRAME2a
            tag_tf.child_frame_id = new_tag_frame
            self.broadcaster.sendTransform(tag_tf)


    def run(self):
        while not rospy.is_shutdown():
            if self.list_tag_frames() != None:
                self.tag_frames = self.list_tag_frames()



            print(self.tag_frames)
            self.broadcast_tag_tfs()
            self.update_rate.sleep()
            # self.save_tag_tfs()

            if self.complete == True:
                break




if __name__ == '__main__':
    gtc = GroundTruthCreator()
    gtc.run()
