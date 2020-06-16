#!/usr/bin/env python

import roslib
import rosbag
import rospy
import math
import tf2_ros
from sensor_msgs.msg import PointCloud2
from joint_calibration.msg import PointLabeled


class VirtualSensors():
    """

    load bag file
    publish static transform from base_link to tag tfs

    self.tag_frames = **SOMEHOW LOOK UP LIST OF TAGS**

    self.sensor_base_pub = Publisher
    self.sensor_fisheye_pub = Publisher

    def run(self):
        while not rospy.is_shutdown():
            base_pcl = self.computeBasePoints()
            fisheye_pcl = self.computeFisheyePoints()

            self.sensor_base_pub.publish(base_pcl)
            self.sensor_fisheye_pub.publish(fisheye_pcl)

    def computeBasePoints(self):
        base_tf = lookupTransform
        self.reformat_as_pcl(tf)

    def computeFisheyePoints(self):
    """
    def __init__(self):
        rospy.init_node("GroundTruthCreator")
        self.update_rate = rospy.Rate(10)

        bag_name = "/home/amy/whoi_ws/src/joint_calibration/bags/tag_ground_truth.bag" # TODO: ROS param this
        self.base_frame = rospy.get_param("base_frame", "base_link")
        self.timeout = rospy.get_param("timeout", 5)

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(self.timeout))
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self.broadcaster = tf2_ros.TransformBroadcaster()

        self.ground_truth_tfs = []
        self.tag_frames = []

        self.load_tag_tf_bag(bag_name)

    def load_tag_tf_bag(self, bag_name):
        """ Load tag ground truth messages from bag and publish as
        static tf """
        bag = rosbag.Bag(bag_name)
        for topic, msg, t in bag.read_messages(topics=['/tag_ground_truth']):
            self.ground_truth_tfs.append(msg)
        rospy.loginfo("Ground truth messages loaded!")
        bag.close()

    def is_tag_frame(self, frame):
        """ Returns True if input frame is a virtual tag frame (starts with
         virtual_tag_), False otherwise

        Args:
            frame (str): Frame name

        Returns:
            is_tag (bool): Whether or not the input frame is a tag frame """

        if frame[0:12] == "virtual_tag_":
            return True
        else:
            return False

    def list_tag_frames(self):# TODO: move this to helper functions ==========================================

        """ Filters through tf frames and returns a list only containing
        frames that represent AprilTags """
        frame_list = self.tf_buffer._getFrameStrings()
        return filter(self.is_tag_frame, frame_list)

    def update_tag_tf(self):
        """ Re-publish ground truth tf """
        # TODO: make this use static publisher
        for frame in self.ground_truth_tfs:
            self.broadcaster.sendTransform(frame)

    def publishBasePoints(self):
        for tag in self.tag_frames:
            msg = PointLabeled()
            tag_tf = self.tf_buffer.lookup_transform(self.base_frame,
                tag, rospy.Time(0))

            print(self.tag_frames)
            # TODO: finish this


    def run(self):
        while not rospy.is_shutdown():
            self.update_tag_tf()

            if self.list_tag_frames() != None:
                self.tag_frames = self.list_tag_frames()

            self.publishBasePoints()

            self.update_rate.sleep()

if __name__ == '__main__':
    vs = VirtualSensors()
    vs.run()
