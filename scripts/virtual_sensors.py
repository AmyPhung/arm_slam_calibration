#!/usr/bin/env python

import roslib
import rosbag
import rospy
import math
import tf2_ros
from joint_calibration.msg import PointLabeled


class VirtualSensors():
    """ Node to create two virtual sensors - publishes labeled points
    representing tag positions in base_link frame as if they were measured
    by the fisheye camera or from the base_link itself (as a "ground truth")
    """
    def __init__(self):
        rospy.init_node("GroundTruthCreator")
        self.update_rate = rospy.Rate(10)

        # TODO: make bag name an ros param
        bag_name = "/home/amy/whoi_ws/src/joint_calibration/bags/tag_ground_truth.bag" # TODO: ROS param this
        self.base_frame = rospy.get_param("base_frame", "base_link")
        self.camera_frame = rospy.get_param("camera_frame", "fisheye")
        self.timeout = rospy.get_param("timeout", 500) # TODO: pick more sensible timeout

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(self.timeout))
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self.broadcaster = tf2_ros.TransformBroadcaster()

        self.sensor_base_pub = rospy.Publisher("/virtual_sensor_base",
            PointLabeled, queue_size=10)
        self.sensor_fisheye_pub = rospy.Publisher("/virtual_sensor_fisheye",
            PointLabeled, queue_size=10)

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

    def list_tag_frames(self):
        """ Filters through tf frames and returns a list only containing
        frames that represent AprilTags """
        frame_list = self.tf_buffer._getFrameStrings()
        return filter(self.is_tag_frame, frame_list)

    def update_tag_tf(self):
        """ Re-publish ground truth tf """
        # TODO: make this use static publisher
        for frame in self.ground_truth_tfs:
            frame.header.stamp = rospy.get_rostime()
            self.broadcaster.sendTransform(frame)

    def publishPoints(self, sensor):
        if sensor == "base":
            frame = self.base_frame
            publisher = self.sensor_base_pub
        elif sensor == "fisheye":
            frame = self.camera_frame
            publisher = self.sensor_fisheye_pub
        else:
            rospy.logerr("Invalid sensor name inputted in publishPoints")
            return

        for tag in self.tag_frames:
            # TODO: Get rid of warnings on startup

            # Get position of tag in sensor frame
            try:
                tag_tf = self.tf_buffer.lookup_transform(
                    frame, tag, rospy.Time(0))
            except (tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                rospy.logwarn("Frame %s couldn't be found", tag)
                continue

            msg = PointLabeled()
            msg.point_stamped.header = tag_tf.header
            msg.label = tag_tf.child_frame_id
            msg.point_stamped.point = tag_tf.transform.translation

            publisher.publish(msg)


    def run(self):
        while not rospy.is_shutdown():
            self.update_tag_tf()

            if self.list_tag_frames() != None:
                self.tag_frames = self.list_tag_frames()

            self.publishPoints(sensor="base")
            self.publishPoints(sensor="fisheye")

            self.update_rate.sleep()

if __name__ == '__main__':
    vs = VirtualSensors()
    vs.run()
