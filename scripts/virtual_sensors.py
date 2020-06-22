#!/usr/bin/env python

import roslib
import rosbag
import rospy
import math
import tf2_ros
from joint_calibration.msg import PointLabeled


class VirtualSensors():
    """ Node to create two virtual sensors - looks up tf between base_link ->
    ground truth tags and fisheye -> virtual tags as labeled points """
    def __init__(self):
        rospy.init_node("VirtualSensors")
        self.update_rate = rospy.Rate(10)

        self.base_frame = rospy.get_param("base_frame", "base_link")
        self.camera_frame1 = rospy.get_param("camera_frame1", "fisheye")
        self.timeout = rospy.get_param("timeout", 500) # TODO: pick more sensible timeout
        tfs = rospy.get_param("tag_frames",
            ['tag_584', 'tag_582', 'tag_580',
             'tag_579', 'tag_578', 'tag_577',
             'tag_9', 'tag_5', 'tag_7'])

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(self.timeout))
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self.broadcaster = tf2_ros.TransformBroadcaster()

        self.sensor_base_pub = rospy.Publisher("/virtual_sensor_base",
            PointLabeled, queue_size=10)
        self.sensor_fisheye_pub = rospy.Publisher("/virtual_sensor_fisheye",
            PointLabeled, queue_size=10)

        # Pre-set ground truth frames
        self.base_tag_frames = \
            [i + j for i, j in zip(['base_']*len(tfs), tfs)]
        # From tagslam
        self.virtual_tag_frames = \
            [i + j for i, j in zip(['virtual_']*len(tfs), tfs)]

    def publishPoints(self, sensor):
        if sensor == "base":
            source_frame = self.base_frame
            frames_list = self.base_tag_frames
            frames_list2 = self.base_tag_frames
            publisher = self.sensor_base_pub
        if sensor == "fisheye":
            source_frame = self.camera_frame1
            frames_list = self.base_tag_frames
            frames_list2 = self.virtual_tag_frames
            publisher = self.sensor_fisheye_pub

        i = -1
        for tag in frames_list:
            i = i + 1
            tag2 = frames_list2[i]
            # Get position of tag in sensor frame
            try:
                tag_tf = self.tf_buffer.lookup_transform(
                    source_frame, tag, rospy.Time(0))
            except (tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                continue

            try:
                tag_tf2 = self.tf_buffer.lookup_transform(
                    source_frame, tag2, rospy.Time(0))
            except (tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                continue

            msg = PointLabeled()
            msg.point_stamped.header = tag_tf.header
            msg.label = tag_tf2.child_frame_id
            msg.point_stamped.point = tag_tf.transform.translation

            publisher.publish(msg)

    def run(self):
        while not rospy.is_shutdown():
            self.publishPoints(sensor="base")
            self.publishPoints(sensor="fisheye")
            self.update_rate.sleep()

if __name__ == '__main__':
    vs = VirtualSensors()
    vs.run()
