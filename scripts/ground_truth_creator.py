#!/usr/bin/env python
# ROS Imports
import roslib
import rospy
import tf2_ros

# ROS Messages
from std_msgs.msg import Bool
from geometry_msgs.msg import TransformStamped

class GroundTruthCreator():
    """ Republishes tf from camera to each of the tags to connect tags to
    base frame. To be used with ground_truth_gui.py to select point to use
    as ground truth for tag locations with respect to the robot base

    ROS Params:
        base_frame: base_link or equivalent
        camera_frame1: camera frame attached to base frame
        camera_frame2: camera frame attached to tags
        timeout: number of seconds to wait for tf
    """
    def __init__(self):
        rospy.init_node("GroundTruthCreator")
        self.update_rate = rospy.Rate(1)

        self.base_frame = rospy.get_param("base_frame", "base_link")
        self.camera_frame1 = rospy.get_param("camera_frame1", "fisheye")
        self.camera_frame2 = rospy.get_param("camera_frame2", "cam0")
        self.timeout = rospy.get_param("timeout", 5)

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(self.timeout))
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.capture_sub  = rospy.Subscriber("/capture_ground_truth",
            Bool, self.capture_callback)
        self.tag_ground_truth_pub = rospy.Publisher("/tag_ground_truth",
            TransformStamped, queue_size=10)

        self.tag_frames = []
        self.complete = False

    def capture_callback(self, msg):
        """ Callback function from gui output topic """
        if msg.data == True:
            self.publish_tag_tfs()
            self.complete = True

    def is_tag_frame(self, frame):
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

    def list_tag_frames(self):
        """ Filters through tf frames and returns a list only containing
        frames that represent AprilTags """
        frame_list = self.tf_buffer._getFrameStrings()
        return filter(self.is_tag_frame, frame_list)


    def publish_tag_tfs(self):
        """ Publish transforms from base frame to each of the tags """
        rospy.loginfo("Publishing tag transforms")

        for tag in self.tag_frames:
            virtual_tag = "virtual_" + tag

            virtual_tf = self.tf_buffer.lookup_transform(self.base_frame,
                virtual_tag, rospy.Time(0))
            self.tag_ground_truth_pub.publish(virtual_tf)

    def broadcast_tag_tfs(self):
        """ Broadcasts tag locations with respect to base frame """
        for tag in self.tag_frames:
            try:
                tag_tf = self.tf_buffer.lookup_transform(tag,
                    self.camera_frame2, rospy.Time(0))
            except (tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                rospy.logwarn("Frame %s couldn't be found", tag)
                continue

            new_tag_frame = "virtual_" + tag

            # Change tag tf names & re-broadcast
            tag_tf.header.frame_id = self.camera_frame1
            tag_tf.child_frame_id = new_tag_frame
            self.broadcaster.sendTransform(tag_tf)


    def run(self):
        while not rospy.is_shutdown():
            if self.list_tag_frames() != None:
                self.tag_frames = self.list_tag_frames()

            rospy.loginfo(self.tag_frames)
            self.broadcast_tag_tfs()
            self.update_rate.sleep()

            if self.complete == True:
                break




if __name__ == '__main__':
    gtc = GroundTruthCreator()
    gtc.run()
