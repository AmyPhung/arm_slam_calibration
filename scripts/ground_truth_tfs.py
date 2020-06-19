#!/usr/bin/env python
# ROS Imports
import roslib
import rospy
import tf2_ros
import rosbag

# ROS Messages
from std_msgs.msg import Bool
from geometry_msgs.msg import TransformStamped

class GroundTruthTfs():
    """ Republishes tf from camera to each of the tags to connect tags to
    base frame.

    ROS Params:
        timeout: number of seconds to wait for tf
    """
    def __init__(self):
        rospy.init_node("GroundTruthTfs")
        self.update_rate = rospy.Rate(10)

        self.timeout = rospy.get_param("timeout", 5)
        self.tag_frames = rospy.get_param("tag_frames",
            ['tag_584', 'tag_582', 'tag_580',
             'tag_579', 'tag_578', 'tag_577',
             'tag_9', 'tag_5', 'tag_7'])
        self.save_filename = rospy.get_param("save_filename",
            '/home/amy/whoi_ws/src/joint_calibration/bags/tag_ground_truth.bag')

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(self.timeout))
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self.broadcaster = tf2_ros.TransformBroadcaster()

        self.ground_truth_tfs = []

        self.load_tag_tf_bag(self.save_filename)

    def load_tag_tf_bag(self, bag_name):
        """ Load tag ground truth messages from bag """
        bag = rosbag.Bag(bag_name)
        for topic, msg, t in bag.read_messages(topics=['/tag_ground_truth']):
            self.ground_truth_tfs.append(msg)
        rospy.loginfo(self.ground_truth_tfs)
        rospy.loginfo("Ground truth messages loaded!")
        bag.close()

    def broadcast_tag_tfs(self):
        """ Broadcasts ground truth tag locations with respect to base
        frame """
        # TODO: make this use static publisher
        for frame in self.ground_truth_tfs:
            frame.header.stamp = rospy.get_rostime()
            self.broadcaster.sendTransform(frame)

    def run(self):
        while not rospy.is_shutdown():
            self.broadcast_tag_tfs()
            self.update_rate.sleep()

if __name__ == '__main__':
    gtt = GroundTruthTfs()
    gtt.run()
