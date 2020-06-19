#!/usr/bin/env python
# ROS Imports
import roslib
import rospy
import tf2_ros
import rosbag

# ROS Messages
from std_msgs.msg import Bool
from geometry_msgs.msg import TransformStamped

class CaptureGroundTruth():
    """ To be used with ground_truth_gui.py to select point to use
    as ground truth for tag locations with respect to the robot base

    ROS Params:
        base_frame: base_link or equivalent
        camera_frame1: camera frame attached to base frame
        camera_frame2: camera frame attached to tags
        timeout: number of seconds to wait for tf
    """
    def __init__(self):
        rospy.init_node("GroundTruthCreator")
        self.update_rate = rospy.Rate(10)

        self.base_frame = rospy.get_param("base_frame", "base_link")
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
        self.capture_sub  = rospy.Subscriber("/capture_ground_truth",
            Bool, self.capture_callback)

        self.complete = False

        # TODO: Make this a param
        self.bag = rosbag.Bag(self.save_filename, 'w')

    def capture_callback(self, msg):
        """ Callback function from gui output topic """
        if msg.data == True:
            self.record_tag_tfs()
            self.bag.close()
            self.complete = True

    def record_tag_tfs(self):
        """ Record transforms from base frame to each of the tags """
        rospy.loginfo("Recording tag transforms to bag")

        for tag in self.tag_frames:
            sample_tag = "virtual_" + tag

            sample_tf = self.tf_buffer.lookup_transform(self.base_frame,
                sample_tag, rospy.Time(0))

            sample_tf.child_frame_id = "base_" + tag
            self.bag.write('/tag_ground_truth', sample_tf)

    def run(self):
        while not rospy.is_shutdown():
            self.update_rate.sleep()

            if self.complete == True:
                rospy.loginfo("Ground truth tag frames captured! Exiting...")
                break


if __name__ == '__main__':
    cgt = CaptureGroundTruth()
    cgt.run()
