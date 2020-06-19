#!/usr/bin/env python

import rospy
import rosbag
from robot_calibration_msgs.msg import CalibrationData
from robot_calibration_msgs.msg import Observation
from robot_calibration_msgs.msg import ExtendedCameraInfo
from joint_calibration.msg import PointLabeled
from std_msgs.msg import Bool
from std_msgs.msg import String
from sensor_msgs.msg import JointState

# Python Imports
from collections import OrderedDict

class DataFormatter():
    """ Formats data from two virtual sensor topics and publishes
    calibration data to /calibration_data topic. Also writes robot_description
    and calibration data to calibration bag file
    """
    def __init__(self):
        rospy.init_node("DataFormatter")
        self.update_rate = rospy.Rate(10)

        # TODO: Make these syncronized
        self.sensor_base_sub = rospy.Subscriber("/virtual_sensor_base",
            PointLabeled, self.sensor_base_cb)
        self.sensor_fisheye_sub = rospy.Subscriber("/virtual_sensor_fisheye",
            PointLabeled, self.sensor_fisheye_cb)
        self.capture_sub  = rospy.Subscriber("/capture_calibration",
            Bool, self.capture_cb)
        self.state_sub  = rospy.Subscriber("/joint_states",
            JointState, self.state_cb)

        self.calibration_pub = rospy.Publisher("/calibration_data",
            CalibrationData, queue_size=10)

        self.tags = rospy.get_param("tag_frames",
            ['tag_584', 'tag_582', 'tag_580',
             'tag_579', 'tag_578', 'tag_577',
             'tag_9', 'tag_5', 'tag_7'])
        self.calib_filename = rospy.get_param("calib_filename",
            '/home/amy/whoi_ws/src/joint_calibration/bags/calibration_data.bag')

        self.base_tag_frames = \
            [i + j for i, j in zip(['base_']*len(self.tags), self.tags)]
        self.virtual_tag_frames = \
            [i + j for i, j in zip(['virtual_']*len(self.tags), self.tags)]
        self.blank = [None] * len(self.tags)

        self.base_msgs = OrderedDict(zip(self.base_tag_frames, self.blank))
        self.fisheye_msgs = OrderedDict(zip(self.virtual_tag_frames, self.blank))
        self.state_msg = None

        self.bag = rosbag.Bag(self.calib_filename, 'w')

        # write the URDF
        description = String()
        description.data = rospy.get_param('robot_description')
        self.bag.write('robot_description', description)

        # Node will shutdown when this is True
        self.complete = False


    def sensor_base_cb(self, msg):
        tag_name = msg.label
        self.base_msgs[tag_name] = msg.point_stamped

    def sensor_fisheye_cb(self, msg):
        tag_name = msg.label
        self.fisheye_msgs[tag_name] = msg.point_stamped

    def capture_cb(self, msg):
        """ Callback function from gui output topic """
        if msg.data == True:
            self.publish_current_values()
        else:
            self.complete = True

    def state_cb(self, msg):
        self.state_msg = msg

    def publish_current_values(self):
        data_msg = CalibrationData()
        data_msg.joint_states = self.state_msg

        # TODO: Put this somewhere else - just for quick testing of offsets
        # data_msg.joint_states.position = list(map(lambda x : x - 1, data_msg.joint_states.position))

        fisheye_obs = Observation()
        fisheye_obs.sensor_name = "fisheye"
        fisheye_obs.features = self.fisheye_msgs.values()

        base_obs = Observation()
        base_obs.sensor_name = "base"
        base_obs.features = self.base_msgs.values()

        data_msg.observations = [fisheye_obs, base_obs]

        self.calibration_pub.publish(data_msg)
        self.bag.write('calibration_data', data_msg)

    def run(self):
        while not rospy.is_shutdown():
            if self.complete == True:
                break
            self.update_rate.sleep()

        self.bag.close()


if __name__ == '__main__':
    df = DataFormatter()
    df.run()
