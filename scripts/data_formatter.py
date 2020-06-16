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

        # TODO: Remove hardcode
        self.tags = ['virtual_tag_584', 'virtual_tag_582', 'virtual_tag_580',
                     'virtual_tag_579', 'virtual_tag_578', 'virtual_tag_577',
                     'virtual_tag_9', 'virtual_tag_5', 'virtual_tag_7']

        self.blank = [None] * len(self.tags)

        self.base_msgs = OrderedDict(zip(self.tags, self.blank))
        self.fisheye_msgs = OrderedDict(zip(self.tags, self.blank))
        self.state_msg = None

        # TODO: Make this a param
        self.bag = rosbag.Bag('/home/amy/whoi_ws/src/joint_calibration/bags/calibration_data.bag', 'w') # TODO: make this a param

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

        # TODO: Put this somewhere else - just for quick testing
        data_msg.joint_states.position = list(map(lambda x : x - 1, data_msg.joint_states.position))

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
