#!/usr/bin/env python

import rospy
from robot_calibration_msgs.msg import CalibrationData
from robot_calibration_msgs.msg import Observation
from robot_calibration_msgs.msg import ExtendedCameraInfo
from joint_calibration.msg import PointLabeled
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState

# Python Imports
from collections import OrderedDict

class DataFormatter():
    """
    sync subs gui
    sync subs tagslam (fisheye)
    sync subs tagslam2 (stereo pair)



    dictionary: tag
    ** sequence needs to match between sensors **


    publisher: robot_description
    publisher: calibration_data






    ordered dictionary - sort alphabetically





    In caputre data:
    sync callback:
        if gui.true
            capture sample (fisheye.msg, tagslam2.msg)
        else
            save bag? - send shutdown signal?
            exit

    def capture_sample(self):

    formats and publishes calibration data to /calibration_data topic

    TODO: add break on exit
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

    def state_cb(self, msg):
        self.state_msg = msg

    def publish_current_values(self):
        data_msg = CalibrationData()
        data_msg.joint_states = self.state_msg

        fisheye_obs = Observation()
        fisheye_obs.sensor_name = "fisheye"
        fisheye_obs.features = self.fisheye_msgs.values()
        print(type(fisheye_obs.features))

        base_obs = Observation()
        base_obs.sensor_name = "base"
        base_obs.features = self.base_msgs.values()

        data_msg.observations = [fisheye_obs, base_obs]

        self.calibration_pub.publish(data_msg)


if __name__ == '__main__':
    df = DataFormatter()
    rospy.spin()
