#!/usr/bin/env python

import rospy
import rosbag
from joint_calibration.msg import CalibrationData
from joint_calibration.msg import PointGroup
from joint_calibration.msg import PointLabeled
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool
from std_msgs.msg import String
from sensor_msgs.msg import JointState

# Python Imports
from collections import OrderedDict

class DataFormatter():
    """ Formats data from fisheye and publishes calibration data to
    /calibration_data topic. Also writes robot_description and
    calibration data to calibration bag file
    """
    def __init__(self):
        rospy.init_node("DataFormatter")
        self.update_rate = rospy.Rate(10)

        # Publishers & Subscribers ---------------------------------------------
        # TODO: Make these syncronized
        self.sensor_fisheye_sub = rospy.Subscriber("/virtual_sensor_fisheye",
            PointLabeled, self.sensor_fisheye_cb)
        self.capture_sub  = rospy.Subscriber("/capture_calibration",
            Bool, self.capture_cb)
        self.state_sub  = rospy.Subscriber("/joint_states",
            JointState, self.state_cb)
        self.calibration_pub = rospy.Publisher("/calibration_data",
            CalibrationData, queue_size=10)

        # Load ROS Params ------------------------------------------------------
        self.tags = rospy.get_param("tag_frames",
            ['tag_584', 'tag_582', 'tag_580',
             'tag_579', 'tag_578', 'tag_577',
             'tag_9', 'tag_5', 'tag_7'])
        self.calib_filename = rospy.get_param("calib_filename",
            '/home/amy/whoi_ws/src/joint_calibration/bags/calibration_data.bag')

        # Initialize Class Variables -------------------------------------------
        # Create dict to save incoming fisheye messages by tag
        self.virtual_tag_frames = \
            [i + j for i, j in zip(['virtual_']*len(self.tags), self.tags)]
        self.blank = [None] * len(self.tags)
        self.fisheye_msgs = OrderedDict(zip(self.virtual_tag_frames, self.blank))

        # Where JointState messages will be saved
        self.state_msg = None

        # Initialize CalibrationData with empty PointGroup messages
        self.calibration_data = CalibrationData()
        self.calibration_data.labels = self.virtual_tag_frames
        self.calibration_data.point_groups = []
        for i in range(len(self.tags)):
            self.calibration_data.point_groups.append(PointGroup())

        # Open bag we'll write data to -----------------------------------------
        self.bag = rosbag.Bag(self.calib_filename, 'w')

        # Write the URDF -------------------------------------------------------
        description = String()
        description.data = rospy.get_param('robot_description')
        self.bag.write('robot_description', description)

        # Node will shutdown when this is True
        self.complete = False


    def sensor_fisheye_cb(self, msg):
        tag_name = msg.label
        self.fisheye_msgs[tag_name] = msg.point_stamped

    def capture_cb(self, msg):
        """ Callback function from gui output topic """
        if msg.data == True:
            self.publish_current_values()
        else:
            for group in self.calibration_data.point_groups:
                group.num_pts = len(group.observations)
            self.bag.write('calibration_data', self.calibration_data)
            self.complete = True

    def state_cb(self, msg):
        # TODO: Put this somewhere else - just for quick testing of offsets
        TICKS_PER_DEGREE = [46, 49, 52, 46, 46, 46, 46]
        TICKS_ZERO = [65322, 3792, 59777, 64437, 65324, 65252, 1082]
        PI = 3.14159265359

        joint_states_ticks = []
        for i in range(len(msg.position)):
            joint_angle = msg.position[i]
            ticks_per_degree = TICKS_PER_DEGREE[i]
            tick_zero = TICKS_ZERO[i]
            tick_value = (joint_angle * (180/PI) * ticks_per_degree) + tick_zero
            joint_states_ticks.append(int(tick_value))
        msg.position = joint_states_ticks

        self.state_msg = msg

    def publish_current_values(self):
        # Iterate through point groups
        for i in range(len(self.calibration_data.labels)):
            # Get matching point in current values
            landmark = self.calibration_data.labels[i]
            new_pt = self.fisheye_msgs[landmark] # This is a stamped point

            # Save current point to calibration data
            curr_group = self.calibration_data.point_groups[i]
            curr_group.observations.append(new_pt)
            curr_group.joint_states.append(self.state_msg)

        # Publish current points for visibility
        self.calibration_pub.publish(self.calibration_data)

    def run(self):
        while not rospy.is_shutdown():
            if self.complete == True:
                break
            self.update_rate.sleep()

        self.bag.close()


if __name__ == '__main__':
    df = DataFormatter()
    df.run()
