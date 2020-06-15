#!/usr/bin/env python
import Tkinter as tk
from std_msgs.msg import Bool
import rospy


class GroundTruthGui():
    def __init__(self):
        rospy.init_node("GroundTruthGui")
        self.capture_pub  = rospy.Publisher("/capture_ground_truth", Bool, queue_size=1)
        self.update_rate = rospy.Rate(10)

        # GUI Setup
        self.root = tk.Tk()
        self.root.geometry("250x100")
        self.button_capture = tk.Button(self.root, width=23,
            text="Use current tf as ground truth", command = self.captureCB)
        self.button_capture.pack()

        self.exit = False

    def captureCB(self):
        msg = Bool()
        msg.data = True
        self.capture_pub.publish(msg)
        self.exit = True

    def run(self):
        while not rospy.is_shutdown():
            self.root.update()

            if self.exit == True:
                break

if __name__=="__main__":
    gtg = GroundTruthGui()
    gtg.run()
