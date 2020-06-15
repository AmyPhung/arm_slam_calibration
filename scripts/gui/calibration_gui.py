#!/usr/bin/env python
import Tkinter as tk
from std_msgs.msg import Bool
import rospy


class CalibrationGui():
    def __init__(self):
        rospy.init_node("CalibrationGui")
        self.capture_pub  = rospy.Publisher("/capture_calibration", Bool, queue_size=1)
        self.update_rate = rospy.Rate(10)

        # GUI Setup
        self.root = tk.Tk()
        self.root.geometry("300x100")
        self.button_capture = tk.Button(self.root, width=25,
            text="Capture Calibration Data Point", command = self.captureCB)
        self.button_exit = tk.Button(self.root, width=25,
            text="Exit", command = self.exitCB)
        self.button_capture.pack()
        self.button_exit.pack()

        self.exit = False

    def captureCB(self):
        msg = Bool()
        msg.data = True
        self.capture_pub.publish(msg)

    def exitCB(self):
        msg = Bool()
        msg.data = False
        self.capture_pub.publish(msg)
        self.exit = True

    def run(self):
        while not rospy.is_shutdown():
            self.root.update()
            
            if self.exit == True:
                break

if __name__=="__main__":
    cg = CalibrationGui()
    cg.run()
