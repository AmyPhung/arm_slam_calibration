#!/usr/bin/env python
import Tkinter as tk
from std_msgs.msg import Bool
import rospy


class GuiWindow():
    def __init__(self):
        rospy.init_node("GuiWindow")
        self.capture_pub  = rospy.Publisher("/capture", Bool, queue_size=1)
        self.update_rate = rospy.Rate(10)

        # GUI Setup
        self.root = tk.Tk()
        self.root.geometry("150x100")
        self.button_capture = tk.Button(self.root, width=8, text="Capture",
            command = self.captureCB)
        self.button_exit = tk.Button(self.root, width=8, text="Exit",
            command = self.exitCB)
        self.button_capture.pack()
        self.button_exit.pack()

    def captureCB(self):
        msg = Bool()
        msg.data = True
        self.capture_pub.publish(msg)

    def exitCB(self):
        msg = Bool()
        msg.data = False
        self.capture_pub.publish(msg)

    def run(self):
        while not rospy.is_shutdown():
            self.root.update()

if __name__=="__main__":
    gw = GuiWindow()
    gw.run()
