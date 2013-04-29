#!/usr/bin/env python

import roslib; roslib.load_manifest("corobot_manager")
import rospy
import threading

from corobot_common.msg import UIMessage, UIConfirm
from corobot_manager.ui import CorobotUIMessage

class CorobotUI():
    """ROS Node for on-screen message popups"""
       
    def start(self):
        self.init_ros_node()

    def show_msg_callback(self, ui_message):
        """Callback for show_msg subscriber"""
        win = CorobotUIMessage(ui_message.msg, ui_message.timeout,
                                ui_message.req_confirm)

        win.mainloop()
        #When ui exits, if we needed confirmation then poll confirm flag.
        if ui_message.req_confirm:
            confirm = win.was_confirmed()
            self.confirm_pub.publish(UIConfirm(ui_message.id, confirm))

    def init_ros_node(self):
        """Initialize all ROS node/sub/pub/srv stuff."""
        rospy.init_node("corobot_ui")

        self.confirm_pub = rospy.Publisher("confirm_msg", UIConfirm)
        rospy.Subscriber("show_msg", UIMessage, self.show_msg_callback)

        rospy.spin()

def main():
    CorobotUI().start()

if __name__ == "__main__":
    main()
