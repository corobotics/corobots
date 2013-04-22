#!/usr/bin/env python

import roslib; roslib.load_manifest("corobot_manager")
import rospy

from corobot_common.msg import UIMessage, UIConfirm
from corobot_manager.ui import CorobotUIMessage

class CorobotUI():
       
    def start(self):
        self.init_ros_node()

    def show_msg(self, ui_message):
        rospy.logerr("Message received")
        if ui_message.req_confirm:
            win = CorobotUIMessage(ui_message.msg, ui_message.timeout)
        else:
            win = CorobotUIMessage(ui_message.msg, ui_message.timeout, True)

        rospy.logerr("Entering mainloop()")
        win.mainloop()
        if ui_message.req_confirm:
            confirm = win.wasConfirmed()
            self.confirm_pub(UIConfirm(ui_message.id, confirm))


    def init_ros_node(self):
        rospy.init_node("corobot_ui")

        self.confirm_pub = rospy.Publisher("confirm_msg", UIConfirm)
        rospy.Subscriber("show_msg", UIMessage, self.show_msg)

        rospy.spin()

def main():
    CorobotUI().start()

if __name__ == "__main__":
    main()
