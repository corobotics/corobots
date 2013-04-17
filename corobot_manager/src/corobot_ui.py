#!/usr/bin/env python

import roslib; roslib.load_manifest("corobot_manager")
import rospy

from corobot_common.msg import CorobotUIConfirm
from corobot_manager.ui import CorobotUIMessage

class CorobotUI():
       
    def start(self):
        self.init_ros_node()

    def show_msg(ui_message):
        if ui_message.confirm:
            win = CorobotUIMessage(ui_message.text, ui_message.timeout, 
        else:
            win = CorobotUIMessage(ui_message.text, ui_message.timeout, self.confirm_pub

    def init_ros_node():
        rospy.init_node("corobot_ui")

        self.confirm_pub = rospy.Publisher("confirm_msg", CorobotUIConfirm)
        rospy.Subscriber("show_msg", self.show_msg)

def main():
    CorobotUI().start()

if __name__ == "__main__":
    main()
