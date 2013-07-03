#!/usr/bin/env python

import roslib; roslib.load_manifest("corobot_manager")
import rospy

from corobot_common.msg import Pose
from corobot_manager.ui import CorobotMonitorUI

class CorobotMonitor():
    """ROS Node for data monitoring and interaction"""
       
    def __init__(self):
        self.win = CorobotMonitorUI()
        self.init_ros_node()
        self.win.mainloop()

    def pose_callback(self, pose_message):
        self.win.setPose(pose_message.x, pose_message.y, pose_message.theta)
        #self.win.after(100,rospy.spin_once())

    def init_ros_node(self):
        """Initialize all ROS node/sub/pub/srv stuff."""
        rospy.init_node("corobot_monitor")

        rospy.Subscriber("pose", Pose, self.pose_callback)


def main():
    cm = CorobotMonitor()

if __name__ == "__main__":
    main()
