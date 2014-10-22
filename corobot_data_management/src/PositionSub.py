#!/usr/bin/env python
# PositionSub.py
# author - Jaydeep Untwal (jmu8722@cs.rit.edu)
import roslib; roslib.load_manifest('corobot_data_management')
import rospy
from corobot_common.msg import Pose
from DataManager import DataManager

dm = DataManager("http://vhost7.cs.rit.edu/Corobot/CorobotMain.php", "corobot", "corobot!@#")

class PositionSub:

    # Self variables
    count = 0
    frequency = 100

    def __init__(self):
        self.pose = None

    def ros_init(self):
        rospy.init_node('corobot_data_management')
        rospy.Subscriber('pose', Pose, self.pose_callback)

    def start(self):
        rospy.spin()

    def pose_callback(self, pose):

        self.count = self.count + 1

        if self.count == self.frequency:
            self.pose = pose
            position = str(pose.x) + "," + str(pose.y) + "," + str(pose.theta)
            dm.saveLocal(dm.hostName, "Server", "Position", position)
            self.count = 0

def main():
    ps = PositionSub()
    ps.ros_init()
    ps.start()

if __name__ == '__main__':
    main()

