#!/usr/bin/env python

import roslib; roslib.load_manifest("corobot_ekf")
import rospy

from geometry_msgs.msg import PoseWithCovariance
from nav_msgs.msg import Odometry

def odom_callback(odom):
    pub.publish(odom.pose)

def main():
    global pub;
    rospy.init_node("localization")
    pub = rospy.Publisher("pose", PoseWithCovariance)
    rospy.Subscriber("odom", Odometry, odom_callback)
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
