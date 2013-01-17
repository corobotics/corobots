#!/usr/bin/env python

from math import atan2

import roslib; roslib.load_manifest("corobot_ekf")
import rospy

from corobot_msgs.msg import Pose
from nav_msgs.msg import Odometry

def convert_to_pose(odom):
    pose = Pose()
    pose.header = odom.header
    pose.x = odom.pose.position.x
    pose.y = odom.pose.position.y
    qz = odom.pose.orientation.z
    qw = odom.pose.orientation.w
    pose.theta = atan2(0, w * w - z * z)
    cov = odom.pose.covariance
    pose.cov = (
        cov[0],  cov[1],  cov[5],
        cov[6],  cov[7],  cov[11],
        cov[30], cov[31], cov[35])
    return pose

def odom_callback(odom):
    pub.publish(convert_to_pose(odom))

def main():
    global pub;
    rospy.init_node("localization")
    pub = rospy.Publisher("pose", Pose)
    rospy.Subscriber("odom", Odometry, odom_callback)
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
