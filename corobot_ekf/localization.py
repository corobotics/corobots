#!/usr/bin/env python

import roslib; roslib.load_manifest("corobot_localization")
import rospy

from geometry_msgs.msg import PoseWithCovarianceStamped

def odom_callback(odom):
    pub.publish(odom.pose)

def main():
    global pub;
    rospy.init_node("localization")
    pub = rospy.Publisher("pose", 1000)
    rospy.Subscriber("odom", Odometry, odom_callback)
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
