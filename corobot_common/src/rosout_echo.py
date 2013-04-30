#!/usr/bin/env python

"""Super simple script to echo the rosout topic to a console."""

import roslib; roslib.load_manifest('corobot_common')
import rospy
from rosgraph_msgs.msg import Log

def rosout_callback(log):
    print "%s: %s" % (log.name, log.msg)

def main():
    rospy.init_node('rosout_echoer')
    rospy.Subscriber('rosout', Log, rosout_callback)
    rospy.spin()

if __name__ == "__main__":
    main()
