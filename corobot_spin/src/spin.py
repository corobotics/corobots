#!/usr/bin/env python
import roslib; roslib.load_manifest("corobot_spin")
import rospy
import sys
from corobot_common.msg import Pose
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import atan2

pub = rospy.Publisher("cmd_vel", Twist)

def odom_to_pose(odom):
    pose = Pose()
    pose.header = odom.header
    pose.x = odom.pose.pose.position.x
    pose.y = odom.pose.pose.position.y
    qz = odom.pose.pose.orientation.z
    qw = odom.pose.pose.orientation.w
    pose.theta = atan2(2*qw*qz, 1-2*qz*qz)
    return pose
    
class spin():
    def __init__(self, nTurns, timePerTurn, angularV):
	self.nTurns = nTurns
	self.timePerTurn = timePerTurn
	self.angularV = angularV

    def pose_callback(self, pose):
	self.observed = odom_to_pose(pose).theta

    def timer_callback(self, event):
	go = Twist()
	go.linear.x = 0
	go.angular.z = 0
	pub.publish(go)
	self.done = True
	rospy.loginfo(self.expected)
	rospy.loginfo(self.observed)
    def start(self):
	self.done = False
	rospy.init_node("rotate")
	rospy.Subscriber("odom", Odometry, self.pose_callback)
	self.expected = self.nTurns * self.timePerTurn * self.angularV
	rospy.Timer(rospy.Duration(secs = self.nTurns * self.timePerTurn),self.timer_callback)
	go = Twist()
	go.linear.x = 0
	go.angular.z = self.angularV
	while(not self.done):
  	    pub.publish(go)
	rospy.spin()


def main():
    argv = sys.argv
    for arg in argv:
    	rospy.loginfo(arg)
    spin(rospy.get_param("/rotate/nTurns"), rospy.get_param("/rotate/timePerTurn"),rospy.get_param("/rotate/angularV")).start()

if __name__ == "__main__":
    main()
