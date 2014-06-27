#!/usr/bin/env python
import roslib; roslib.load_manifest("corobot_spin")
import rospy
import sys
from corobot_common.msg import Pose
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import atan2,pi,sqrt

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
        self.pose = Pose()
        self.traveled = 0
        self.observed = 0

    def pose_callback(self, pose):
        self.lastpose = self.pose
        self.pose = odom_to_pose(pose)
        self.observed = self.pose.theta
        self.traveled += sqrt(pow(self.lastpose.x-self.pose.x,2)+pow(self.lastpose.y-self.pose.y,2))

    def pause_callback(self, event):
        rospy.Timer(rospy.Duration(secs = self.timePerTurn),self.timer_callback,True)
        self.done = False
        go = Twist()
        go.linear.x = 0 # fwd velocity during turning - make a parameter?
        go.angular.z = self.angularV
        rospy.loginfo("Theta: %g, Dist traveled: %g",self.observed,self.traveled)
        while(not self.done):
              pub.publish(go)

    def timer_callback(self, event):
        self.nTurns-=1
        self.done = True
        go = Twist()
        go.linear.x = 0.2 # fwd velocity between turns - make a parameter?
        go.angular.z = self.angularV
        if(self.nTurns  >= 1):
            rospy.Timer(rospy.Duration(secs = 2),self.pause_callback,True)
        else:
            go.linear.x = 0
            go.angular.z = 0
        rospy.loginfo("Theta: %g, Dist traveled: %g",self.observed,self.traveled)
        while(self.done):
            pub.publish(go)
        

    def start(self):
        self.done = False
        rospy.init_node("rotate")
        rospy.Subscriber("odom", Odometry, self.pose_callback)
        self.expected = self.nTurns * self.timePerTurn * self.angularV
        self.pause_callback(None)
        rospy.spin()


def main():
    spin(rospy.get_param("/rotate/nTurns"), rospy.get_param("/rotate/timePerTurn"),rospy.get_param("/rotate/angularV")).start()

if __name__ == "__main__":
    main()
