#!/usr/bin/env python
import roslib; roslib.load_manifest("corobot_localization")
import rospy

from corobot_common.msg import Pose
from geometry_msgs.msg import Twist,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

from ekf_raj import EKF
from raj_test import *
from utils import odom_to_pose

def odom_callback(odom):
    
	#print "ODOMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM"
	# print odom.pose
	# odom = linear twist, angular twist and convariance 
	# between xl, yl, zl, xa, ya, za = 36 values
	
	# converting odom topic to pose value
	ekf.predict(odom_to_pose(odom))
	#print "O"
	#print "O"
	#print "O"
	#print "O"

#   pose_pub.publish(ekf.get_pose())

def laser_callback(scan):
	#print "LASERRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR"
	ekf.update(scan)
	#pose_pub.publish(ekf.get_pose())
	#print "L"
	#print "L"
	#print "L"
	#print "L"

def qrcode_callback(pose):
	#print "QRCODEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE"
	ekf.update_pos(pose)
	#print "Q"
	#print "Q"
	#print "Q"
	#print "Q"

def main():
	global ekf, pose_pub;
	rospy.init_node("localization")
	ekf = EKF()
	pose_pub = rospy.Publisher("pose", Pose)
	rospy.Subscriber("odom", Odometry, odom_callback)
	rospy.Subscriber("scan", LaserScan, laser_callback)
    # odom_combined comes from robot_pose_ekf, different message type from
    # regular odom, but seems to have the same basic contents, so we can use
    # the same callback (yay dynamic typing!)
    #rospy.Subscriber("odom_combined", PoseWithCovarianceStamped, odom_callback)
    #rospy.Subscriber("/corobot_pose_ekf/odom_semicombined", PoseWithCovarianceStamped, odom_callback)
    #rospy.Subscriber("laser_pose", Pose, laser_callback)
    	rospy.Subscriber("qrcode_pose", Pose, qrcode_callback)
	rospy.spin()

if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass
