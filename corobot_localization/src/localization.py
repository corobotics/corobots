#!/usr/bin/env python

import roslib; roslib.load_manifest("corobot_localization")
import rospy

from corobot_common.msg import Pose
from geometry_msgs.msg import Twist,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

from ekf import EKF
from utils import odom_to_pose

#from raj_test import *

# Expected frequency of odom updates, in Hz.
ODOM_FREQ = 10.0

def odom_callback(odom):
    # by definition, if there is laser and/or QR data, it was there
    # before this odometry came in, so process that first.
    if ekf.lastlaser is not None and ekf.uselaser:
        ekf.update_pos(ekf.lastlaser)
        ekf.lastlaser = None
        ekf.uselaser = False
        pose = ekf.get_pose()
        rospy.loginfo("After laser pose is (%6.3f, %6.3f, %6.3f) cov %s",pose.x, pose.y,pose.theta,pose.cov) 
    if ekf.lastqr is not None and ekf.useqr:
        ekf.update_pos(ekf.lastqr,True)
        ekf.lastqr = None
        ekf.useqr = False
        pose = ekf.get_pose()
        rospy.loginfo("After QR pose is (%6.3f, %6.3f, %6.3f) cov %s",pose.x, pose.y,pose.theta,pose.cov) 
    opose = odom_to_pose(odom)
    rospy.loginfo("Odom is (%6.3f, %6.3f, %6.3f)",opose.x, opose.y,opose.theta) 
    ekf.predict(opose)
    #print ekf.get_pose()
    pose = ekf.get_pose()
    rospy.loginfo("After odom pose is (%6.3f, %6.3f, %6.3f) cov %s",pose.x, pose.y,pose.theta,pose.cov)
    pose_pub.publish(ekf.get_pose())

def laser_callback(pose):
    # large negative values are impossible, used as a sentinel to the GUI, ignore here
    if pose.x < -999:
        return
    if ekf.uselaser:
        rospy.loginfo("Laser says (%6.3f, %6.3f, %6.3f) cov %s",pose.x, pose.y,pose.theta,pose.cov)
        ekf.lastlaser = pose
        #pose = ekf.get_pose()
        #rospy.loginfo("After laser pose is (%6.3f, %6.3f, %6.3f) cov %s",pose.x, pose.y,pose.theta,pose.cov) 
        #ekf.uselaser = False

def qrcode_callback(pose):
    if ekf.useqr:
        rospy.loginfo("QR says (%6.3f, %6.3f, %6.3f) cov %s",pose.x, pose.y,pose.theta,pose.cov)
        ekf.lastqr = pose
        #pose = ekf.get_pose()
        #rospy.loginfo("After QR pose is (%6.3f, %6.3f, %6.3f) cov %s",pose.x, pose.y,pose.theta,pose.cov)
        #ekf.useqr = False

def odom_comb_callback(odom):
    ocpose = odom_to_pose(odom)
    rospy.loginfo("Odom combined is (%6.3f, %6.3f, %6.3f)",ocpose.x, ocpose.y,ocpose.theta) 

def main():
    global ekf, pose_pub;
    rospy.init_node("localization")
    ekf = EKF()
    pose_pub = rospy.Publisher("pose", Pose)
    #rospy.Subscriber("odom_combined", PoseWithCovarianceStamped, odom_comb_callback)
    rospy.Subscriber("odom", Odometry, odom_callback)
    # odom_combined comes from robot_pose_ekf, different message type from
    # regular odom, but seems to have the same basic contents, so we can use
    # the same callback (yay dynamic typing!)
    #rospy.Subscriber("odom_combined", PoseWithCovarianceStamped, odom_callback)
    #rospy.Subscriber("/corobot_pose_ekf/odom_semicombined", PoseWithCovarianceStamped, odom_callback)
    rospy.Subscriber("laser_pose", Pose, laser_callback)
    rospy.Subscriber("qrcode_pose", Pose, qrcode_callback)
    rospy.spin()

def test():
    from math import pi
    import numpy
    # pose in frame A
    a = EKF.column_vector(1, 2, pi / 2)
    # pose in frame B
    b = EKF.column_vector(1, 1, 0)
    # actual origin of B in A
    B = EKF.column_vector(2, 1, pi / 2)
    # calculated origin of B in A
    BB = EKF.get_offset(a, b)
    # use it to convert b into a
    aa = EKF.coord_transform(b, BB)
    # assert that things match
    assert numpy.equal(B, BB).all()
    assert numpy.equal(a, aa).all()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
