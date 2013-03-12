#!/usr/bin/env python

from math import atan2

import roslib; roslib.load_manifest("corobot_ekf")
import rospy

from corobot_msgs.msg import Pose
from nav_msgs.msg import Odometry

from EKF import EKF

# Expected frequency of odom updates, in Hz.
ODOM_FREQ = 10.0

def reduce_covariance(cov):
    """Convert a flat 6x6 covariance matrix into a flat 3x3."""
    return (cov[0],  cov[1],  cov[5],
            cov[6],  cov[7],  cov[11],
            cov[30], cov[31], cov[35])

def odom_to_pose(odom):
    pose = Pose()
    pose.header = odom.header
    pose.x = odom.pose.pose.position.x
    pose.y = odom.pose.pose.position.y
    qz = odom.pose.pose.orientation.z
    qw = odom.pose.pose.orientation.w
    pose.theta = atan2(2 * qw * qz, 1 - 2 * qz * qz)
    pose.cov = reduce_covariance(odom.pose.covariance)
    return pose

def odom_to_velocity(odom):
    """Pulls the velocity info out of an Odom object."""
    v = odom.twist.twist.linear.x
    w = odom.twist.twist.angular.z
    V = reduce_covariance(odom.twist.covariance)
    return ((v, w), V)

def odom_callback(odom):
    ekf.data_received("odom", odom_to_pose(odom))
    ekf.update()
    ekf.data_received("velocity", odom_to_velocity(odom))
    pub.publish(ekf.get_pose())

def laser_callback(pose):
    ekf.data_received("laser", pose)

def barcode_callback(pose):
    ekf.data_received("barcode", pose)

def main():
    global ekf, pub;
    rospy.init_node("localization")
    ekf = EKF(1.0 / ODOM_FREQ)
    pub = rospy.Publisher("pose", Pose)
    rospy.Subscriber("odom", Odometry, odom_callback)
    rospy.Subscriber("laser_pose", Pose, laser_callback)
    rospy.Subscriber("barcode_pose", Pose, barcode_callback)
    rospy.spin()

def test():
    from math import pi
    import numpy
    # pose in frame A
    a = EKF.column_vector(1, 2, pi / 2)
    # pose in frame B
    b = EKF.column_vector(1, 1, 0)
    # get the origin of B in A
    o = EKF.get_offset(a, b)
    # use it to convert b into a
    t = EKF.coord_transform(b, o)
    assert numpy.equal(t, a).all()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
