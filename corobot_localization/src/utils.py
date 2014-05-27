from math import atan2, cos, sin

from numpy.matlib import matrix

from corobot_common.msg import Pose

def reduce_covariance(cov):
    """Convert a flat 6x6 covariance matrix into a flat 3x3."""
    """ 6 variances are for linear x, y, z and rotate x, y, z """
    return (cov[0],  cov[1],  cov[5],
            cov[6],  cov[7],  cov[11],
            cov[30], cov[31], cov[35])

def odom_to_pose(odom):
    """Utility function to convert an Odometry message into a Pose message."""
    pose = Pose()
    pose.header = odom.header
    pose.x = odom.pose.pose.position.x
    pose.y = odom.pose.pose.position.y
    qz = odom.pose.pose.orientation.z
    qw = odom.pose.pose.orientation.w
    pose.theta = atan2(2 * qw * qz, 1 - 2 * qz * qz)
    pose.cov = reduce_covariance(odom.pose.covariance)
    return pose

def column_vector(*args):
    """Utility function to construct a column vector (single-column matrix)."""
    return matrix([[e] for e in args])

def rotation_matrix(theta):
    return matrix([
        [cos(theta), -sin(theta), 0],
        [sin(theta),  cos(theta), 0],
        [         0,           0, 1]])

def coord_transform(state, offset):
    """Perform a coordinate transform on a state vector.

    state: a column vector of x, y, theta
    offset: a column vector of dx, dy, dtheta

    """
    dt = offset.item(2, 0)
    rotation = rotation_matrix(dt)
    return rotation * state + offset

def get_offset(state_a, state_b):
    """Get the origin of reference frame B in A."""
    dt = state_a.item(2, 0) - state_b.item(2, 0)
    rotation = rotation_matrix(dt)
    return state_a - rotation * state_b
