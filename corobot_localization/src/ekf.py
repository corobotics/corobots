from math import cos, sin

from numpy.matlib import concatenate, eye, matrix, zeros

from corobot_common.msg import Pose
from utils import column_vector, reduce_covariance

class EKF(object):

    """Implements an Extended Kalman Filter.

    Follows the EKF described in Principles of Robotic Motion.

    """

    def __init__(self, dt):
        # Time delta between updates in seconds.
        self.dt = float(dt)
        # x(k|k); the system state column vector.
        self.state = column_vector(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        # P(k|k); the system covariance matrix.
        self.covariance = matrix([
            [1000.0,    0.0,    0.0, 0.0, 0.0, 0.0],
            [   0.0, 1000.0,    0.0, 0.0, 0.0, 0.0],
            [   0.0,    0.0, 1000.0, 0.0, 0.0, 0.0],
            [   0.0,    0.0,    0.0, 1.0, 0.0, 0.0],
            [   0.0,    0.0,    0.0, 0.0, 1.0, 0.0],
            [   0.0,    0.0,    0.0, 0.0, 0.0, 1.0]])
        # Constants for the update H matrices.
        self.HPOS = concatenate(eye(3), zeros((3, 3)), axis=1)
        self.HVEL = concatenate(zeros((3, 3)), eye(3), axis=1)

    @property
    def x(self):
        return self.state.item(0, 0)

    @property
    def y(self):
        return self.state.item(1, 0)

    @property
    def theta(self):
        return self.state.item(2, 0)

    @property
    def state_tuple(self):
        """Returns a tuple of the current state: (x, y, theta, vx, vy, w)"""
        return tuple(self.state.T.tolist()[0])

    def get_pose(self):
        """Converts the current EKF state into a Pose object."""
        pose = Pose()
        pose.header.frame_id = "world"
        pose.x = self.x
        pose.y = self.y
        pose.theta = self.theta
        pose.cov = tuple(self.covariance[0:3,0:3].flat)
        return pose

    def update_pos(self, pose):
        """Convenience function to do a position update."""
        y = column_vector(pose.x, pose.y, pose.theta)
        W = matrix(pose.cov).reshape(3, 3)
        self.update(y, W, self.HPOS)

    def update_vel(self, twist_with_cov):
        """Convenience function to do a velocity update."""
        v = twist_with_cov.twist.linear.x
        w = twist_with_cov.twist.angular.z
        y = column_vector(v * cos(self.theta), v * sin(self.theta), w)
        W = reduce_covariance(twist_with_cov.covariance)
        self.update(y, W, self.HVEL)

    def update(self, y, W, H):
        """Perform an EKF update with the given sensor data.

        y -- The sensor data (3x1 matrix).
        W -- The sensor noise/covariance (3x3 matrix).
        H -- The mapping that describes which attributes are being updated.
             A 3x6 matrix with I on the left for pos and on the right for vel.

        """
        x, P = self.state, self.covariance
        R = P * H.T * (H * P * H.T + W).I
        self.state = x - R * (y - H * x)
        self.covariance = P - R * H * P

    def predict(self, twist):
        # command velocity and angular velocity
        vc, wc = twist.linear.x, twist.angular.z
        x, y, theta, vx, vy, w = self.state_tuple
        cost, sint = cos(theta), sin(theta)
        dt = self.dt
        V = matrix(V).reshape(3, 3)
        # state prediction
        self.state = column_vector(
            x + dt * vx,
            y + dt * vy,
            theta + dt * w,
            vc * cost,
            vc * sint,
            wc)
        # the partial derivatives of f for the state
        Fp = matrix([
            [1.0, 0.0, 0.0,  dt, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0,  dt, 0.0],
            [0.0, 0.0, 1.0, 0.0, 0.0,  dt],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])
        # the partial derivatives of f for the commands
        Fu = matrix([
            [ 0.0, 0.0],
            [ 0.0, 0.0],
            [ 0.0, 0.0],
            [cost, 0.0],
            [sint, 0.0],
            [ 0.0, 1.0]])
        # the input command covariances
        V = matrix([
            [0.05 * vc, 0.0],
            [0.0, 0.05 * wc]])
        # covariance prediction
        self.covariance = Fp * P * Fp.T + Fu * V * Fu.T
