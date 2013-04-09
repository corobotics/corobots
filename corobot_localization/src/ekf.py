from math import cos, sin

from numpy.matlib import eye, matrix, zeros

from corobot_common.msg import Pose
from utils import column_vector

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
        return tuple(self.state.T.tolist()[0])

    def get_pose(self):
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

    def update_vel(self, twist):
        """Convenience function to do a velocity update."""
        v = twist.twist.linear.x
        w = twist.twist.angular.z
        y = column_vector(v * cos(self.theta), v * sin(self.theta), w)
        W = reduce_covariance(twist.covariance)
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

    def predict(self, vc, wc, V):
        x, y, t, vx, vy, w = self.state_tuple
        V = matrix(V).reshape(3, 3)
        # state prediction
        sp = EKF.column_vector(x + v * cos(t), y + self.dt * v * sin(t), t + self.dt * w)
        F = matrix([
            [1, 0, -self.dt * v * sin(t)],
            [0, 1, self.dt * v * cos(t)],
            [0, 0, 1]])
        # covariance prediction
        PP = F * P * F.T + V
        return sp, PP
