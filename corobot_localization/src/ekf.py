from numpy.matlib import absolute, array, concatenate, diag, eye, matrix, zeros

from corobot_common.msg import Pose
from utils import column_vector, coord_transform, get_offset, reduce_covariance

class EKF(object):

    """Implements an Extended Kalman Filter.

    Follows the EKF described in Principles of Robotic Motion.

    """

    def __init__(self):
        # x(k|k); the system state column vector.
        self.state = column_vector(0.0, 0.0, 0.0)
        # P(k|k); the system covariance matrix.
        self.covariance = matrix([
            [1000.0,    0.0, 0.0],
            [   0.0, 1000.0, 0.0],
            [   0.0,    0.0, 6.0]])
        # Need to store old odom state for delta updates.
        self.odom_state = None

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
        """Returns a tuple of the current state: (x, y, theta)"""
        return tuple(self.state.T.tolist()[0])

    def get_pose(self):
        """Converts the current EKF state into a Pose object."""
        pose = Pose()
        pose.header.frame_id = "world"
        pose.x = self.x
        pose.y = self.y
        pose.theta = self.theta
        pose.cov = tuple(self.covariance.flat)
        return pose

    def get_odom_delta(self, pose):
        # Convert into matrix form.
        y_odom = column_vector(pose.x, pose.y, pose.theta)
        odom_delta = None
        if self.odom_state is not None:
            # Get the odom frame origin in the map frame.
            odom_origin = get_offset(self.state, self.odom_state)
            # Transform the sensor state into the map frame.
            y = coord_transform(y_odom, odom_origin)
            # Calculate the change odom state, in map coords (delta y).
            odom_delta = y - coord_transform(self.odom_state, odom_origin)
        # Update the stored odom state.
        self.odom_state = y_odom
        return odom_delta

    def update_pos(self, pose):
        """Convenience function to do a position update."""
        y = column_vector(pose.x, pose.y, pose.theta)
        W = matrix(pose.cov).reshape(3, 3)
        self.update(y, W)

    def update(self, y, W):
        """Perform an EKF update with the given sensor data.

        y -- The sensor data (3x1 matrix).
        W -- The sensor noise/covariance (3x3 matrix).

        """
        H = eye(3)
        x, P = self.state, self.covariance
        R = P * H.T * (H * P * H.T + W).I
        self.state = x - R * (y - H * x)
        self.covariance = P - R * H * P

    def predict(self, odom_pose):
        delta = self.get_odom_delta(odom_pose)
        if delta is None:
            return
        # state prediction
        self.state = self.state + delta
        # use 10% of the delta values as covariance.
        V = matrix(diag(absolute(array(delta).T[0]))) * 0.1
        # covariance prediction
        self.covariance = self.covariance + V
