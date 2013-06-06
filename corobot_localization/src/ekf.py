from math import pi

from numpy.matlib import matrix

from corobot_common.msg import Pose
from utils import column_vector, coord_transform, get_offset, reduce_covariance

tau = pi * 2.0

class EKF(object):

    """Implements an Extended Kalman Filter.

    Follows the EKF described in Principles of Robotic Motion.

    """

    def __init__(self):
        # x(k|k); the system state column vector.
        self.state = column_vector(0.0, 0.0, 0.0)
        # P(k|k); the system covariance matrix.
        self.covariance = matrix([
            [1000.0,    0.0,    0.0],
            [   0.0, 1000.0,    0.0],
            [   0.0,    0.0, 1000.0]])
        # Need to store old odom state for delta updates.
        self.odom_state = None

    def state_tuple(self, state=None):
        """Returns a tuple of the given state matrix: (x, y, theta)"""
        if state is None:
            state = self.state
        return tuple(state.T.tolist()[0])

    def get_pose(self):
        """Converts the current EKF state into a Pose object."""
        pose = Pose()
        pose.header.frame_id = "world"
        pose.x, pose.y, pose.theta = self.state_tuple()
        pose.cov = tuple(self.covariance.flat)
        return pose

    def get_odom_delta(self, pose):
        """Calculates the delta of the odometry position since the last time called.

        Returns None the first time, and a 3x1 state matrix thereafter.

        """
        # Convert the pose into matrix form.
        y_odom = column_vector(pose.x, pose.y, pose.theta)
        # None is the fallback in case we can't get an actual delta.
        odom_delta = None
        if self.odom_state is not None:
            # Transform the sensor state into the map frame.
            y = coord_transform(y_odom, self.odom_origin)
            # Calculate the change odom state, in map coords (delta y).
            odom_delta = y - coord_transform(self.odom_state, self.odom_origin)
        # Update the stored odom state.
        self.odom_state = y_odom
        # Get the odom frame origin in the map frame and store it for next time.
        self.odom_origin = get_offset(self.state, self.odom_state)
        return odom_delta

    def update_pos(self, pose):
        """Convenience function to do a position update."""
        # assuming that the ekf is never lost by more than pi (except on startup),
        # we can intelligently mod the incoming pose angle to be close to the current estimate
        # note that the ordered if statements mean that if we are super-lost, we
        # will randomly add some pi but not infinitely loop.
        # this if statement says that if we're lost in theta, don't worry about it.
        if self.covariance[2,2] < tau:
            while pose.theta - self.state[2] > pi:
                pose.theta -= tau
            while self.state[2] - pose.theta > pi:
                pose.theta += tau
        y = column_vector(pose.x, pose.y, pose.theta)
        W = matrix(pose.cov).reshape(3, 3)
        self.update(y, W)

    def update(self, y, W):
        """Perform an EKF update with the given sensor data.

        y -- The sensor data (3x1 matrix).
        W -- The sensor noise/covariance (3x3 matrix).

        """
        x, P = self.state, self.covariance
        R = P * (P + W).I
        self.state = x + R * (y - x)
        self.covariance = P - R * P

    def predict(self, odom_pose):
        """Perform an EKF prediction step using odometry information."""
        delta = self.get_odom_delta(odom_pose)
        if delta is None:
            # Can't do a delta update the first time.
            return
        dx, dy, dt = self.state_tuple(delta)
        # State prediction; nice and simple.
        self.state = self.state + delta
        # Use 50% of the delta x/y values as covariance, and 200% theta.
        V = matrix([
            [abs(dx) * 0.5, 0.0, 0.0],
            [0.0, abs(dy) * 0.5, 0.0],
            [0.0, 0.0, abs(dt) * 2.0]])
        # Covariance prediction; add our custom covariance assumptions.
        self.covariance = self.covariance + V
