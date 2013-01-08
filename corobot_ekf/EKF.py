from math import cos, sin

from numpy import matrix

class EKF(object):

    """Implements an Extended Kalman Filter.

    Follows the EKF described in Principles of Robotic Motion.

    """

    def __init__(self, dt):

        # Time delta between updates in seconds.
        self.dt = dt

        # x(k|k); the system state column vector.
        x = matrix([[0.0], [0.0], [0.0]])

        # P(k|k); the system covariance matrix.
        P = matrix([
            [10000.0, 0.0, 0.0],
            [0.0, 10000.0, 0.0],
            [0.0, 0.0, 10000.0]])

        self.state = (x, P)
        self.velocity = None

        # Collects updated sensor data for the next update call.
        self.new_data = {}

        # Tracks the previous state of sensors for those that accumulate error over time.
        self.old_data = {
            # None indicates there is no previous data for this sensor.
            "odom": None
        }

        self.frame_offset = {
            "base": None
        }

    @staticmethod
    def column_vector(*args):
        """Utility function to construct a column vector (single-column matrix)."""
        return matrix([e] for e in args)

    @staticmethod
    def coord_transform(state, offset):
        """Perform a coordinate transform on a state vector.

        state: a column vector of x, y, theta
        offset: a column vector of dx, dy, dtheta

        """
        dt = offset[0][2]
        rotation = matrix([
            [cos(dt), -sin(dt), 0],
            [sin(dt),  cos(dt), 0],
            [      0,        0, 1]])
        return rotation * state + offset

    def data_received(self, sensor, pose):
        self.new_data[sensor] = pose

    def update(self):
        """Perform an EKF state update."""
        # store the old state
        old_x, old_P = self.state
        # Velocity is a special case; if we have it perform a prediction based
        # on the old state before continuing.
        if "velocity" in self.new_data:
            u, V = self.new_data.pop("velocity")
            self.state = self.predict(u, V)
        # For each sensor, modify the EKF state.
        # TODO: See if these need to be combined somehow instead of applying
        # them on top of the previous results.
        for sensor, pose in self.new_data.items():
            # state, covariance
            x, P = self.state
            # sensor output
            y = EKF.column_vector(pose.x, pose.y, pose.theta)
            # sensor covariance
            W = pose.cov
            # check if this sensor accumulates error over time.
            if sensor in self.old_data:
                old_pose = self.old_data[sensor]
                if old_pose is None:
                    continue
                # if it does, use the difference from its previous pose applied to the old state.
                old_y = EKF.column_vector(old_pose.x, old_pose.y, old_pose.theta)
                y = y - old_y + old_x
            # check if the sensor's coordinate frame is different from the global map's.
            if pose.header.frame_id in self.frame_offset:
                frame_offset = self.frame_offset[pose.header.frame_id]
                if frame_offset is None:
                    continue
                # if it's different, transform the coords.
                y = EKF.coord_transform(y, frame_offset)
            v = ?
            theta = y[0][2]
            R = P * (P + W).I
            x = x + R * (y - x)
            P = P - R * P
            self.state = x, P

    def predict(self, u, V):
        # state vector, covariance matrix
        s, P = self.state
        # x, y, theta
        x, y, t = s[0][0], s[1][0], s[2][0]
        # velocity, angular velocity (omega)
        v, w = u
        # state prediction
        sp = EKF.column_vector(x + v * cos(t), y + self.dt * v * sin(t), t + self.dt * w)
        F = matrix([
            [1, 0, -self.dt * v * sin(t)],
            [0, 1, selt.dt * v * cos(t)],
            [0, 0, 1]])
        # covariance prediction
        PP = F * P * F.transpose() + V
        return sp, PP

    def R():
        pass
