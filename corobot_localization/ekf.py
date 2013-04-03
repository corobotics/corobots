from math import cos, sin

from numpy import matrix

from corobot_msgs.msg import Pose

class EKF(object):

    """Implements an Extended Kalman Filter.

    Follows the EKF described in Principles of Robotic Motion.

    """

    def __init__(self, dt):

        # Time delta between updates in seconds.
        self.dt = float(dt)

        # x(k|k); the system state column vector.
        x = matrix([[0.0], [0.0], [0.0]])

        # P(k|k); the system covariance matrix.
        P = matrix([
            [10000.0, 0.0, 0.0],
            [0.0, 10000.0, 0.0],
            [0.0, 0.0, 10000.0]])

        self.state = (x, P)

        # Collects updated sensor data for the next update call.
        self.new_data = {}

        # Tracks the previous state of sensors for those that accumulate error over time.
        self.old_data = {
            # None indicates there is no previous data for this sensor.
            "odom": None
        }

    @staticmethod
    def column_vector(*args):
        """Utility function to construct a column vector (single-column matrix)."""
        return matrix([[e] for e in args])

    @staticmethod
    def rotation_matrix(theta):
        return matrix([
            [cos(theta), -sin(theta), 0],
            [sin(theta),  cos(theta), 0],
            [         0,           0, 1]])

    @staticmethod
    def coord_transform(state, offset):
        """Perform a coordinate transform on a state vector.

        state: a column vector of x, y, theta
        offset: a column vector of dx, dy, dtheta

        """
        dt = offset.item(2, 0)
        rotation = EKF.rotation_matrix(dt)
        return rotation * state + offset

    @staticmethod
    def get_offset(state_a, state_b):
        """Get the origin of reference frame B in A."""
        dt = state_a.item(2, 0) - state_b.item(2, 0)
        rotation = EKF.rotation_matrix(dt)
        return state_a - rotation * state_b

    def data_received(self, sensor, pose):
        if sensor in self.new_data:
            self.old_data[sensor] = self.new_data[sensor]
        self.new_data[sensor] = pose

    def get_pose(self):
        s, P = self.state
        pose = Pose()
        pose.header.frame_id = "world"
        pose.x = s.item(0, 0)
        pose.y = s.item(1, 0)
        pose.theta = s.item(2, 0)
        pose.cov = tuple(P.flat)
        return pose

    def update(self):
        """Perform an EKF state update."""

        # store the old state
        old_x, old_P = self.state

        # Velocity is a special case; if we have it, perform a prediction based
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
            W = matrix(pose.cov).reshape(3, 3)

            # odom accumulates error over time, so use the diff from last time.
            if sensor == "odom":
                old_pose = self.old_data[sensor]
                # can't do anything with the first odom measurement.
                if old_pose is None:
                    # Store the data for next time.
                    self.old_data[sensor] = pose
                    continue
                # remember, x is the EKF state and y is the sensor state.
                old_y = EKF.column_vector(old_pose.x, old_pose.y, old_pose.theta)
                old_W = matrix(pose.cov).reshape(3, 3)
                # get the odom frame origin in the map frame.
                odom_offset = EKF.get_offset(old_x, old_y)
                # transform both sensors states into the map frame.
                y_map = EKF.coord_transform(y, odom_offset)
                old_y_map = EKF.coord_transform(old_y, odom_offset)
                W_map = EKF.coord_transform(W, odom_offset)
                old_W_map = EKF.coord_transform(old_W, odom_offset)
                # apply the difference between them to the old state.
                y = y_map - old_y_map + old_x
                W = W_map - old_W_map + old_P
            else:
                assert pose.header.frame_id == "map"

            R = P * (P + W).I
            x = x + R * (y - x)
            P = P - R * P
            self.state = x, P
            self.old_data[sensor] = pose

        # Clear the new data dict.
        self.new_data.clear()

    def predict(self, u, V):
        # state vector, covariance matrix
        s, P = self.state
        # x, y, theta
        x, y, t = s.item(0, 0), s.item(1, 0), s.item(2, 0)
        # velocity, angular velocity (omega)
        v, w = u
        V = matrix(V).reshape(3, 3)
        # state prediction
        sp = EKF.column_vector(x + v * cos(t), y + self.dt * v * sin(t), t + self.dt * w)
        F = matrix([
            [1, 0, -self.dt * v * sin(t)],
            [0, 1, self.dt * v * cos(t)],
            [0, 0, 1]])
        # covariance prediction
        PP = F * P * F.transpose() + V
        return sp, PP
