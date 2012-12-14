from math import cos, sin

class EKF(object):

    """Implements an Extended Kalman Filter.

    Follows the EKF described in Principles of Robotic Motion.

    """

    def __init__(self, dt):

        # Time delta between updates.
        self.dt = dt

        # x(k|k); the system state.
        x = np.zeros(3)

        # P(k|k); the system covariance matrix.
        P = array([
            [10000.0, 0.0, 0.0],
            [0.0, 10000.0, 0.0],
            [0.0, 0.0, 10000.0]])

        self.state = (x, P)

        # x(k+1|k); the state prediction.
        xp = np.zeros(3)

        # P(k+1|k); the covariance prediction.
        PP = array([
            [10000.0, 0.0, 0.0],
            [0.0, 10000.0, 0.0],
            [0.0, 0.0, 10000.0]])

        self.pred = (xp, PP)

        # Tracks the previous state of sensors.
        self.old = {}

    # update :: string -> Pose -> void
    def update(self, sensor, pose):
        if sensor not in self.old:
            return


    def predict(self, stamp, u, V):
        (x, y, t), P = self.state
        v, w = u
        xp = array([x + v * cos(t), y + self.dt * v * sin(t), t + self.dt * w])
        F = array([
            [1, 0, -self.dt * v * sin(t)],
            [0, 1, selt.dt * v * cos(t)],
            [0, 0, 1]])
        PP = F * P * np.transpose(F) + V
        return xp, PP

    def R():
        pass
