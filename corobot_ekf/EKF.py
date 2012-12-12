
class EKF(object):

    """Implements an Extended Kalman Filter.

    Follows the EKF described in Priciples of Robotic Motion.

    """

    def __init__(self):
        self.initialized = False

        # x(k+1|k+1); the system state.
        self.x = np.zeros(3)

        # P(k+1|k+1); the system covariance matrix.
        self.P = matrix([[10000.0, 0.0, 0.0],
                         [0.0, 10000.0, 0.0],
                         [0.0, 0.0, 10000.0]])

        # x(k+1|k); the state prediction.
        self.xp = np.zeros(3)

        # P(k+1|k); the covariance prediction.
        self.PP = matrix([[10000.0, 0.0, 0.0],
                          [0.0, 10000.0, 0.0],
                          [0.0, 0.0, 10000.0]])

    def update(pose):
        pass

    def predict():
        pass

    def R():
        pass
