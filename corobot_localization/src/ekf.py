from math import pi, copysign, sqrt, cos, sin, fmod

from numpy.matlib import matrix

from corobot_common.msg import Pose
from utils import column_vector, coord_transform, get_offset, reduce_covariance
import rospy

#from raj_test import *

tau = pi * 2.0
ACCEL_ANGLE_BONUS = 0.052 # should come from a file of calibration data

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
        self.lastposdt = rospy.get_time() - 1
        self.lastnegdt = rospy.get_time() - 1 
        self.lastqr = None
        self.lastlaser = None
        self.useqr = True
        self.uselaser = True

    def state_tuple(self, state = None):
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

    def update_pos(self, pose, absolute=False):
        """Convenience function to do a position update."""
        # assuming that the ekf is never lost by more than pi (except on startup),
        # we can intelligently mod the incoming pose angle to be close to the current estimate
        # note that the ordered if statements mean that if we are super-lost, we
        # will randomly add some pi but not infinitely loop.
        # this if statement says that if we're lost in theta, don't worry about it.
        if self.covariance[2,2] < 10:
            while pose.theta - self.state[2] > pi:
                pose.theta -= tau
            while self.state[2] - pose.theta > pi:
                pose.theta += tau
        # if we have absolute (i.e. QR-code) data, and it's far from our current
        # estimate, then assume we're lost and reset the EKF.
        if absolute:
                dx = self.state[0]-pose.x
                dy = self.state[1]-pose.y
                dt = fmod(self.state[2]-pose.theta,2*pi)
                if dx*dx + dy*dy > 0.5 or abs(dt) > 0.5:
                        self.state = column_vector(pose.x, pose.y, pose.theta)
                        self.covariance = matrix(pose.cov).reshape(3, 3)
                        return
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
        while dt > pi:
            dt -= tau
        while dt < -pi:
            dt += tau
        #skip the update if we aren't moving
        if abs(dx) < 0.00001 and abs(dy) < 0.00001 and abs(dt) < 0.00001:
            return

        dist = sqrt(dx*dx + dy*dy)
        if abs(dt) > 0:
            print("Nonzero rotation detected at",rospy.get_time(), \
                    "Odom vel",dist*30)
            if dist*30 < 0.1:
                if ((dt > 0 and rospy.get_time() - self.lastposdt > 0.25) \
                 or (dt < 0 and rospy.get_time() - self.lastnegdt > 0.25)):
                    # give the acceleration compensation
                    bonus = copysign(ACCEL_ANGLE_BONUS,dt)
                    print("Adding",bonus,"to robot dt")
                    delta[2] += bonus
                if dt > 0:
                    self.lastposdt = rospy.get_time()
                else:
                    self.lastnegdt = rospy.get_time()
        # State prediction; nice and simple.
        self.state = self.state + delta
        # Use 50% of the delta x/y values as covariance, and 200% theta.

        fwderr = 0.2 * dist
        sideerr = 0.1 * dist
        yawerr  = 0.5 * dt + 0.1 * dist * 4
        cth = cos(odom_pose.theta)
        sth = sin(odom_pose.theta)
        dev = matrix([cth*fwderr - sth*sideerr, sth*fwderr + cth*sideerr, yawerr])
        V = dev.transpose()*dev
        rospy.loginfo("Odom cov: %s",V)

        '''
        V = matrix([
                [cth*cth*fwderr + sth*sth*sideerr, sth*cth*(sideerr - fwderr)      , 0.0   ],
                [sth*cth*(sideerr - fwderr)      , sth*sth*fwderr + cth*cth*sideerr, 0.0   ],
                [0.0                             , 0.0                             , yawerr]])
        '''
        """
        V = matrix([
            [abs(dx) * 0.5, 0.0, 0.0],
            [0.0, abs(dy) * 0.5, 0.0],
            [0.0, 0.0, abs(dt) * 2.0]])
        """

        # Covariance prediction; add our custom covariance assumptions.
        #vel = dist * 30.0
        F = matrix([
                [1.0, 0.0, -sin(odom_pose.theta) * dist],
                [0.0, 1.0, cos(odom_pose.theta) * dist],
                [0.0, 0.0, 1.0]])
                
        self.covariance = F*self.covariance*F.transpose() + V
        # now we can get a sensor update again:
        self.uselaser = True
        self.useqr = True

