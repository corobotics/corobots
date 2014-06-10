from corobot_common.msg import Pose
import rospy

from utils import column_vector, coord_transform, get_offset

from numpy.matlib import matrix, linalg
from math import sqrt, cos, sin, pi

from raj_test import *

tau = pi * 2.0

class EKF(object):

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
        	self.lastdt = rospy.get_time() - 1

	def state_tuple(self, state=None):
		""" Return a tuple of the given state matrix: (x, y, theta) """
		if state is None:
			state = self.state

		return tuple(state.T.tolist()[0])

	def get_pose(self):
		pose = Pose()
		pose.header.frame_id = "world"
		pose.x, pose.y, pose.theta = self.state_tuple()
		pose.cov = tuple(self.covariance.flat)
		return pose

	def get_odom_delta(self, pose):
		# odom_state is the previous x, y, theta of odom_pose
		# y_odom is the current x, y, theta of odom_pose
		# odom_origin kind keeps track of all the odom changes till date
		
		y_odom = column_vector(pose.x, pose.y, pose.theta)

		odom_delta = None
		
		if self.odom_state is not None:
			# y = y_odom * rot(odom_origin.theta) + odom_origin
			y = coord_transform(y_odom, self.odom_origin)
			# current change - previous change			
			odom_delta = y - coord_transform(self.odom_state, self.odom_origin)

		self.odom_state = y_odom
		# these two should be exactly the same. Using y_odom to 
		# differentiate between previous and new odom values
		#self.odom_origin = get_offset(self.state, self.odom_state)
		self.odom_origin = get_offset(self.state, y_odom)
		return odom_delta

	def update_pos(self, pose):
		if self.covariance[2,2] < 1000:
			while pose.theta - self.state[2] > pi:
				pose.theta -= tau
			while self.state[2] - pose.theta > pi:
				pose.theta += tau
		y = column_vector(pose.x, pose.y, pose.theta)
		W = matrix(pose.cov).reshape(3, 3)
		self.update2(y, W)

	def update2(self, y, W):
		x, P = self.state, self.covariance
		R = P * (P + W).I
		self.state = x + R * (y - x)
		self.covariance = P - R * P

	def update(self, scan):

	        currentPose = self.state

		currentCov = self.covariance
	        samplePoints = get_sample_points(currentPose)
	        sampleProbability = []
	
		#sample = self.state
        	for samp in samplePoints:
        	        newLaser = get_expected_scan(samp)
			currentProbability = get_sample_probability(scan.ranges, newLaser)
			sampleProbability.append(currentProbability)

        	sum_x = 0.0
        	sum_y = 0.0
        	sum_theta = 0.0
        	count = 0.0

        	for i in range(0, len(samplePoints)):
			samp = samplePoints[i]	
        	        sum_x += sampleProbability[i] * samp[0]
        	        sum_y += sampleProbability[i] * samp[1]
        	        sum_theta += sampleProbability[i] * samp[2]
               		count = count + sampleProbability[i]

        	mean_pose = [sum_x/count, sum_y/count, sum_theta/count]

        	sum_cx = 0.0
        	sum_cy = 0.0
        	sum_ctheta = 0.0
        	sum_xy = 0.0
        	sum_xt = 0.0
        	sum_yt = 0.0
        	cCount = len(samplePoints) - 1
        	for i in range(0, len(samplePoints)):
			samp = samplePoints[i]
        	        xx = (samp[0] - mean_pose[0])
        	        yy = (samp[1] - mean_pose[1])
        	        tt = (samp[2] - mean_pose[2])

        	        sum_cx += xx * xx
        	        sum_cy += yy * yy
               		sum_ctheta += tt * tt
                	sum_xy += xx * yy
	                sum_xt += xx * tt
	                sum_yt += yy * tt

	        covariance_pose = matrix([[sum_cx/cCount, sum_xy/cCount, sum_xt/cCount],
       		                   [sum_xy/cCount, sum_cy/cCount, sum_yt/cCount],
                	           [sum_xy/cCount, sum_yt/cCount, sum_ctheta/cCount]])

		W = covariance_pose
		P = currentCov
		x = currentPose
		I = matrix([[1.0, 0.0, 0.0],
			    [0.0, 1.0, 0.0],
			    [0.0, 0.0, 1.0]])

		# I = Identity Matrix
		# v = actualScan - CalculatedScan
		# v = y - H * x 
		# y = LaserScan from Kinect
		# H * x = Calculated Scan using Bresenhem
		# P = predicted covariance matrix for Pose
		# W = Calculated(should have been actual) Covarience matrix of output(laserScan) in terms on pose
		# x = predicted Pose
		ay = [i[0] for i in currentPose.tolist()]
		v = matrix([[ay[0] - mean_pose[0]], [ay[1] - mean_pose[1]], [ay[2] - mean_pose[2]]])
		print ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"
		print P
		S = I * P * I + W
		R = P * I * linalg.inv(S)
		x = x + R * v
		P = P - R * I * P
		print P
		print "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<"

	def predict(self, odom_pose):
		delta = self.get_odom_delta(odom_pose)
		
		if delta is None:
			# Can't do a delta update the first time.
			return

		dx, dy, dt = self.state_tuple(delta)
		
		# Give bonus change in angel since odom cannot
		# read that properly	
		if abs(dt) > 0:
			if rospy.gettime() - self.lastdt > 0.25:
				bonus = copysign(ACCEL_ANGEL_BONUS, dt)
				delta[2] += bonus

			self.lastdt = rospy.get_time
		
		# PREDICT NEW STATE (X, Y, THETA)
		self.state = self.state + delta

		dist = sqrt(dx*dx + dy*dy)
		fwderr  = 0.05 * dist
		sideerr = 0.1 * dist
		yawerr  = 0.2 * dist
		cth = cos(odom_pose.theta)
		sth = sin(odom_pose.theta)
		
		V = matrix([
			[cth*cth*fwderr + sth*sth*sideerr, sth*cth*(sideerr - fwderr)      , 0.0   ],
			[sth*cth*(sideerr - fwderr)      , sth*sth*fwderr + cth*cth*sideerr, 0.0   ],
			[0.0                             , 0.0                             , yawerr]])
		# PREDICT COVARIANCE
		self.covariance = self.covariance + V
