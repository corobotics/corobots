#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from corobot_obstacle_avoidance_testing.msg import Wall
import numpy as np
import matplotlib
import matplotlib.cm as cm
import matplotlib.mlab as mlab
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class WallViz():

	def __init__(self):
		self.testing()
		rospy.Subscriber("wall", Wall, self.wallDisp_callback, queue_size = 1)
		rospy.spin()

	def wallDisp_callback(self,wallDat):
		print "here"
		#print type(wallDat.accumulator)
		acc = wallDat.accumulator
		size = len(acc)
		width = wallDat.tdiv
		height = wallDat.height
		print "width= " + str(width)
		print "height= " + str(height)	
		x = np.linspace(0, width, width)
		y = np.linspace(0, height, height)
		z = np.zeros((len(x), len(y)))
		for i in range(len(x)):
			for j in range(len(y)):
				z[i, j] = acc[(j*width)+i]
		fig = plt.figure()
		#ax = fig.add_subplot(111, projection='3d')
		CS = plt.contour(y, x, z)
		#ax.plot_surface(y,x,z)
		plt.show()

	def testing(self):
		rospy.init_node("NewNode")
		print "Hello!!!"

def main():
	wv = WallViz()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
