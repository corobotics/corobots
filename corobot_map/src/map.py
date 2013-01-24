#!/usr/bin/env python
import roslib; roslib.load_manifest('corobot_map')
import rospy
import sys

from nav_msgs.srv import *
from corobot_msgs.msg import Waypoint
from corobot_msgs.srv import *

wploc = roslib.packages.get_pkg_dir('corobot_map') + "/map/waypoints.csv"
wps = {}

def handle_get_map(req):
	return map_server_client()
def handle_get_waypoints(req):
	return get_waypoints()
def handle_get_neighbor(req):
	return get_neighbors(req)

def map_server_client():
	rospy.wait_for_service('static_map')
	try:
		static_map = rospy.ServiceProxy('static_map', GetMap)
		resp = static_map()
		return GetMapResponse(resp.map)
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def get_waypoints():
	return None

def get_neighbors(req):
	neighbors = wps[req.name][2]
	waypoints = []
	for neighbor in neighbors:
		node = wps[neighbor]
		waypoints.append( Waypoint(node[0],node[1],neighbor) )
	return GetNeighborsResponse(waypoints)

def loadWaypoints():
	with open(wploc, 'r') as wpFile:
		first = True
		for line in wpFile:
			if first:
				first = False
				continue
			vals = line.strip().split(',')
			neighborList = []
			for neighbor in vals[6:]:
				if neighbor != "":
					neighborList.append(neighbor)
			wps[vals[0]]=float(vals[3]),float(vals[4]),neighborList

'''This node is acting as a relay for image-driven map data and then
ties in our waypoint/location data'''
def main():
	#locations.csv parsing.
	loadWaypoints()

	rospy.init_node('corobot_map_server')
	rospy.Service('get_map', GetMap, handle_get_map)
	rospy.Service('get_waypoints', GetWaypoints, handle_get_waypoints)
	rospy.Service('get_neighbors', GetNeighbors, handle_get_neighbors)
	print "Ready to serve map."
	rospy.spin()

if __name__ == "__main__":
	main()
