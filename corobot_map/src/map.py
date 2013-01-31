#!/usr/bin/env python
import roslib; roslib.load_manifest('corobot_map')
import rospy
import sys

from nav_msgs.srv import *
from corobot_msgs.msg import Waypoint
from corobot_msgs.srv import *

wpfile = roslib.packages.get_pkg_dir('corobot_map') + "/map/waypoints.csv"
wps = {}

def handle_get_map(req):
	return map_server_client()
def handle_get_waypoints(req):
	return GetWaypointsResponse(get_waypoints())
def handle_get_neighbors(req):
	return GetNeighborsResponse(get_neighbors(req.curr.name))

def map_server_client():
	rospy.wait_for_service('static_map')
	try:
		static_map = rospy.ServiceProxy('static_map', GetMap)
		resp = static_map()
		return GetMapResponse(resp.map,get_waypoints())
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

#Builds and returns Waypoint[] from the graph data, no neighbor data included.
def get_waypoints():
	wpList = []
	for k in wps.keys():
		wpList.append( Waypoint(wps[k][0],wps[k][1],k) )
	return wpList

#Builds and returns Waypoint[] of neighbors to the given waypoint name.
def get_neighbors(nodeName):
	neighbors = wps[nodeName][2]
	waypoints = []
	for neighbor in neighbors:
		node = wps[neighbor]
		waypoints.append( Waypoint(node[0],node[1],neighbor) )
	return waypoints

#Parses waypoint "graph" data and builds a dictionary out of the info.
def loadWaypoints():
	with open(wpfile, 'r') as wpFile:
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
			#wps[wp_name] = (X_Meters,Y_Meters,[neighbor0,neighbor1,...])
			wps[vals[0]]=float(vals[3]),float(vals[4]),neighborList

'''This node is acting as a relay for image-driven map data and then
ties in our waypoint/location data'''
def main():
	loadWaypoints()

	rospy.init_node('corobot_map_server')
	rospy.Service('get_map', GetMap, handle_get_map)
	rospy.Service('get_waypoints', GetWaypoints, handle_get_waypoints)
	rospy.Service('get_neighbors', GetNeighbors, handle_get_neighbors)
	print "Ready to serve map."
	rospy.spin()

if __name__ == "__main__":
	main()
