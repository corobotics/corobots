#!/usr/bin/env python
import roslib; roslib.load_manifest('corobot_map')
import rospy

from nav_msgs.srv import *

def handle_get_map(req):
	return map_server_client()

def map_server_client():
	rospy.wait_for_service('static_map')
	try:
		static_map = rospy.ServiceProxy('static_map', GetMap)
		resp = static_map()
		return resp.map
	except: rospy.ServiceException, e:
		print "Service call failed: %s"%e

'''This node is acting as a relay for image-driven map data and then
ties in our waypoint/location data'''
def main():
	rospy.init_node('corobot_map_server')
	rospy.Service('get_map', GetMap, handle_get_map)
	print "Ready to serve map."
	rospy.spin()

if __name__ == "__main__":
	main()
