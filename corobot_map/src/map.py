#!/usr/bin/env python
import roslib; roslib.load_manifest('corobot_map')
import rospy
import sys
import math

from nav_msgs.srv import GetMap
from corobot_msgs.msg import Waypoint
from corobot_msgs.srv import *

wpfile = roslib.packages.get_pkg_dir('corobot_map') + "/map/waypoints.csv"
wps = {}
occMap = None

#Service callbacks
def handle_get_co_map(req):
    if occMap == None:
        load_map()
    return GetCoMapResponse(occMap, get_waypoints())

def handle_get_waypoints(req):
    return GetWaypointsResponse(get_waypoints())

def handle_get_neighbors(req):
    return GetNeighborsResponse(get_neighbors(req.curr.name.upper()))

def handle_get_landmark(req):
    return GetLandmarkResponse(Waypoint(x=wps[req.name][0],y=wps[req.name][1],name=req.name))

def handle_get_pixel_occupancy(req):
    if occMap == None:
        load_map()
    pX = int(math.floor(req.x/occMap.info.resolution))
    pY = int(math.floor(req.y/occMap.info.resolution))
    off = pY*occMap.info.width + pX
    #rospy.logerr("Occupancy of requested pixel X:{} Y:{} Occ:{}".format(pX,pY,occMap.data[off]))
    return GetPixelOccupancyResponse(occMap.data[off])

#Utility methods.
def load_map():
    """Pulls map data from static_map service provided by map_server"""
    global occMap
    rospy.wait_for_service('static_map')
    try:
        static_map = rospy.ServiceProxy('static_map', GetMap)
        occMap = static_map().map
    except rospy.ServiceException as e:
        print("Service call failed: {}".format(e))

def get_waypoints():
    """Builds and returns a Waypoint[] from the graph data, no neighbor data included."""
    wpList = []
    for k in wps.keys():
        wpList.append( Waypoint(wps[k][0],wps[k][1],k) )
    return wpList

def get_neighbors(nodeName):
    """Builds and returns Waypoint[] of neighbors to the given waypoint name."""
    neighbors = wps[nodeName][2]
    waypoints = []
    for nbr in neighbors:
        node = wps[nbr]
        waypoints.append( Waypoint(node[0],node[1],nbr) )
    return waypoints

def load_waypoints():
    """Parses waypoint "graph" data and builds a dictionary out of the info."""
    with open(wpfile, 'r') as wpFile:
        global wps
        first = True
        for line in wpFile:
            if first:
                first = False
                continue
            vals = line.strip().split(',')
            neighborList = []
            for neighbor in vals[6:]:
                if neighbor != "":
                    neighborList.append(neighbor.upper())
            #wps[wp_name] = (X_Meters,Y_Meters,[neighbor0,neighbor1,...])
            wps[vals[0].upper()]=float(vals[3]),float(vals[4]),neighborList
    #rospy.logerr(wps)

def main():
    """This node is acting as a relay for image-driven map data and then
    ties in our waypoint/location data
    """
    load_waypoints()

    rospy.init_node('corobot_map_server')
    rospy.Service('get_map', GetCoMap, handle_get_co_map)
    rospy.Service('get_waypoints', GetWaypoints, handle_get_waypoints)
    rospy.Service('get_neighbors', GetNeighbors, handle_get_neighbors)
    rospy.Service('get_landmark', GetLandmark, handle_get_landmark)
    rospy.Service('get_pixel_occupancy', GetPixelOccupancy, handle_get_pixel_occupancy)
    print("Ready to serve map.")
    rospy.spin()

if __name__ == "__main__":
    main()
