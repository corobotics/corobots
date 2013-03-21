#!/usr/bin/env python
import sys
import math

import roslib; roslib.load_manifest('corobot_map')
import rospy
from nav_msgs.srv import GetMap

from corobot_msgs.msg import Landmark
from corobot_msgs.srv import *  #Importing star because I'm using 95% of the 
                                #services defined here and I'm not importing 
                                #all 3 forms of the service definition.

wp_filename = roslib.packages.get_pkg_dir('corobot_map') + "/map/waypoints.csv"
wps = {}
occ_map = None

#Service callbacks
def handle_get_co_map(req):
    if occ_map == None:
        load_map()
    return GetCoMapResponse(occ_map, get_waypoints())

def handle_get_landmarks(req):
    return GetLandmarksResponse(get_waypoints())

def handle_get_neighbors(req):
    return GetNeighborsResponse(get_neighbors(req.curr.name.upper()))

def handle_get_landmark(req):
    return GetLandmarkResponse(Landmark(x=wps[req.name][0],y=wps[req.name][1],name=req.name))

def handle_get_pixel_occupancy(req):
    if occ_map == None:
        load_map()
    p_x = int(math.floor(req.x/occ_map.info.resolution))
    p_y = int(math.floor(req.y/occ_map.info.resolution))
    off = p_y*occ_map.info.width + p_x
    #rospy.logerr("Occupancy of requested pixel X:{} Y:{} Occ:{}".format(p_x,p_y,occ_map.data[off]))
    return GetPixelOccupancyResponse(occ_map.data[off])

#Utility methods.
def load_map():
    """Pulls map data from static_map service provided by map_server"""
    global occ_map
    rospy.wait_for_service('static_map')
    try:
        static_map = rospy.ServiceProxy('static_map', GetMap)
        occ_map = static_map().map
    except rospy.ServiceException as e:
        print("Service call failed: {}".format(e))

def get_waypoints():
    """Builds and returns a Landmark[] from the graph data, no neighbor data included."""
    wp_list = []
    for k in wps.keys():
        wp_list.append( Landmark(wps[k][0],wps[k][1],k) )
    return wp_list

def get_neighbors(nodeName):
    """Builds and returns Landmark[] of neighbors to the given waypoint name."""
    neighbors = wps[nodeName][2]
    waypoints = []
    for nbr in neighbors:
        node = wps[nbr]
        waypoints.append(Landmark(node[0],node[1],nbr))
    return waypoints

def load_waypoints():
    """Parses waypoint "graph" data and builds a dictionary out of the info."""
    with open(wp_filename, 'r') as wp_file:
        global wps
        first = True
        for line in wp_file:
            if first:
                first = False
                continue
            vals = line.strip().split(',')
            neighbor_list = []
            for neighbor in vals[6:]:
                if neighbor != "":
                    neighbor_list.append(neighbor.upper())
            #wps[wp_name] = (X_Meters,Y_Meters,[neighbor0,neighbor1,...])
            wps[vals[0].upper()]=float(vals[3]),float(vals[4]),neighbor_list
    #rospy.logerr(wps)

def main():
    """This node is acting as a relay for image-driven map data and then
    ties in our waypoint/location data
    """
    load_waypoints()

    rospy.init_node('corobot_map_server')
    rospy.Service('get_map', GetCoMap, handle_get_co_map)
    rospy.Service('get_landmarks', GetLandmarks, handle_get_landmarks)
    rospy.Service('get_neighbors', GetNeighbors, handle_get_neighbors)
    rospy.Service('get_landmark', GetLandmark, handle_get_landmark)
    rospy.Service('get_pixel_occupancy', GetPixelOccupancy, handle_get_pixel_occupancy)
    print("Ready to serve map.")
    rospy.spin()

if __name__ == "__main__":
    main()
