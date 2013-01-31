#!/usr/bin/env python
import roslib; roslib.load_manifest('corobot_map')
import rospy

from corobot_msgs.msg import Waypoint
from corobot_msgs.srv import *

def testGetNeighbors():
    rospy.wait_for_service('get_neighbors')
    node = Waypoint(0,0,'WHall1')
    neighbors = ['SWInter', 'WHall2', 'Office3573', 'Office3571', 'Office3569',
     'Office3559', 'Office3557', 'Class3560']
    neighbors = [x.upper() for x in neighbors]
    resp = rospy.ServiceProxy('get_neighbors', GetNeighbors)
    retNeighbors = resp(node).neighbors
    retList = []
    for item in retNeighbors:
        retList.append(item.name)
    if not( set(neighbors) == set(retList) ):
        print( neighbors )
        print( retList )
    else:
        print( "NEIGHBORS: SUCCESS!" )

def testGetWaypoints():
    rospy.wait_for_service('get_waypoints')
    resp = rospy.ServiceProxy('get_waypoints', GetWaypoints)
    num = 0
    for wp in resp().allWPs:
        num+=1
        print( wp.name + "- x: " + str(wp.x) + " y: " + str(wp.y) )
    print( num )

def main():
    testGetNeighbors()
    testGetWaypoints()

if __name__ == "__main__":
    main()
