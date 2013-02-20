#!/usr/bin/env python
import roslib; roslib.load_manifest('corobot_comm')
import rospy
import socket
import math

from geometry_msgs.msg import Point
from corobot_msgs.srv import GetPixelOccupancy,GetNeighbors,GetLocation,GetWaypoints
from corobot_msgs.msg import Pose
from Queue import PriorityQueue

#Robot's current position.  Defaults to a test position.
myPose = Pose(x=26.5352,y=-7.7736,theta=0)

#Used to track last set goal from a user.
lastWP = None

'''
Pose subscription callback
'''
def pose_callback(pose):
    global myPose
    myPose = pose

'''
Can I straight line nav to this wp from current position?
'''
def navigableTo(wp):
    cx = myPose.x
    cy = myPose.y
    dx = wp.x-cx
    dy = wp.y-cy
    sdx = math.copysign(1,dx)
    sdy = math.copysign(1,dy)
    if(math.fabs(dx) > math.fabs(dy)):
        incx = sdx/2
        incy = dy/(dx/incx)
    else:
        incy = sdy/2
        incx = dx/(dy/incy)
    rospy.wait_for_service('get_pixel_occupancy')
    mapAt = rospy.ServiceProxy('get_pixel_occupancy',GetPixelOccupancy,persistent=True)
    while(sdx*dx > 0 or sdy*dy > 0):
        #Service request
        occ = mapAt(cx+dx,cy+dy).occupancy
        if(occ == 0):
            return False
        dx -= incx
        dy -= incy
    mapAt.close()
    return True

'''
Distance between two points

Arguments:
wp1x -- X value of the first point
wp1y -- Y value of the first point
wp2x -- X value of the second point
wp2y -- Y value of the second point
'''
def pointDistance(wp1x,wp1y,wp2x,wp2y):
    return math.sqrt(math.pow(wp2x-wp1x,2)+math.pow(wp2y-wp1y,2))

'''
Find nearest Waypoint to the current value of myPose

Arguments:
wps -- Waypoint[] with all waypoints in the graph/map

Returns a 2-tuple (float,Waypoint):
    (distance from current position to nearest navigable Waypoint, nearest navigable Waypoint)
    None if no nearby waypoint can be found
'''
def findNearestNavigable(wps):
    closest = None
    for wp in wps:
        if closest == None and navigableTo(wp):
            closest = (pointDistance(myPose.x,myPose.y,wp.x,wp.y),wp)
            continue
        dist = pointDistance(myPose.x,myPose.y,wp.x,wp.y)
        if not closest == None and dist < closest[0] and navigableTo(wp):
            closest = (dist,wp)
    if closest == None:
        rospy.logerr("Cannot find a nearby waypoint to begin navigation!")
        return None
    return closest[1]

'''
Perform A* to produce path of waypoints to given dest from nearest map waypoint.

Arguments:
dest -- Destination Waypoint
wps  -- List of Waypoints (Waypoint[]) representing full list of map waypoints.

Returns:
    Waypoint[] representing path to follow to the destination.
    Empty list if no path can be found.
'''
def aStar(dest,wps):
    near = findNearestNavigable(wps)
    if near == None:
        rospy.logerr("AStar navigation failed, couldn't find a starting node.")
        return []
    rospy.logdebug("WaypointClosestToMe: {}".format(near.name))
    preds = {near.name:None}
    pq = PriorityQueue()
    openSet = [near]
    visited = []
    #dict holding {waypoint name:distance from robot to waypoint} pairs
    gScores = {near.name:pointDistance(myPose.x,myPose.y,near.x,near.y)}
    #pq elements are (g+h,node)
    # g=distRobotWp, h=distWpGoal
    pq.put((gScores[near.name]+pointDistance(near.x,near.y,dest.x,dest.y),
        near))
    #Set up persistent connection to the GetNeighbors service
    rospy.wait_for_service('get_neighbors')
    getNeighbors = rospy.ServiceProxy('get_neighbors',GetNeighbors,persistent=True)
    while(not(pq.empty())):
        curr = pq.get()
        cnode = curr[1]
        
        if(cnode.name==dest.name):
            #Found the path! Now build it.
            path = []
            pnode = dest
            while(not(pnode==None)):
                pname = pnode.name
                path.insert(0,pnode)
                pnode = preds[pname]
            return path

        openSet.remove(cnode)
        visited.append(cnode)
        for nbr in getNeighbors(cnode).neighbors:
            if(nbr in visited):
                continue
            tentG = gScores[cnode.name]+pointDistance(cnode.x,cnode.y,nbr.x,nbr.y)
            if(not(nbr in openSet)or(tentG<gScores[nbr.name])):
                preds[nbr.name]=cnode
                gScores[nbr.name]=tentG
                pq.put((gScores[nbr.name]+pointDistance(nbr.x,nbr.y,dest.x,dest.y),nbr))
                if(not(nbr in openSet)):
                    openSet.append(nbr)
    #Cleanup persistent service connection.
    getNeighbors.close()
    return []

'''
Begin client API communication

Arguments:
socket -- Active socket to a connected client
addr -- Client's IP address
'''
def clientComm(socket,addr):
    rospy.init_node('corobot_client_comm')
    clIn = socket.makefile('r')
    clOut = socket.makefile('w')
    #Publisher to obstacle_avoidance
    pointPub = rospy.Publisher('waypoints',Point)
    rospy.Subscriber('pose',Pose,pose_callback)
    
    while True:
        cmd = clIn.readline()
        if len(cmd) == 0:
            clIn.close()
            clOut.close()
            break
        cmd = cmd.strip().split(' ')
        rospy.logdebug("Command recieved from client %s: %s", addr, cmd)

        #Command processing
        if cmd[0] == 'GETPOS':
            clOut.write("POS {} {} {}\n".format(str(myPose.x),str(myPose.y),str(myPose.theta)))
            clOut.flush()
        elif cmd[0] == 'GOTOXY':
            #Add dest point!
            pointPub.publish(x=float(cmd[1]),y=float(cmd[2]))
        elif cmd[0] == 'GOTOLOC':
            #Goto location, no navigation
            rospy.wait_for_service('get_location')
            try:
                getLoc = rospy.ServiceProxy('get_location',GetLocation)
                #returns Waypoint
                resp = getLoc(cmd[1])
                pointPub.publish(x=resp.wp.x,y=resp.wp.y)
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: {}".format(e))
        elif cmd[0] == 'NAVTOLOC':
            #Navigate to location
            rospy.wait_for_service('get_waypoints')
            rospy.wait_for_service('get_location')
            try:
                getLoc = rospy.ServiceProxy('get_location',GetLocation)
                getWps = rospy.ServiceProxy('get_waypoints',GetWaypoints)
                #Gets waypoints, no neighbor data...maybe I should change that ~Karl
                # wps is a Waypoint[]
                wps = getWps().allWPs
                start = getLoc(cmd[1])
                path = aStar(start.wp,wps)
                for node in path:
                    pointPub.publish(x=node.x,y=node.y)
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: {}".format(e))
        elif cmd[0] == 'QUERY_ARRIVE':
            print("Query_Arrive")
            #How to figure this out?!

def main():
    serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    serversocket.bind((socket.gethostname(),15001))
    serversocket.listen(1)

    while True:
        (client, clAddr) = serversocket.accept()
        #On connection accept, go into ROS node method
        clientComm(client,clAddr)

if __name__ == '__main__':
    main()