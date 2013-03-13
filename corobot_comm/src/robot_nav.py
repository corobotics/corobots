#!/usr/bin/env python
import roslib; roslib.load_manifest('corobot_comm')
import rospy
import math

from geometry_msgs.msg import Point
from corobot_msgs.srv import GetPixelOccupancy,GetNeighbors,GetLocation,GetWaypoints
from corobot_msgs.msg import Pose,Goal
from Queue import PriorityQueue
from collections import deque

#Robot's current position.  Defaults to a test position.
myPose = Pose(x=26.3712,y=-7.7408,theta=0) # NE Atrium

#Used to track set goals from a user.
# Queue of (node,isGoal?) pairs
wpQueue = deque()

def pose_callback(pose):
    '''
    Pose subscription callback
    '''
    global myPose
    myPose = pose

def waypoints_reached_callback(wp):
    '''
    Wayoints Reached subscription callback
    '''
    top = wpQueue[0]
    if top[0].x == wp.x and top[0].y == wp.y:
        wpQueue.popleft()
        if top[1] == True:
            goalReachedPub = rospy.Publisher('goals_reached',Goal)
            goalReachedPub.publish(Goal(top[0].name))

def goals_callback(new_goal):
    '''
    Goals subscription callback.
    '''
    rospy.wait_for_service('get_waypoints')
    rospy.wait_for_service('get_location')

    try:
        #Publisher to obstacle_avoidance
        pointPub = rospy.Publisher('waypoints',Point)
        
        getLoc = rospy.ServiceProxy('get_location',GetLocation)
        getWps = rospy.ServiceProxy('get_waypoints',GetWaypoints)
        #Gets waypoints, no neighbor data...maybe I should change that ~Karl
        # wps is a Waypoint[]
        wps = getWps().allWPs
        end = getLoc(cmd[1])
        path = aStar(end.wp,wps)
        for node in path:
            pointPub.publish(x=node.x,y=node.y)
            if node.name == end.name:
                wpQueue.append((node,True))
            wpQueue.append((node,False))
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: {}".format(e))

def navigableTo(wp):
    '''
    Can I straight line nav to this wp from current position?
    '''
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
    #rospy.logerr("Occupancy of curr location X:{} Y:{} Occ:{}".format(cx,cy,mapAt(cx,cy).occupancy))
    while(sdx*dx > 0 or sdy*dy > 0):
        #Service request
        occ = mapAt(cx+dx,cy+dy).occupancy
        if(occ == 0):
            return False
        dx -= incx
        dy -= incy
    mapAt.close()
    return True

def pointDistance(wp1x,wp1y,wp2x,wp2y):
    '''
    Distance between two points

    Arguments:
    wp1x -- X value of the first point
    wp1y -- Y value of the first point
    wp2x -- X value of the second point
    wp2y -- Y value of the second point
    '''
    return math.sqrt(math.pow(wp2x-wp1x,2)+math.pow(wp2y-wp1y,2))

def findNearestNavigable(wps):
    '''
    Find nearest Waypoint to the current value of myPose

    Arguments:
    wps -- Waypoint[] with all waypoints in the graph/map

    Returns a 2-tuple (float,Waypoint):
        (distance from current position to nearest navigable Waypoint, nearest navigable Waypoint)
        None if no nearby waypoint can be found
    '''
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

def aStar(dest,wps):
    '''
    Perform A* to produce path of waypoints to given dest from nearest map waypoint.

    Arguments:
    dest -- Destination Waypoint
    wps  -- List of Waypoints (Waypoint[]) representing full list of map waypoints.

    Returns:
        Waypoint[] representing path to follow to the destination.
        Empty list if no path can be found.
    '''
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

def main():
    rospy.init_node('robot_navigator')
    #Publisher to obstacle_avoidance
    pointPub = rospy.Publisher('waypoints',Point)
    rospy.Subscriber('goals',Goal,goals_callback)
    rospy.Subscriber('pose',Pose,pose_callback)

    rospy.spin()

if __name__ == '__main__':
    main()