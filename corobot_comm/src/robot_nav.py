#!/usr/bin/env python
import roslib; roslib.load_manifest('corobot_comm')
import rospy
import math

from geometry_msgs.msg import Point
from corobot_msgs.srv import GetPixelOccupancy,GetNeighbors,GetLandmark,GetLandmarks
from corobot_msgs.msg import Pose,Landmark
from Queue import PriorityQueue
from collections import deque

#Robot's current position.  Defaults to a test position.
my_pose = Pose(x=26.3712,y=-7.7408,theta=0) # NE Atrium

#Used to track set goals from a user.
# Queue of (node,isGoal?) pairs
wpQueue = deque()

def pose_callback(pose):
    """Pose subscription callback"""
    global my_pose
    my_pose = pose

def waypoints_reached_callback(wp):
    """Wayoints Reached subscription callback"""
    top = wpQueue[0]
    if top[0].x == wp.x and top[0].y == wp.y:
        wpQueue.popleft()
        if top[1] == True:
            goalReachedPub = rospy.Publisher('goals_reached',Landmark)
            goalReachedPub.publish(wp)

def goals_callback(new_goal):
    """Goals subscription callback."""
    rospy.wait_for_service('get_waypoints')

    try:
        #Publisher to obstacle_avoidance
        pointPub = rospy.Publisher('waypoints',Point)
        
        getWps = rospy.ServiceProxy('get_waypoints',GetLandmarks)
        #Gets waypoints, no neighbor data...maybe I should change that ~Karl
        # wps is a Landmark[]
        wps = getWps().allWPs
        end = new_goal
        path = a_star(end,wps)
        for node in path:
            pointPub.publish(x=node.x,y=node.y)
            if node.name == end.name:
                wpQueue.append((node,True))
            wpQueue.append((node,False))
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: {}".format(e))

def navigable_to(wp):
    """Can I straight line nav to this wp from current position?"""
    cx = my_pose.x
    cy = my_pose.y
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
    map_at = rospy.ServiceProxy('get_pixel_occupancy',GetPixelOccupancy,persistent=True)
    #rospy.logerr("Occupancy of curr location X:{} Y:{} Occ:{}".format(cx,cy,map_at(cx,cy).occupancy))
    while(sdx*dx > 0 or sdy*dy > 0):
        #Service request
        occ = map_at(cx+dx,cy+dy).occupancy
        if(occ == 0):
            return False
        dx -= incx
        dy -= incy
    map_at.close()
    return True

def point_distance(wp1x, wp1y, wp2x, wp2y):
    """Distance between two points

    Arguments:
    wp1x -- X value of the first point
    wp1y -- Y value of the first point
    wp2x -- X value of the second point
    wp2y -- Y value of the second point
    """
    return math.sqrt(math.pow(wp2x-wp1x, 2)+math.pow(wp2y-wp1y, 2))

def find_nearest_navigable(wps):
    """Find nearest Landmark to the current value of my_pose

    Arguments:
    wps -- Landmark[] with all waypoints in the graph/map

    Returns a 2-tuple (float,Landmark):
        (distance from current position to nearest navigable Landmark, nearest navigable Landmark)
        None if no nearby waypoint can be found
    """
    closest = None
    for wp in wps:
        if closest == None and navigable_to(wp):
            closest = (point_distance(my_pose.x,my_pose.y,wp.x,wp.y),wp)
            continue
        dist = point_distance(my_pose.x,my_pose.y,wp.x,wp.y)
        if not closest == None and dist < closest[0] and navigable_to(wp):
            closest = (dist,wp)
    if closest == None:
        rospy.logerr("Cannot find a nearby waypoint to begin navigation!")
        return None
    return closest[1]

def a_star(dest,wps):
    """Perform A* to produce path of waypoints to given dest from nearest map waypoint.

    Arguments:
    dest -- Destination Landmark
    wps  -- List of Landmarks (Landmark[]) representing full list of map waypoints.

    Returns:
        Landmark[] representing path to follow to the destination.
        Empty list if no path can be found.
    """
    near = find_nearest_navigable(wps)
    if near == None:
        rospy.logerr("AStar navigation failed, couldn't find a starting node.")
        return []
    rospy.logdebug("LandmarkClosestToMe: {}".format(near.name))
    preds = {near.name:None}
    pq = PriorityQueue()
    openSet = [near]
    visited = []
    #dict holding {waypoint name:distance from robot to waypoint} pairs
    gScores = {near.name:point_distance(my_pose.x,my_pose.y,near.x,near.y)}
    #pq elements are (g+h,node)
    # g=distRobotWp, h=distWpGoal
    pq.put((gScores[near.name]+point_distance(near.x,near.y,dest.x,dest.y),
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
            tentG = gScores[cnode.name]+point_distance(cnode.x,cnode.y,nbr.x,nbr.y)
            if(not(nbr in openSet)or(tentG<gScores[nbr.name])):
                preds[nbr.name]=cnode
                gScores[nbr.name]=tentG
                pq.put((gScores[nbr.name]+point_distance(nbr.x,nbr.y,dest.x,dest.y),nbr))
                if(not(nbr in openSet)):
                    openSet.append(nbr)
    #Cleanup persistent service connection.
    getNeighbors.close()
    return []

def main():
    rospy.init_node('robot_navigator', log_level=rospy.DEBUG)
    #Publisher to obstacle_avoidance
    pointPub = rospy.Publisher('waypoints',Point)
    rospy.Subscriber('goals',Landmark,goals_callback)
    rospy.Subscriber('pose',Pose,pose_callback)

    rospy.spin()

if __name__ == '__main__':
    main()