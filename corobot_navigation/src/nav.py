#!/usr/bin/env python
import math
from collections import deque
from Queue import PriorityQueue
from contextlib import closing

import roslib; roslib.load_manifest('corobot_navigation')
import rospy
from geometry_msgs.msg import Point

from corobot_common import bresenham, distance, point_distance
from corobot_common.srv import GetPixelOccupancy, GetNeighbors, GetLandmark, GetLandmarks, GetCoMap
from corobot_common.msg import Pose, Landmark

#Robot's current position.  Defaults to a test position.
my_pose = Pose(x=67.7648,y=14.9568,theta=0) # Close to EInter

occ_map = None

#Used to track set goals from a user.
# Queue of (Point, isGoal?) pairs
wp_queue = deque()

def pose_callback(pose):
    """Pose subscription callback"""
    global my_pose
    my_pose = pose

def waypoints_reached_callback(wp):
    """Wayoints Reached subscription callback"""
    top = wp_queue[0]
    if (top[0].x == wp.x) and (top[0].y == wp.y):
        wp_queue.popleft()
        if top[1] == True:
            goal_reached_pub = rospy.Publisher('goals_reached', Point)
            goal_reached_pub.publish(top[0])

def goals_callback(new_goal):
    """No navigation goal queuing"""
    global wp_queue
    wp_queue.append((new_goal, True))
    point_pub = rospy.Publisher('waypoints', Point)
    point_pub.publish(new_goal)

def goals_nav_callback(new_goal):
    """Goals subscription callback."""
    rospy.wait_for_service('get_landmarks')

    try:
        #Publisher to obstacle_avoidance
        point_pub = rospy.Publisher('waypoints', Point)
        get_wps_srv = rospy.ServiceProxy('get_landmarks', GetLandmarks)
        #Gets waypoints, no neighbor data...maybe I should change that ~Karl
        # wps is a Landmark[]
        wps = get_wps_srv().all_wps
        end = new_goal
        #Will return a path of Landmarks from the 
        #   Landmark closest to the robot to the Landmark closest to the goal.
        path = a_star(end, wps)
        for node in path:
            point_pub.publish(x=node.x, y=node.y)
            wp_queue.append((Point(x=node.x, y=node.y), False))
        #And then finally publish the final waypoint
        if len(path) > 0:
            wp_queue.append((new_goal, True))
            point_pub.publish(new_goal)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: {}".format(e))

def bresenham_callback(x, y):
    i = x + y * occ_map.info.width
    occ_prob = occ_map.data[i]
    if occ_prob > 50:
        return False

def navigable(p1, p2):
    """Test whether there are any obstacles between p1 and p2."""
    res = occ_map.info.resolution
    x1, y1 = int(p1.x / res), int(p1.y / res)
    x2, y2 = int(p2.x / res), int(p2.y / res)
    v = bresenham(x1, y1, x2, y2, bresenham_callback)
    return False if v is False else True

def find_nearest_visibles(point, landmarks, num):
    """Find nearest <num> visible landmarks

    Arguments:
    point -- The starting point
    landmarks -- Landmark[] with all landmarks in the graph/map
    num -- Return the closest <num> landmarks.

    Returns a Landmark[]:
        Nearest <num> navigable Landmark
        None if no nearby waypoint can be found

    """
    closest = []
    for wp in landmarks:
        d = point_distance(point, wp)
        if (closest == [] or d < closest[0]) and navigable(point, wp):
            closest.append((d, wp))
    if closest == []:
        rospy.logerr("Cannot find a nearby waypoint to begin navigation!")
        return []
    closest.sort()
    closest_n = [y for (x,y) in closest][0:num]
    return closest_n

def a_star(dest, wps):
    """Perform a modified A* to produce path of waypoints to given dest from nearest map waypoint.
    
    Modifications were made to consider a handful of starting locations instead of one absolute
    origin (the robot's position), this lets the algorithm find the best landmark for the robot
    to start its planned path.  Also, because the goal may not be a predefined landmark the 
    concept of a "goal zone" was added which adds a dynamic landmark for the goal as a neighbor
    to any of the landmarks in the goal zone.

    Arguments:
    dest -- Destination Point
    wps  -- List of Landmarks (Landmark[]) representing full list of map waypoints.

    Returns:
        Landmark[] representing path to follow to the destination.
        Empty list if no path can be found.

    """
    zone_size = 4
    near = find_nearest_visibles(my_pose, wps, zone_size)
    goal = Landmark(name="CORO_GOAL_",x=dest.x,y=dest.y)
    goal_zone = find_nearest_visibles(dest, wps, zone_size)
    if near is []:
        rospy.logerr("A* navigation failed, no landmarks visible from robot.")
        return []
    if goal_zone == []:
        rospy.logerr("A* navigation failed, no landmarks visible to goal.")

    #preds used to build path when a path is found.
    preds = {}
    
    #pq elements are (g+h, node)
    # g=distRobotWp, h=distWpGoal
    # These are 'f_scores' of the estimated best path cost through the node
    pq = PriorityQueue()
 
    # Set of nodes to be potentially evaluated, 
    #  initialized with our set of potential starting nodes
    open_set = []

    # Set of nodes already evaluated
    visited = []

    #dict holding {waypoint name: distance from robot to waypoint} pairs
    # This is the cost from the node along the best known path
    g_scores = {}

    rospy.logdebug("Near nodes: " + str(near))
    rospy.logdebug("Goal Zone: " + str(goal_zone))

    #Prime the queue and other data structures with the potential starting nodes.
    for node in near:
        g = point_distance(my_pose, node)
        g_scores[node.name] = g

        pq.put((g + point_distance(node, goal), node))
        open_set.append(node.name)
        preds[node.name] = None
        rospy.logdebug("Initialized %s" % (node.name))

    #Set up persistent connection to the GetNeighbors service
    rospy.wait_for_service('get_neighbors')
    with closing(rospy.ServiceProxy('get_neighbors', GetNeighbors, persistent=True)) as get_nbrs_srv:
        while not pq.empty():
            curr = pq.get()
            cnode = curr[1]
            rospy.logdebug("Processing node: " + cnode.name)
            if cnode.name == "CORO_GOAL_":
                #Found the path! Now build it.
                path = []
                pnode = goal
                while pnode is not None:
                    pname = pnode.name
                    path.insert(0, pnode)
                    pnode = preds[pname]
                rospy.logdebug("Path: " + str(path))
                return path

            open_set.remove(cnode.name)
            visited.append(cnode.name)

            #Bit of hackery to add the goal as a neighbor to all of
            # the waypoints in the goal_zone, so that we aren't forced
            # to overshoot the goal and then backtrack
            nbrs = get_nbrs_srv(cnode).neighbors
            if cnode in goal_zone:
                nbrs.append(goal)

            for nbr in nbrs:
                tentG = g_scores[cnode.name] + point_distance(cnode, nbr)
                if nbr.name in visited:
                    if tentG >= g_scores[nbr.name]:
                        continue
                if nbr.name not in open_set or tentG < g_scores[nbr.name]:
                    preds[nbr.name] = cnode
                    g_scores[nbr.name] = tentG
                    pq.put((g_scores[nbr.name] + point_distance(nbr, goal), nbr))
                    open_set.append(nbr.name)
    except rospy.ServiceProxy as e:
        rospy.logerr("Service call failed: %s" % e)
    return []

def main():
    rospy.init_node('robot_navigator')
    #Publisher to obstacle_avoidance
    point_pub = rospy.Publisher('waypoints', Point)
    rospy.Subscriber('goals_nav', Point, goals_nav_callback)
    rospy.Subscriber('goals', Point, goals_callback)
    rospy.Subscriber('pose', Pose, pose_callback)

    global occ_map
    rospy.wait_for_service('get_map')
    try:
        get_map_srv = rospy.ServiceProxy('get_map',GetCoMap)
        occ_map = get_map_srv().map
    except rospy.ServiceProxy as e:
        rospy.logerr("Service call failed: {}".format(e))

    rospy.spin()

if __name__ == '__main__':
    main()
