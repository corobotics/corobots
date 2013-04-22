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

class CorobotNavigator():

    def __init__(self, occupancy_map):
        # Robot's current position.
        self.pose = None
        self.occupancy_map = occupancy_map
        # Queue of (Point, isGoal?) pairs used to track goals from user.
        self.wp_queue = deque()
        self.ros_init()

    def ros_init(self):
        rospy.init_node('corobot_navigator')
        #Publisher to obstacle_avoidance
        self.point_pub = rospy.Publisher('waypoints', Point)
        self.goal_reached_pub = rospy.Publisher('goals_reached', Point)
        rospy.Subscriber('goals_nav', Point, self.goals_nav_callback)
        rospy.Subscriber('goals', Point, self.goals_callback)
        rospy.Subscriber('pose', Pose, self.pose_callback)

    def start(self):
        rospy.spin()

    def pose_callback(self, pose):
        """Pose subscription callback."""
        self.pose = pose

    def waypoints_reached_callback(self, wp):
        """Wayoints Reached subscription callback"""
        top = self.wp_queue[0]
        if (top[0].x == wp.x) and (top[0].y == wp.y):
            self.wp_queue.popleft()
            if top[1] == True:
                self.goal_reached_pub.publish(top[0])

    def goals_callback(self, new_goal):
        """No navigation goal queuing"""
        self.wp_queue.append((new_goal, True))
        self.point_pub.publish(new_goal)

    def goals_nav_callback(self, new_goal):
        """Goals subscription callback."""
        rospy.wait_for_service('get_landmarks')

        try:
            get_wps_srv = rospy.ServiceProxy('get_landmarks', GetLandmarks)
            #Gets waypoints, no neighbor data...maybe I should change that ~Karl
            # wps is a Landmark[]
            wps = get_wps_srv().all_wps
            end = new_goal
            # Will return a path of Landmarks from the Landmark
            # closest to the robot to the Landmark closest to the goal.
            path = self.a_star(end, wps)
            for node in path:
                self.point_pub.publish(x=node.x, y=node.y)
                self.wp_queue.append((Point(x=node.x, y=node.y), False))
            #And then finally publish the final waypoint
            if len(path) > 0:
                self.wp_queue.append((new_goal, True))
                self.point_pub.publish(new_goal)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: {}".format(e))

    def bresenham_callback(self, x, y):
        i = x + y * self.occupancy_map.info.width
        occ_prob = self.occupancy_map.data[i]
        if occ_prob > 50:
            return False

    def navigable(self, p1, p2):
        """Test whether there are any obstacles between p1 and p2."""
        res = self.occupancy_map.info.resolution
        x1, y1 = int(p1.x / res), int(p1.y / res)
        x2, y2 = int(p2.x / res), int(p2.y / res)
        v = bresenham(x1, y1, x2, y2, self.bresenham_callback)
        return v is not False

    def find_nearest_visibles(self, point, landmarks, num):
        """Finds landmarks visible from a point.

        point -- The starting point.
        landmarks -- Landmark[] with all landmarks in the map.
        num -- Return the closest <num> landmarks.

        Returns a list of up to n closest Landmarks visible from the given point.

        """
        closest = []
        for wp in landmarks:
            d = point_distance(point, wp)
            if (closest == [] or d < closest[0]) and self.navigable(point, wp):
                closest.append((d, wp))
        if closest == []:
            rospy.logerr("Cannot find a nearby waypoint to begin navigation!")
            return []
        closest.sort()
        return [landmark for d, landmark in closest[:num]]

    def a_star(self, dest, wps):
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
        near = self.find_nearest_visibles(my_pose, wps, zone_size)
        goal = Landmark(name="CORO_GOAL_",x=dest.x,y=dest.y)
        goal_zone = self.find_nearest_visibles(dest, wps, zone_size)
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
        try:
            get_nbrs_srv = rospy.ServiceProxy('get_neighbors', GetNeighbors, persistent=True)
            with closing(get_nbrs_srv):
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

                    # Bit of hackery to add the goal as a neighbor to all of
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
    rospy.wait_for_service('get_map')
    try:
        get_map_srv = rospy.ServiceProxy('get_map',GetCoMap)
        occupancy_map = get_map_srv().map
    except rospy.ServiceProxy as e:
        rospy.logerr("Service call failed: %s" % e)
    CorobotNavigator(occupancy_map).start()

if __name__ == '__main__':
    main()
