#!/usr/bin/env python
import math
from collections import deque
from contextlib import closing
from itertools import chain, imap
from Queue import PriorityQueue

import roslib; roslib.load_manifest('corobot_navigation')
import rospy
from geometry_msgs.msg import Point

from corobot_common import a_star, bresenham, distance, point_distance, point_equals
from corobot_common.srv import GetPixelOccupancy, GetNeighbors, GetLandmark, GetLandmarks, GetCoMap
from corobot_common.msg import Pose, Landmark

class CorobotNavigator():

    # Maximum distance in meters to try to go straight from the start to a node.
    MAX_ZONE_DIST = 18.0

    # The maximum number of nodes that can be in a zone.
    MAX_ZONE_SIZE = 4

    def __init__(self, occupancy_map, landmark_graph):
        # Robot's current position.
        self.pose = None
        self.occupancy_map = occupancy_map
        self.landmark_graph = landmark_graph
        # Queue of (Point, isGoal?) pairs used to track goals from user.
        self.wp_queue = deque()
        self.ros_init()

    def ros_init(self):
        rospy.init_node('corobot_navigator')
        #Publisher to obstacle_avoidance
        self.point_pub = rospy.Publisher('waypoints', Point)
        self.goals_reached_pub = rospy.Publisher('goals_reached', Point)
        self.goals_failed_pub = rospy.Publisher('goals_failed', Point)
        rospy.Subscriber('goals_nav', Point, self.goals_nav_callback)
        rospy.Subscriber('goals', Point, self.goals_callback)
        rospy.Subscriber('pose', Pose, self.pose_callback)
        rospy.Subscriber('waypoints_failed', Point, self.waypoints_failed_callback)
        rospy.Subscriber('waypoints_reached', Point, self.waypoints_reached_callback)

    def start(self):
        rospy.spin()

    def pose_callback(self, pose):
        """Pose subscription callback."""
        self.pose = pose

    def waypoints_reached_callback(self, waypoint):
        """Waypoints reached subscription callback."""
        if not self.wp_queue:
            rospy.logerr("Waypoint reached but queue is empty.")
            return
        head, is_goal = self.wp_queue[0]
        rospy.loginfo("Waypoint reached at (%f, %f), is_goal is %s", head.x, head.y, is_goal) 
        if point_equals(waypoint, head):
            self.wp_queue.popleft()
            if is_goal:
                self.goals_reached_pub.publish(head)
        else:
            rospy.logerr("Waypoint reached but doesn't match head of queue.")

    def waypoints_failed_callback(self, waypoint):
        if not self.wp_queue:
            rospy.logerr("Waypoint failed but queue is empty.")
            return
        head, is_goal = self.wp_queue[0]
        if point_equals(waypoint, head):
            self.wp_queue.popleft()
            if is_goal:
                self.goals_failed_pub.publish(head)
        else:
            rospy.logerr("Waypoint failed but doesn't match head of queue.")

    def goals_callback(self, new_goal):
        """No navigation goal queuing"""
        self.wp_queue.append((new_goal, True))
        self.point_pub.publish(new_goal)

    def goals_nav_callback(self, goal):
        """Goals subscription callback."""
        # Will return a path of Landmarks from the Landmark
        # closest to the robot to the Landmark closest to the goal.
        rospy.loginfo("New goal: (%.2f, %.2f)" % (goal.x, goal.y))
        path = self.navigate(goal)
        if not path:
            rospy.logerr("A* navigation failed!")
            return
        rospy.loginfo("A* result: %s" % (", ".join(l.name for l in path)))
        for node in path[:-1]:
            self.point_pub.publish(x=node.x, y=node.y)
            self.wp_queue.append((Point(x=node.x, y=node.y), False))
        # And then finally publish the final waypoint
        self.point_pub.publish(goal)
        self.wp_queue.append((goal, True))

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
        # v will be False if we found an obstacle, and None otherwise.
        return v is not False

    def find_nearest_visibles(self, point, landmarks):
        """Finds landmarks visible from a point.

        Limited to only nodes within MAX_ZONE_DIST of point, unless there
        are none in which case the single next closest node is returned.

        Arguments:
          point     - The starting point.
          landmarks - An iter of landmarks to consider.

        Returns a list of up to MAX_ZONE_SIZE closest visible Landmarks.

        """
        #rospy.loginfo("Calculating visibles for %s", point)
        #rospy.loginfo("Selecting from %s" % (", ".join(l.name for l in landmarks)))
        visibles = []
        for landmark in landmarks:
            if self.navigable(point, landmark):
                d = point_distance(point, landmark)
                visibles.append((d, landmark))
        visibles.sort()
        '''
        for d, landmark in visibles:
            rospy.loginfo("%s at %f", landmark.name, d)
            '''
        nearest = []
        for d, landmark in visibles[:CorobotNavigator.MAX_ZONE_SIZE]:
            # Limit to within MAX_ZONE_DIST, but take at least one.
            if d < CorobotNavigator.MAX_ZONE_DIST or not nearest:
                nearest.append(landmark)
        return nearest

    def navigate(self, dest):
        """Find a path from current location to the given destination."""
        # Make our node objects for A*.
        start = Landmark(name="START", x=self.pose.x, y=self.pose.y)
        goal = Landmark(name="GOAL", x=dest.x, y=dest.y)
        # Find visible neighbors of start and goal to add as edges to the graph.
        landmarks = imap(lambda pair: pair[0], self.landmark_graph.itervalues())
        # Here we manually add the goal to the candidate landmarks for start
        # to allow for navigation directly to the goal location.
        start_zone = self.find_nearest_visibles(start, chain(landmarks, [goal]))
        # We have to add start here for the empty zone check below.
        landmarks = imap(lambda pair: pair[0], self.landmark_graph.itervalues())
        goal_zone = self.find_nearest_visibles(goal, chain(landmarks, [start]))
        rospy.loginfo("Start zone: %s" % (", ".join(l.name for l in start_zone)))
        rospy.loginfo("Goal zone: %s" % (", ".join(l.name for l in goal_zone)))
        if not start_zone or not goal_zone:
            rospy.logerr("Cannot connect start and goal! A* failed.")
            return []
        # A* functions.
        is_goal = lambda node: node.name == "GOAL"
        heuristic = lambda node: point_distance(node, goal)
        def neighbors(node):
            if node.name == "START":
                return start_zone
            nbrs = self.landmark_graph[node.name][1]
            if node in goal_zone:
                # Intentionally use list concat to make a copy of the list.
                # If we just modify nbrs, it will modify the original graph.
                return nbrs + [goal]
            return nbrs
        return a_star(start, is_goal, neighbors, point_distance, heuristic)

def load_occupancy_map():
    rospy.wait_for_service('get_map')
    get_occupancy_map_srv = rospy.ServiceProxy('get_map', GetCoMap)
    return get_occupancy_map_srv().map

def load_landmark_graph():
    rospy.wait_for_service('get_landmarks')
    rospy.wait_for_service('get_neighbors')
    get_landmarks_srv = rospy.ServiceProxy('get_landmarks', GetLandmarks)
    get_neighbors_srv = rospy.ServiceProxy('get_neighbors', GetNeighbors)
    landmarks = get_landmarks_srv().all_wps
    graph = {}
    for landmark in landmarks:
        neighbors = get_neighbors_srv(landmark).neighbors
        graph[landmark.name] = (landmark, neighbors)
    return graph

def main():
    occupancy_map = load_occupancy_map()
    landmark_graph = load_landmark_graph()
    CorobotNavigator(occupancy_map, landmark_graph).start()

if __name__ == '__main__':
    main()
