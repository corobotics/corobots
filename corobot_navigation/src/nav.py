#!/usr/bin/env python
import math
from collections import deque
from Queue import PriorityQueue
from contextlib import closing

import roslib; roslib.load_manifest('corobot_navigation')
import rospy
from geometry_msgs.msg import Point

from corobot_common import a_star, bresenham, distance, point_distance
from corobot_common.srv import GetPixelOccupancy, GetNeighbors, GetLandmark, GetLandmarks, GetCoMap
from corobot_common.msg import Pose, Landmark

class CorobotNavigator():

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
        if waypoint.x == head.x and waypoint.y == head.y:
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
        if waypoint.x == head.x and waypoint.y == head.y:
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
        rospy.logdebug("A* result: %s" % (", ".join(l.name for l in path)))
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

    def find_nearest_visibles(self, point, num):
        """Finds landmarks visible from a point.

        point -- The starting point.
        num -- Return the closest <num> landmarks.

        Returns a list of up to n closest Landmarks visible from the given point.

        """
        closest = []
        for landmark, _ in self.landmark_graph.itervalues():
            d = point_distance(point, landmark)
            if (closest == [] or d < closest[0]) and self.navigable(point, landmark):
                closest.append((d, landmark))
        if closest == []:
            rospy.logerr("Cannot find a nearby waypoint to begin navigation!")
            return []
        closest.sort()
        return [landmark for d, landmark in closest[:num]]

    def navigate(self, dest):
        """Find a path from current location to the given destination."""
        ZONE_SIZE = 4
        # Make our node objects for A*.
        start = Landmark(name="START", x=self.pose.x, y=self.pose.y)
        goal = Landmark(name="GOAL", x=dest.x, y=dest.y)
        # Find neighbors of start and goal to add as edges to the graph.
        start_zone = self.find_nearest_visibles(start, ZONE_SIZE)
        goal_zone = self.find_nearest_visibles(goal, ZONE_SIZE)
        # Manually add the edge from start to goal if it's navigable.
        if self.navigable(start, goal):
            start_zone.append(goal)
        rospy.logdebug("Start zone: %s" % (", ".join(l.name for l in start_zone)))
        rospy.logdebug("Goal zone: %s" % (", ".join(l.name for l in goal_zone)))
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
