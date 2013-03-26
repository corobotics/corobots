#!/usr/bin/env python
import socket
import time
import threading
from collections import deque

import roslib; roslib.load_manifest('corobot_comm')
import rospy
from geometry_msgs.msg import Point

from corobot_msgs.srv import GetLandmark
from corobot_msgs.msg import Pose,Landmark

#Robot's current position.  Defaults to a test position.
my_pose = Pose(x=26.896,y=-9.7088,theta=0) # Class3435N

goal_queue = deque()
cl_socket = None
sock_write_lock = threading.Lock()

def pose_callback(pose):
    """Pose subscription callback"""
    global my_pose
    my_pose = pose

def goals_reached_callback(reached):
    """Goals Reached subscription callback"""
    global goal_queue
    if ((len(goal_queue) > 0) and 
            (goal_queue[0].x == reached.x) and
            (goal_queue[0].y == reached.y)):
        goal_queue.popleft()
        with sock_write_lock:
            cl_out = cl_socket.makefile('w')
            cl_out.write("ARRIVED\n")
            cl_out.flush()


def client_comm(addr, goals_pub, goals_nav_pub):
    """Begin client API communication

    Arguments:
    socket -- Active socket to a connected client
    addr -- Client's IP address
    """

    cl_in = cl_socket.makefile('r')
    cl_out = cl_socket.makefile('w')

    global goal_queue
    
    while True:
        rospy.loginfo("Ready for commands.")
        cmd = cl_in.readline()
        #Communication terminated?
        if len(cmd) == 0:
            cl_in.close()
            with sock_write_lock:
                cl_out.close()
            break

        cmd = cmd.strip().split(' ')
        rospy.loginfo("Command recieved from client %s: %s", addr, cmd)

        #Command processing
        if cmd[0] == 'GETPOS':
            with sock_write_lock:
                cl_out.write("POS {} {} {}\n".format(str(my_pose.x),str(my_pose.y),str(my_pose.theta)))
                cl_out.flush()
        elif cmd[0] == 'GOTOXY':
            #Add dest point!
            goals_pub.publish(x=float(cmd[1]),y=float(cmd[2]))
            goal_queue.append(Point(x=float(cmd[1]), y=float(cmd[2])))
        elif cmd[0] == 'GOTOLOC':
            #Goto location, no navigation
            rospy.wait_for_service('get_landmark')
            dest = cmd[1].upper()
            try:
                get_lmark = rospy.ServiceProxy('get_landmark',GetLandmark)
                #returns Landmark
                resp = get_lmark(dest)
                goals_pub.publish(x=resp.wp.x,y=resp.wp.y)
                goal_queue.append(Point(x=resp.wp.x, y=resp.wp.y))
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: {}".format(e))
        elif cmd[0] == 'NAVTOLOC':
            rospy.wait_for_service('get_landmark')
            dest = cmd[1].upper()
            try:
                get_lmark = rospy.ServiceProxy('get_landmark',GetLandmark)
                resp = get_lmark(dest)
                goals_nav_pub.publish(x=resp.wp.x,y=resp.wp.y)
                goal_queue.append(Point(x=resp.wp.x, y=resp.wp.y))
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: {}".format(e))

def main():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((socket.gethostname(), 15001))
    server_socket.listen(1)

    rospy.loginfo("Listening for client robots.")
    rospy.init_node('corobot_client_comm')
    rospy.Subscriber('pose', Pose,pose_callback)

    #Publishers to robot_nav
    goals_pub = rospy.Publisher('goals', Point)
    goals_nav_pub = rospy.Publisher('goals_nav', Point)
    rospy.Subscriber('goals_reached', Point, goals_reached_callback)

    while True:
        (client, clAddr) = server_socket.accept()
        global cl_socket
        cl_socket = client
        #On connection accept, go into ROS node method
        client_comm(clAddr, goals_pub, goals_nav_pub)

if __name__ == '__main__':
    main()