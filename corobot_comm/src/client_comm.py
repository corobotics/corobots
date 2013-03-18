#!/usr/bin/env python
import roslib; roslib.load_manifest('corobot_comm')
import rospy
import socket
import time
import thread

from geometry_msgs.msg import Point
from corobot_msgs.srv import GetLandmark
from corobot_msgs.msg import Pose,Waypoint
from collections import deque

#Robot's current position.  Defaults to a test position.
myPose = Pose(x=26.3712,y=-7.7408,theta=0) # NE Atrium

goal_queue = deque()

def pose_callback(pose):
    """Pose subscription callback"""
    global myPose
    myPose = pose

def goals_reached_callback(reached):
    """Goals Reached subscription callback"""
    if goal_queue[0] == reached.name:
        goal_queue.popleft()

def client_comm(socket,addr,point_pub,goal_pub):
    """Begin client API communication

    Arguments:
    socket -- Active socket to a connected client
    addr -- Client's IP address
    """

    cl_in = socket.makefile('r')
    cl_out = socket.makefile('w')
    
    while True:
        rospy.loginfo("Ready for commands.")
        cmd = cl_in.readline()
        #Communication terminated?
        if len(cmd) == 0:
            cl_in.close()
            cl_out.close()
            break

        cmd = cmd.strip().split(' ')
        rospy.loginfo("Command recieved from client %s: %s", addr, cmd)

        #Command processing
        if cmd[0] == 'GETPOS':
            cl_out.write("POS {} {} {}\n".format(str(myPose.x),str(myPose.y),str(myPose.theta)))
            cl_out.flush()
        elif cmd[0] == 'GOTOXY':
            #Add dest point!
            point_pub.publish(x=float(cmd[1]),y=float(cmd[2]))
        elif cmd[0] == 'GOTOLOC':
            #Goto location, no navigation
            rospy.wait_for_service('get_landmark')
            dest = cmd[1].upper()
            try:
                get_lmark = rospy.ServiceProxy('get_landmark',GetLandmark)
                #returns Waypoint
                resp = get_lmark(dest)
                point_pub.publish(x=resp.wp.x,y=resp.wp.y)
                goal_queue.append(resp.wp.name)
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: {}".format(e))
        elif cmd[0] == 'NAVTOLOC':
            rospy.wait_for_service('get_landmark')
            dest = cmd[1].upper()
            try:
                get_lmark = rospy.ServiceProxy('get_landmark',GetLandmark)
                resp = get_lmark(dest)
                goal_pub.publish(resp.wp)
                goal_queue.append(resp.wp.name)
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: {}".format(e))
        elif cmd[0] == 'QUERY_ARRIVE':
            dest = cmd[1].upper()
            if dest in goal_queue:
                while dest in goal_queue:
                    time.sleep(1)
                cl_out.write("ARRIVED {}\n".format(cmd[1]))


def main():
    serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    serversocket.bind((socket.gethostname(),15001))
    serversocket.listen(1)

    rospy.loginfo("Listening for client robots.")
    rospy.init_node('corobot_client_comm')
    rospy.Subscriber('pose',Pose,pose_callback)

    #Publisher to obstacle_avoidance, for GOTO* commands
    point_pub = rospy.Publisher('waypoints',Point)
    #Publisher to robot_nav, for NAVTO* commands
    goal_pub = rospy.Publisher('goals',Waypoint)
    rospy.Subscriber('goals_reached',Waypoint,goals_reached_callback)

    while True:
        (client, clAddr) = serversocket.accept()
        #On connection accept, go into ROS node method
        client_comm(client,clAddr,point_pub,goal_pub)

if __name__ == '__main__':
    main()