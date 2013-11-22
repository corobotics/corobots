#!/usr/bin/env python

import asyncore
from collections import deque
from contextlib import closing
import socket
import time
import threading
import thread

import roslib; roslib.load_manifest("corobot_manager")
import rospy
from geometry_msgs.msg import Point
from diagnostic_msgs.msg import *

from corobot_common import point_equals
from corobot_common.srv import GetLandmark
from corobot_common.msg import Pose, Landmark, UIMessage, UIConfirm
from corobot_manager.io import CorobotServer

# Akshay - Make a global status flag
SERVER_DELIMITER = ':'
HOST = "129.21.30.80"
PORT = 51000

class CorobotManager():

    def __init__(self):
        global HOST, PORT
        # Robot"s current position.  Defaults to a test position.
        self.pose = Pose(x=67.7648,y=14.9568,theta=0) # Close to EInter
        # Track goals.
        self.goal_queue = deque()
        # The asyncore.dispatcher object wrapping our server socket.
        self.server = None
        self.STATUS_FLAG = "IDLE"
        self.serverSocket = socket.socket (socket.AF_INET, socket.SOCK_STREAM)

    # Akshay - Function to communicate continuously with the Web Server
    def communicate (self, event):
        
        global SERVER_DELIMITER
        #while True:
        try:
            self.serverSocket.send (self.STATUS_FLAG + SERVER_DELIMITER + str(self.pose.x) + SERVER_DELIMITER + str(self.pose.y))
            #print self.goal_queue
            '''if(len(self.goal_queue) > 0):
                self.STATUS_FLAG = "BUSY"
            else:
                self.STATUS_FLAG = "IDLE"'''
        except socket.error, msg:
            print ("Server socket error! Error no: %d. Error message : %s" % (msg[0], msg[1]))
        #print 'Timer called at ' + str(event.current_real)
        #break
        #print ("Closing server socket connection.")
        #serverSocket.close()
        #print ("Server socket closed.")

        
    def start(self):
        self.init_ros_node()
        try:

            # Akshay - Spawn a new thread that communicates with Web Server.
            self.serverSocket.connect ((HOST,PORT))
            self.serverSocket.send ("corobot3" +SERVER_DELIMITER+ self.STATUS_FLAG)
            data = self.serverSocket.recv (1024)
            print data
            #thread.start_new_thread (self.communicate, (serverSocket,))
            rospy.Timer(rospy.Duration(1), self.communicate)
            # Akshay - End of new code
            self.listen_for_clients()
        except SystemExit:
            rospy.signal_shutdown("Requested by user.")

    def shutdown(self):
        if not rospy.is_shutdown():
            rospy.signal_shutdown("")
        if self.server is not None:
            self.server.handle_close()
            if self.server.handler is not None:
                self.server.handler.close_when_done()

    def client_write(self, msg_id, msg):
        """Utility function to write a message to the current client."""
        if self.server.handler:
            self.server.handler.write_line("%s %s" % (msg_id, msg))

    def pose_callback(self, pose):
        """Callback for the pose ROS topic."""
        self.pose = pose

    def goals_reached_callback(self, goal):
        """Callback for the goals_reached ROS topic."""
        if self.goal_queue and point_equals(goal, self.goal_queue[0][1]):
            msg_id, _ = self.goal_queue.popleft()
            self.client_write(msg_id, "ARRIVED")

    def goals_failed_callback(self, goal):
        """Callback for the goals_reached ROS topic."""
        if self.goal_queue and point_equals(goal, self.goal_queue[0][1]):
            msg_id, _ = self.goal_queue.popleft()
            self.client_write(msg_id, "ERROR Failed to reach goal.")

    def confirm_ui_callback(self, confirm):
        self.client_write(confirm.id, "CONFIRM %s" % confirm.confirmed)

    def diagnostics_callback(self, dArray):
        pass
    '''status[2] is the DiagnosticStatus related to Battery, 
        values[3] is the KeyValue related to Charge, values[4] is KeyValue related to Capacity'''
        #batteryLevel = float(dArray.status[2].values[3].value) / float(dArray.status[2].values[4].value) 
        #print("battery %: ", batteryLevel)

    def init_ros_node(self):
        """Initialize all ROS node/sub/pub/srv stuff."""
        rospy.init_node("corobot_manager")
        rospy.Subscriber("pose", Pose, self.pose_callback)
        rospy.Subscriber("goals_reached", Point, self.goals_reached_callback)
        rospy.Subscriber("goals_failed", Point, self.goals_failed_callback)
        rospy.Subscriber("confirm_msg", UIConfirm, self.confirm_ui_callback)
        rospy.Subscriber("diagnostics", DiagnosticArray, self.diagnostics_callback)
        rospy.wait_for_service("get_landmark")
        self.get_landmark = rospy.ServiceProxy("get_landmark", GetLandmark)
        self.goals_pub = rospy.Publisher("goals", Point)
        self.goals_nav_pub = rospy.Publisher("goals_nav", Point)
        self.show_msgs_pub = rospy.Publisher("show_msg", UIMessage)
        rospy.loginfo("Listening for client robots.")
        rospy.on_shutdown(self.shutdown)

    def listen_for_clients(self):
        self.server = CorobotServer(15001, self)
        asyncore.loop(0.1)

    def handle_command(self, command):
        rospy.loginfo("Command recieved from client: %s", command)
        tokens = command.strip().split(" ")
        if not tokens:
            self.server.handler.close_when_done()
            return
        msg_id = tokens[0]
        if len(tokens) < 2:
            self.client_write(msg_id, "ERROR Missing message type.")
            return
        msg_type = tokens[1]
        data = tokens[2:]

        #Command processing
        if msg_type == "GETPOS":
            self.client_write(msg_id, "POS %f %f %f" % (self.pose.x, self.pose.y, self.pose.theta))
        elif msg_type.startswith(("GOTO", "NAVTO")):
            if msg_type.endswith("LOC"):
                try:
                    landmark = self.get_landmark(data[0].upper())
                    x, y = landmark.wp.x, landmark.wp.y
                except rospy.ServiceException as e:
                    self.client_write(msg_id, "ERROR " % e)
            else:
                x, y = float(data[0]), float(data[1])
            if msg_type.startswith("NAVTO"):
                self.goals_nav_pub.publish(x=x, y=y)
            else:
                self.goals_pub.publish(x=x, y=y)
            self.goal_queue.append((msg_id, Point(x=x, y=y)))
        elif msg_type.startswith("SHOW_MSG"):
            confirm = msg_type.endswith("CONFIRM")
            themsg = " ".join(data[1:])
            self.show_msgs_pub.publish(UIMessage(id=int(msg_id), timeout=int(data[0]), 
                msg=themsg, req_confirm=confirm))
        else:
            self.client_write(msg_id, "ERROR Unknown message type \"%s\"" % msg_type)

def main():
    CorobotManager().start()

if __name__ == "__main__":
    main()
