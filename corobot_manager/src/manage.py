#!/usr/bin/env python

import asyncore
from collections import deque
from contextlib import closing
import socket
import time
import threading

import roslib; roslib.load_manifest("corobot_manager")
import rospy
from geometry_msgs.msg import Point

from corobot_common import point_equals
from corobot_common.srv import GetLandmark
from corobot_common.msg import Pose, Landmark, UIMessage, UIConfirm
from corobot_manager.io import CorobotServer

class CorobotManager():

    def __init__(self):
        # Robot"s current position.  Defaults to a test position.
        self.pose = Pose(x=67.7648,y=14.9568,theta=0) # Close to EInter
        # Track goals.
        self.goal_queue = deque()
        # The asyncore.dispatcher object wrapping our server socket.
        self.server = None

    def start(self):
        self.init_ros_node()
        try:
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

    def goals_reached_callback(self, reached):
        """Callback for the goals_reached ROS topic."""
        if self.goal_queue and point_equals(self.goal_queue[0][1], reached):
            msg_id, _ = self.goal_queue.popleft()
            self.client_write(msg_id, "ARRIVED")

    def confirm_ui_callback(self, confirm):
        self.client_write(confirm.id, "CONFIRM %s" % confirm.confirmed)

    def init_ros_node(self):
        """Initialize all ROS node/sub/pub/srv stuff."""
        rospy.init_node("corobot_manager")
        rospy.Subscriber("pose", Pose, self.pose_callback)
        rospy.Subscriber("goals_reached", Point, self.goals_reached_callback)
        rospy.Subscriber("confirm_msg", UIConfirm, self.confirm_ui_callback)
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
            self.show_msgs_pub.publish(UIMessage(id=msg_id, timeout=int(data[0]), 
                                        msg=data[1], req_confirm=confirm))
        else:
            self.client_write(msg_id, "ERROR Unknown message type \"%s\"" % msg_type)

def main():
    CorobotManager().start()

if __name__ == "__main__":
    main()
