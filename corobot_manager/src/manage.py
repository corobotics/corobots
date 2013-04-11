#!/usr/bin/env python

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
from corobot_common.msg import Pose, Landmark

class CorobotManager():

    def __init__(self):
        # Robot"s current position.  Defaults to a test position.
        self.pose = Pose(x=67.7648,y=14.9568,theta=0) # Close to EInter
        # Track goals.
        self.goal_queue = deque()
        # The output stream to the current client, or None.
        self.client_out = None
        # A lock so we can write to the client from multiple threads.
        self.client_out_lock = threading.Lock()

    def start(self):
        self.init_ros_node()
        try:
            self.listen_for_clients()
        except SystemExit:
            rospy.signal_shutdown("Requested by user.")

    def client_write(self, msg_id, msg):
        """Utility function to write a message to the current client."""
        with self.client_out_lock:
            if self.client_out:
                self.client_out.write("%s %s\n" % (msg_id, msg))
                self.client_out.flush()

    def pose_callback(self, pose):
        """Callback for the pose ROS topic."""
        self.pose = pose

    def goals_reached_callback(self, reached):
        """Callback for the goals_reached ROS topic."""
        if self.goal_queue and point_equals(self.goal_queue[0][1], reached):
            msg_id, _ = self.goal_queue.popleft()
            self.client_write(msg_id, "ARRIVED")

    def init_ros_node(self):
        """Initialize all ROS node/sub/pub/srv stuff."""
        rospy.init_node("corobot_manager")
        rospy.Subscriber("pose", Pose, self.pose_callback)
        rospy.Subscriber("goals_reached", Point, self.goals_reached_callback)
        rospy.wait_for_service("get_landmark")
        self.get_landmark = rospy.ServiceProxy("get_landmark", GetLandmark)
        self.goals_pub = rospy.Publisher("goals", Point)
        self.goals_nav_pub = rospy.Publisher("goals_nav", Point)
        rospy.loginfo("Listening for client robots.")

    def listen_for_clients(self):
        # Create our server socket.
        server_socket = socket.socket()
        server_socket.bind((socket.gethostname(), 15001))
        server_socket.listen(1)
        while not rospy.is_shutdown():
            # Accept socket.
            try:
                client_socket, client_addr = server_socket.accept()
            except socket.error as e:
                # If a SystemExit is raised during socket.accept(), it gets
                # rethrown as a socket.error, but we want SystemExit.
                raise SystemExit(e)
            with closing(client_socket):
                # Set up the output output stream variable.
                with self.client_out_lock:
                    self.client_out = client_socket.makefile("w")
                # Only this function gets the input stream.
                with closing(client_socket.makefile("r")) as client_in:
                    try:
                        self.listen_for_commands(client_in, client_addr)
                    except socket.error as e:
                        rospy.logerr(e)
                # Clean up client_out.
                with self.client_out_lock:
                    try:
                        self.client_out.close()
                    except:
                        pass
                    self.client_out = None

    def listen_for_commands(self, client_in, client_addr):
        """Listen for API commands from the client.

        client_in -- A file object for reading from the client socket.
        client_addr -- The client's address.

        """
        rospy.loginfo("Ready for commands.")
        while not rospy.is_shutdown():
            command = client_in.readline()

            # Communication terminated?
            if len(command) == 0:
                break

            host, port = client_addr
            rospy.loginfo("Command recieved from client [%s:%s]: %s", host, port, command)
            tokens = command.strip().split(" ")
            msg_id = tokens[0]
            if len(tokens) < 2:
                self.client_write(msg_id, "ERROR Missing message type.")
                continue
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
            else:
                self.client_write(msg_id, "ERROR Unknown message type \"%s\"" % msg_type)

def main():
    CorobotManager().start()

if __name__ == "__main__":
    main()
