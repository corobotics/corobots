#!/usr/bin/env python
import socket
import time
import threading
from collections import deque

import roslib; roslib.load_manifest("corobot_manager")
import rospy
from geometry_msgs.msg import Point

from corobot_common import point_equals
from corobot_common.srv import GetLandmark
from corobot_common.msg import Pose, Landmark

class CorobotManager():

    def __init__(self):
        # Robot"s current position.  Defaults to a test position.
        #self.pose = Pose(x=26.896, y=-9.7088, theta=0) # Class3435N
        self.pose = Pose(x=7.1832, y=-9.184, theta=0) # Close to EInter
        # Track goals.
        self.goal_queue = deque()
        # The output stream to the current client, or None.
        self.client_out = None
        # A lock so we can write to the client from multiple threads.
        self.client_out_lock = threading.Lock()

    def start(self):
        self.init_ros_node()
        self.listen_for_clients()

    def client_write(self, msg_id, msg):
        """Utility function to write a message to the current client."""
        with self.client_out_lock:
            if self.client_out:
                self.client_out.write("%d %s\n" % (msg_id, msg))
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
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind((socket.gethostname(), 15001))
        self.server_socket.listen(1)
        while not rospy.is_shutdown():
            # Accept socket.
            client_socket, client_addr = server_socket.accept()
            # Set up the output output stream variable.
            with self.client_out_lock:
                self.client_out = client_socket.makefile("w")
            # Only this function gets the input stream.
            with client_socket.makefile("r") as client_in:
                self.listen_for_commands(client_in, client_addr)
            # Clean up instance variables client_out, client_socket, and client_addr.
            with self.client_out_lock:
                self.client_out.close()
                self.client_out = None
                client_socket.close()

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

            rospy.loginfo("Command recieved from client %s: %s", client_addr, command)
            tokens = cmd.strip().split(" ")
            msg_id = tokens[0]
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
                        rospy.logerr("Service call failed: %s" % e)
                        self.client_write("ERROR %s" % e)
                else:
                    x, y = float(data[0]), float(data[1])
                if msg_type.startswith("NAVTO"):
                    self.goals_nav_pub.publish(x=x, y=y)
                else:
                    self.goals_pub.publish(x=x, y=y)
                self.goals_queue.append((msg_id, Point(x=x, y=y)))
            else:
                self.client_write(msg_id, "ERROR Unknown message type \"%s\"" % msg_type)

def main():
    CorobotManager().start()

if __name__ == "__main__":
    main()
