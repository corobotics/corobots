#!/usr/bin/env python

import roslib; roslib.load_manifest("corobot_manager")
import rospy
import math

from diagnostic_msgs.msg import *
from corobot_common.msg import Pose
from corobot_common.msg import Goal
from corobot_manager.ui import CorobotMonitorUI

class CorobotMonitor():
    """ROS Node for data monitoring and interaction"""

    def __init__(self):
        self.win = CorobotMonitorUI()
        self.init_ros_node()
        self.win.mainloop()

    def pose_callback(self, pose_message):
        self.win.setPose(pose_message.x, pose_message.y, pose_message.theta)
        #self.win.after(100,rospy.spin_once())

    def rawnav_callback(self, rawnav_message):
        self.win.setRawnavMsg(rawnav_message.name)

    def obstacle_callback(self, obs_message):
        self.win.setObsMsg(obs_message.name)

    def absGoal_callback(self, absGoal_message):
        self.win.setAbsGoalMsg(absGoal_message.name)

    def netForce_callback(self, netForce_message):
        self.win.setNetForceMsg(netForce_message.name)

    def velcmd_callback(self, velcmd_message):
        self.win.setVelCmdMsg(velcmd_message.name)

    def qrCount_callback(self, qrCount_msg):
        self.win.setQrCountMsg(qrCount_msg.name)

    def recovery_callback(self, recovery_msg):
        self.win.setRecoveryMsg(recovery_msg.name)	
    
    def diagnostics_callback(self, dArray):

        #print len(dArray.status)
        #print len(dArray.status[2].values)

        '''for i in range(len(dArray.status[2].values)):
            print dArray.status[2].values[i].key'''
        if(len(dArray.status[2].values) >= 5):
            batteryLevel = float(dArray.status[2].values[3].value) / float(dArray.status[2].values[4].value)*100
            batteryLevel = math.ceil(batteryLevel * 100) / 100
        else:
            batteryLevel = "low"
        #print batteryLevel
        self.win.setBatteryMsg(str(batteryLevel))



    def init_ros_node(self):
        """Initialize all ROS node/sub/pub/srv stuff."""
        rospy.init_node("corobot_monitor")

        rospy.Subscriber("pose", Pose, self.pose_callback)
        rospy.Subscriber("ch_rawnav", Goal, self.rawnav_callback)
        rospy.Subscriber("ch_velcmd", Goal, self.velcmd_callback)
        rospy.Subscriber("ch_obstacle", Goal, self.obstacle_callback)
        rospy.Subscriber("ch_absgoal", Goal, self.absGoal_callback)
        rospy.Subscriber("ch_netforce", Goal, self.netForce_callback)
        rospy.Subscriber("ch_qrcodecount", Goal, self.qrCount_callback)
        rospy.Subscriber("ch_recovery", Goal, self.recovery_callback)
        rospy.Subscriber("diagnostics", DiagnosticArray, self.diagnostics_callback)


def main():
    cm = CorobotMonitor()

if __name__ == "__main__":
    main()
