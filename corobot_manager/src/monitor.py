#!/usr/bin/env python

import roslib; roslib.load_manifest("corobot_manager")
import rospy
import math
import subprocess
import threading
import time

from sensor_msgs.msg import LaserScan
from diagnostic_msgs.msg import *
from corobot_common.msg import Pose
from corobot_common.msg import Goal
from corobot_manager.ui import CorobotMonitorUI
from std_msgs.msg import String

class batteryThread(threading.Thread):
    def __init__(self, delay, win):
        threading.Thread.__init__(self)
        self.delay = delay
        self.win = win

    def run(self):
        self.laptop_battery(self.delay, self.win)

    def laptop_battery(self, delay, win):
        pub = rospy.Publisher('laptopBatman', String)
        while not rospy.is_shutdown():
            p = subprocess.Popen(["acpi", "-V"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            out, err = p.communicate()
            #print out
            batteryPercentage = out.split('\n')[0].split(', ')[1]
            #self.win.setLaptopBatteryMsg(out.split('\n')[0].split(', ')[1])
            self.win.setLaptopBatteryMsg(batteryPercentage)
            #rospy.loginfo(batteryPercentage)
            pub.publish(String(batteryPercentage))
            time.sleep(delay)


class CorobotMonitor():
    """ROS Node for data monitoring and interaction"""

    def __init__(self):
        self.win = CorobotMonitorUI()
        self.init_ros_node()
        self.win.mainloop()

    def pose_callback(self, pose_message):
        self.win.setPose(pose_message.x, pose_message.y, pose_message.theta, pose_message.cov)
        #self.win.after(100,rospy.spin_once())

    def laserpose_callback(self,pose_message):
        if pose_message.x < 0 and pose_message.y < 0:
            self.win.lasercolor = 'red'
        else:
            self.win.lasercolor = 'green'

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
    
    def laserDisp_callback(self, laserDat):
	self.win.setLaserMap(laserDat.angle_min, laserDat.range_min, laserDat.range_max, laserDat.angle_increment, laserDat.ranges)

    def diagnostics_callback(self, dArray):

        #print len(dArray.status)
        #print len(dArray.status[2].values)
        pub = rospy.Publisher("batman", String);
        '''for i in range(len(dArray.status[2].values)):
            print dArray.status[2].values[i].key'''
        if(len(dArray.status) >= 3):
            batteryLevel = float(dArray.status[2].values[3].value) / float(dArray.status[2].values[4].value)*100
            batteryLevel = math.ceil(batteryLevel * 100) / 100
        else:
            batteryLevel = "low"
        #print batteryLevel
        pub.publish(String(str(batteryLevel)))
        self.win.setBatteryMsg(str(batteryLevel))

    def laptop_battery(self):
        while True:
            p = subprocess.Popen(["acpi", "-V"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            out, err = p.communicate()
            print out.split('\n')[0].split(', ')[1]


    def init_ros_node(self):
        """Initialize all ROS node/sub/pub/srv stuff."""
        rospy.init_node("corobot_monitor")

        rospy.Subscriber("pose", Pose, self.pose_callback)
        rospy.Subscriber("laser_pose", Pose, self.laserpose_callback)
        rospy.Subscriber("ch_rawnav", Goal, self.rawnav_callback)
        rospy.Subscriber("ch_velcmd", Goal, self.velcmd_callback)
        rospy.Subscriber("ch_obstacle", Goal, self.obstacle_callback)
        rospy.Subscriber("ch_absgoal", Goal, self.absGoal_callback)
        rospy.Subscriber("ch_netforce", Goal, self.netForce_callback)
        rospy.Subscriber("ch_qrcodecount", Goal, self.qrCount_callback)
        rospy.Subscriber("ch_recovery", Goal, self.recovery_callback)
        rospy.Subscriber("diagnostics", DiagnosticArray, self.diagnostics_callback)
        rospy.Subscriber("scan", LaserScan, self.laserDisp_callback, queue_size = 1)
        t = batteryThread(0.5, self.win)
        t.daemon = True
        t.start()


def main():
    cm = CorobotMonitor()

if __name__ == "__main__":
    main()
