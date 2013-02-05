#!/usr/bin/env python
import roslib; roslib.load_manifest('corobot_comm')
import rospy
import socket
from geometry_msgs.msg import Point
from corobot_msgs.srv import GetLocation

def clientComm(socket,addr):
    rospy.init_node('corobot_client_comm')
    clIn = socket.makefile('r')
    clOut = socket.makefile('w')
    #Publisher for obstacle_avoidance
    pointPub = rospy.Publisher('waypoints',Point)
    
    while True:
        cmd = client.readline()
        if len(cmd) == 0:
            clIn.close()
            clOut.close()
            break
        rospy.loginfo("Command recieved from client %n: %s", addr, cmd)
        cmd = cmd.split(' ')
        if cmd[0] == 'GETPOS':
            clOut.write("POS {} {}\n".format(str(0),str(0)))
        elif cmd[0] == 'GOTOXY':
            #Add dest point!
            pointPub.publish(x=float(cmd[1]),y=float(cmd[2]))
        elif cmd[0] == 'GOTOLOC':
            #Goto location, no navigation
            rospy.wait_for_service('get_location')
            try:
                getLoc = rospy.ServiceProxy('get_location', GetLocation)
                resp = getLoc(cmd[1])
                pointPub.publish(x=resp.x,y=resp.y)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
        elif cmd[0] == 'NAVTOLOC':
            #Navigate to location
        elif cmd[0] == 'QUERY_ARRIVE':
            #How to figure this out?!
            


def main():
    serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    serversocket.bind( (socket.getHostname(),15001) )
    serversocket.listen( 1 )

    while True:
        (client, clAddr) = serversocket.accept()
        #On connection accept, go into ROS node method
        clientComm(client,clAddr)

if __name__ == '__main__':
    main()