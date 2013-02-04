#!/usr/bin/env python
import roslib; roslib.load_manifest('corobot_comm')
import rospy
import socket

def clientComm(socket,addr):
    rospy.init_node('corobot_client_comm')
    clIn = socket.makefile('r')
    clOut = socket.makefile('w')
    while True:
        cmd = client.readline()
        if len(cmd) == 0:
            clIn.close()
            clOut.close()
            break
        rospy.loginfo("Command recieved from client %n: %s", addr, cmd)
        cmd = cmd.split(' ')
        if cmd[0] == 'GETPOS':
            clOut.write("POS {} {}".format(str(0),str(0)))
        if cmd[0] == 'GOTOXY':
            #Add dest point!
            


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