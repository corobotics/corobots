#!/usr/bin/env python
import roslib; roslib.load_manifest('corobot_comm')
import rospy
import socket
from geometry_msgs.msg import Point
from corobot_msgs.srv import GetLocation,GetWaypoints,GetPixelOccupancy
from corobot_msgs.msg import Pose
from queue import PriorityQueue

myPose = None
lastWP = None

def pose_callback(pose):
    global myPose
    myPose = pose

'''
Can I straight line nav to this wp from current pos?
'''
def navigableTo(wp):
    cx = myPose.x
    cy = myPose.y
    dx = wp.x-cx
    dy = wp.y-cy
    sdx = math.copysign(1,dx)
    sdy = math.copysign(1,dy)
    if(math.fabs(dx) > math.fabs(dy)):
        incx = sdx/2
        incy = dy/(dx/incx)
    else:
        incy = sdy/2
        incx = dx/(dy/incy)
    mapAt = rospy.ServiceProxy('get_pixel_occupancy',GetPixelOccupancy,persistent=True)
    while(sdx*dx > 0 or sdy*dy > 0):
        #Service request
        occ = mapAt(cx+dx,cy+dy).occupancy
        if(occ == 0):
            return False
        dx -= incx
        dy -= incy
    mapAt.close()
    return True

def pointDistance(wp1x,wp1y,wp2x,wp2y):
    return math.fabs((wp2y-wp1y)/(wp2x-wp1x))

'''
Find nearest Waypoint to the current value of myPose
'''
def findNearestNavigable(wps):
    closest = None
    for wp in wps:
        if closest == None and navigableTo(wp):
            closest = (waypointDistance(myPose,wp),wp)
            continue
        dist = waypointDistance(myPose,wp)
        if dist < closest[0] and navigableTo(wp):
            closest = (dist,wp)
    return closest[1]

'''
Perform A* to produce path of waypoints to given dest from nearest map waypoint.

Arguments:
dest -- Destination Waypoint
wps  -- List of Waypoints (Waypoint[]) representing full list of map waypoints.
'''
def aStar(dest,wps):
    near = findNearestNavigable(wps)
    preds = {near:None}
    pq = PriorityQueue()
    openSet = [near]
    visited = []
    gScores = {near:pointDistance(myPose.x,myPose.y,near.x,near.y)}
    #pq elements are (g+h,node)
    pq.put((gScores[near]+pointDistance(near.x,near.y,dest.x,dest.y),
        near))
    #Set up persistent connection to the GetNeighbors service
    getNeighbors = rospy.ServiceProxy('get_neighbors',GetNeighbors,persistent=True)
    #Build path from dest back to me.
    #Use distRobotWp+distWpGoal as priority weight
    while(not(pq.empty())):
        curr = pq.get()
        cnode = curr[1]
        if(cnode.name==dest.name):
            #Found the path! Now build it.
            path = []
            pnode = dest
            while(not(pnode==None)):
                path.insert(0,pnode)
                pnode = preds[pnode]
            return path

        openSet.remove(cnode)
        visited.append(cnode)
        for nbr in getNeighbors(cnode):
            if(nbr in visited):
                continue
            tentG = curr[1]+pointDistance(cnode.x,cnode.y,nbr.x,nbr.y)
            if(not(neighbor in openSet)or(tentG<gScores[nbr])):
                preds[nbr]=cnode
                gScores[nbr]=tentG
                pg.put((gScores[nbr]+pointDistance(nbr.x,nbr.y,dest.x,dest.y),nbr))
                if(not(nbr in openSet)):
                    openSet.append(nbr)
    getNeighbors.close()
    return None

def clientComm(socket,addr):
    rospy.init_node('corobot_client_comm')
    clIn = socket.makefile('r')
    clOut = socket.makefile('w')
    #Publisher to obstacle_avoidance
    pointPub = rospy.Publisher('waypoints',Point)
    rospy.Subscriber('pose',Pose,pose_callback)
    
    while True:
        cmd = client.readline()
        if len(cmd) == 0:
            clIn.close()
            clOut.close()
            break
        rospy.loginfo("Command recieved from client %n: %s", addr, cmd)
        cmd = cmd.split(' ')

        #Command processing
        if cmd[0] == 'GETPOS':
            if myPose is None:
                clOut.write("POS {} {} {}\n".format(str(0),str(0),str(0)))
                clOut.flush()
            else:
                clOut.write("POS {} {} {}\n".format(str(myPose.x),str(myPose.y),str(myPose.theta)))
                clOut.flush()
        elif cmd[0] == 'GOTOXY':
            #Add dest point!
            pointPub.publish(x=float(cmd[1]),y=float(cmd[2]))
        elif cmd[0] == 'GOTOLOC':
            #Goto location, no navigation
            rospy.wait_for_service('get_location')
            try:
                getLoc = rospy.ServiceProxy('get_location',GetLocation)
                #returns Waypoint
                resp = getLoc(cmd[1])
                pointPub.publish(x=resp.wp.x,y=resp.wp.y)
            except rospy.ServiceException as e:
                print("Service call failed: {}".format(e))
        elif cmd[0] == 'NAVTOLOC':
            #Navigate to location
            rospy.wait_for_service('get_waypoints')
            rospy.wait_for_service('get_location')
            try:
                getLoc = rospy.ServiceProxy('get_location',GetLocation)
                getWps = rospy.ServiceProxy('get_waypoints',GetWaypoints)
                #Gets waypoints, no neighbor data...maybe I should change that ~Karl
                # wps is a Waypoint[]
                wps = getWps()
                start = getLoc(cmd[1])
                path = aStar(start,wps)
                for node in path:
                    pointPub.publish(x=node.x,y=node.y)
            except rospy.ServiceException as e:
                print("Service call failed: {}".format(e))
        elif cmd[0] == 'QUERY_ARRIVE':
            print("Query_Arrive")
            #How to figure this out?!

def main():
    serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    serversocket.bind((socket.getHostname(),15001))
    serversocket.listen(1)

    while True:
        (client, clAddr) = serversocket.accept()
        #On connection accept, go into ROS node method
        clientComm(client,clAddr)

if __name__ == '__main__':
    main()