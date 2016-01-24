#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('corobot_obstacle_avoidance')
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from math import sqrt, atan2, sin, cos, exp, degrees
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from numpy import linalg, cross, dot, isnan
from geometry_msgs.msg import PointStamped
from corobot_common.msg import Pose, Goal
from corobot_common.srv import GetLandmark, GetLandmarks

# Tunable APF parameters
D_GOAL = 0.25 # The distance(m) at which to switch from conical to constant goal attraction
D_OBS = 1.2#1.2 #1.2 # The influence distance(m) of obstacles
K_GOAL = 0.35 # Attractive force coefficient for goal potential fields
K_OBS = 0.5 # Repulsive force coefficient for obstacle potential fields
KMAX = 15 # Maximum magnitude of perpendicular-to-repulsive forces
TAU = 15 # Tau in 2 * KMAX / (1 - e^(delta/tau))

# Other constants and parameters
ARRIVED_INTERSECTION_DISTANCE = .75 # The threshold distance(m) that a waypoint is considered reached
ARRIVED_HALLWAY_DISTANCE = 2
ARRIVED_GOAL_DISTANCE = 0.2 # The threshold distance(m) that destination is considered reached
M_PI = 3.141592 # pi
MIN_OMEGA = 0.2 # Minimum rotational velocity
MAX_OMEGA = 0.5
ANGLE_WINDOW = 0.15 # Force angle beyond which we will turn instead of go straight 
MAX_VEL = 0.30 # Max allowed forward velocity 
OBS_CACHE_TIMEOUT = 10 # Num seconds after which obstacles leave the cache (unless seen again) 
OBS_MATCH_DIST = 0.5 # Threshold distance for matching an obstacle in the cache

class Polar: #commands
    __slots__ = ['d', 'a']

    def __init__(self, distance, angle):
        self.d = distance
        self.a = angle

    def __str__(self):
        return '(d={},a={})'.format(self.d, self.a)

    def valid(self):
        return self.d > 0

class CachedPoint:
    __slots__ = ['x', 'y', 'time']

    def __init__(self, x, y, time):
        self.x = x
        self.y = y
        self.time = time

    def __str__(self):
        return '(x={}, y={}, time={})'.format(self.x, self.y, self.time)

def bound(n, b, r):
    if n > b + r:
        return b + r
    elif n < b - r:
        return b - r
    else:
        return n

def rCoordTransform(aPoint,  bOrigin):
    bPoint = Point(0, 0, 0)
    dx = aPoint.x - bOrigin.x
    dy = aPoint.y - bOrigin.y
    theta = -bOrigin.theta
    bPoint.x = dx * cos(theta) - dy * sin(theta)
    bPoint.y = dx * sin(theta) + dy * cos(theta)
    return bPoint

def euclideanDistance(a, b):
    return sqrt((b.y - a.y)**2 + (b.x - a.x)**2)

class obstacle_avoidance:
    __slots__ = ['cachedObstacles',
                    'arrivedWaypoints'
                    'failedWaypoints',
                    'goalPoint',
                    'publishers',
                    'kinectMaximumAngle',
                    'kinectMinimumAngle',
                    'pose',
                    'previousCommand',
                    'previousRobotPose',
                    'previousWaypointCommand',
                    'previousWaypointQueueLength'
                    'recovering',
                    'timeLastMoved',
                    'timeLastScanned',
                    'timeSinceLastWaypoint',
                    'self.waypointQueue']

    def __init__(self):
        rospy.init_node('obstacle_avoidance', log_level = rospy.DEBUG , anonymous = True)
        self.publishers = {}
        self.failedQueue = []
        self.cachedObstacles = []
        self.arrivedWaypoints = []
        self.waypointQueue = []
        rospy.wait_for_service('get_landmarks')
        self.publishers['debug'] = rospy.Publisher('debug', String)
        get_landmarks_srv = rospy.ServiceProxy('get_landmarks', GetLandmarks)
        landmarks = get_landmarks_srv().all_wps
        self.landmarkTypes = {}
        for landmark in landmarks:
            self.landmarkTypes[str(landmark.x) + ',' + str(landmark.y)] = landmark.ltype
        self.publishers['rawnav'] = rospy.Publisher('ch_rawnav', Goal)
        self.publishers['obstacle'] = rospy.Publisher('ch_obstacle', Goal)
        self.publishers['absGoal'] = rospy.Publisher('ch_absgoal', Goal)
        self.publishers['netForce'] = rospy.Publisher('ch_netforce', Goal)
        self.publishers['perpForce'] = rospy.Publisher('ch_perpforce', Goal)
        self.publishers['repForce'] = rospy.Publisher('ch_repforce', Goal)
        self.publishers['velCmd'] = rospy.Publisher('ch_velcmd', Goal)
        self.publishers['recovery'] = rospy.Publisher('ch_recovery', Goal)
        self.publishers['commandVelocity'] = rospy.Publisher('cmd_vel', Twist)
        self.publishers['waypointsReached'] = rospy.Publisher('waypoints_reached', Point)
        self.publishers['waypointsFailed'] = rospy.Publisher('waypoints_failed', Point)
        self.publishers['goalsNavigation'] = rospy.Publisher('goals_nav', Point)
        rospy.Subscriber('scan', LaserScan, self.scanCallback)
        rospy.Subscriber('pose', Pose, self.poseCallback)
        rospy.Subscriber('waypoints', Point, self.waypointCallback)
        rospy.Subscriber('ch_qrcodecount', Goal, self.stopRecoveryCallback) 
        rospy.Subscriber('goals', Point, self.goalCallback)
        rospy.Subscriber('goals_nav', Point, self.goalCallback)
        self.distanceTraveled = 0.0
        self.timeDeployed = -1
        self.goal = Point(0, 0, 0)
        self.pose = Pose()
        self.pose.x = 0
        self.pose.y = 0
        self.pose.theta = 0
        self.previousCommand = Polar(0, 0)
        self.previousRobotPose = Pose()
        self.previousRobotPose.x = self.pose.x
        self.previousRobotPose.y = self.pose.y
        self.timeSinceLastWaypoint = 0
        self.timeLastScanned = 0
        self.timeLastMoved = 0
        self.angleMin = 0
        self.angleMax = 0
        self.previousWaypointQueueLength = 0
        self.recovering = False
        self.i = 0

    def scanCallback(self, scan):
        twistMessage = Twist()
        point = self.navigate(scan)
        twistMessage.linear.x = point.d
        twistMessage.angular.z = point.a
        self.publishers['commandVelocity'].publish(twistMessage)
        while len(self.failedQueue) > 0:
            failed = self.failedQueue.pop(0)
            self.publishers['waypointsFailed'].publish(failed)
            rospy.loginfo('Waypoint failed: ({:.2f}, {:.2f})'.format(failed.x, failed.y))

    def stopRecoveryCallback(self, topicMessage):
        if self.recovering:
            self.recovering = False
            self.previousWaypointQueueLength = 0
            rospy.sleep(rospy.Duration(0.5))
            self.publishers['goalsNavigation'].publish(self.goal)

    def waypointCallback(self, waypoint):
        self.waypointQueue.append(waypoint)
        rospy.loginfo('Waypoint added: ({:.2f}, {:.2f})'.format(waypoint.x, waypoint.y))

    def goalCallback(self, waypoint):
        self.goal = waypoint
        self.waypointQueue = []

    # Pose -> None
    def poseCallback(self, pose):
        self.pose = pose
        if len(self.waypointQueue) > 0:
            currentWaypoint = self.waypointQueue[0]
            while (len(self.waypointQueue) > 0 and self.waypointReached(currentWaypoint)):
                rospy.loginfo('Arrived at waypoint ({:.2f}, {:.2f})'
                        .format(currentWaypoint.x, currentWaypoint.y))
                self.waypointQueue.pop(0)
                self.arrivedWaypoints.append(currentWaypoint)
                if len(self.waypointQueue) > 0:
                    currentWaypoint = self.waypointQueue[0]
        while len(self.arrivedWaypoints) > 0:
            self.i = 0
            reached = self.arrivedWaypoints.pop(0)
            self.publishers['waypointsReached'].publish(reached)
            rospy.loginfo('Waypoint reached: ({:.2f}, {:.2f})'.format(reached.x, reached.y))

    # Waypoint -> Boolean
    def waypointReached(self, waypoint):
        waypointType = self.landmarkTypes[str(waypoint.x) + ',' + str(waypoint.y)]
        self.publishers['debug'].publish(waypointType+' -> ' + str(self.i))
        self.i += 1
        if waypointType == 'NSH':
            nextWaypoint = self.waypointQueue[1] # assumes NSH is not a destination
            nextWaypointSide = waypoint.x - nextWaypoint.x
            poseSide = waypoint.x - self.pose.x
            if (nextWaypointSide > 0 and poseSide > 0) or (nextWaypointSide < 0 and poseSide < 0):
                return True
            else:
                return abs(self.pose.x - waypoint.x) < ARRIVED_HALLWAY_DISTANCE
        elif waypointType == 'WEH':
            nextWaypoint = self.waypointQueue[1] # assumes WEH is not a destination
            nextWaypointSide = waypoint.y - nextWaypoint.y
            poseSide = waypoint.x - self.pose.x
            if (nextWaypointSide > 0 and poseSide > 0) or (nextWaypointSide < 0 and poseSide < 0):
                return True
            else:
                return abs(self.pose.x - waypoint.x) < ARRIVED_HALLWAY_DISTANCE
        elif waypointType == 'I':
            return euclideanDistance(self.pose, waypoint) < ARRIVED_INTERSECTION_DISTANCE
        else:
            return euclideanDistance(self.pose, waypoint) < ARRIVED_GOAL_DISTANCE

    @staticmethod
    def scanToList(scan):
        rospy.loginfo('scanToList: {}'.format(scan.angle_increment))
        points = []
        angle = scan.angle_min
        for distance in scan.ranges:
            point = Polar(0, angle)
            if scan.range_min < distance < scan.range_max:
                point.d = distance
            else:
                point.d = -1
            points.append(point)
            angle += scan.angle_increment
        return points

    # [Polar] -> [Polar]
    @staticmethod
    def findLocalMinima(points):
        rospy.loginfo('findLocalMinima:')
        localMinima = []
        lastWasCloser = False
        prev = Polar(-1, 0)
        for point in points:
            if point.d >= 0 and (prev.d < 0 or point.d <= prev.d):
                lastWasCloser = True
            else:
                if lastWasCloser:
                    localMinima.append(prev)
                lastWasCloser = False
            prev = point
        if lastWasCloser:
            localMinima.append(prev)
        i = 0
        for local in localMinima:
            print 'minima{} = ({}, {})'.format(i, local.d, local.a)
            i = i + 1
        return localMinima

    # [Polar] -> [Polar]
    @staticmethod
    def findObjects(points):
        rospy.loginfo('findObjects:')
        objects = []
        if len(points) > 0:
            objMin = points[0]
            last = points[0]
            last.d = -1.0
        for point in points[1:]:
            if point.d >= 0: # true iff point is valid
                if last.d >= 0: # true iff there is an object in progress
                    if abs(point.d - last.d) < 0.2: # true iff point belongs to the current object
                        if point.d < objMin.d:
                            #if this point is closer than object min, save it
                            objMin = point
                    else: 
                        # make a new object and save the previous one
                        objects.append(objMin)
                        objMin = point
                else:
                    objMin = point # no object in progress so start a new one
            elif last.d >= 0:
                # else if there was an object in progress, add it to the list
                objects.append(objMin)
            last = point
        if last.d >= 0:
            objects.append(objMin)
        i = 0
        for ob in objects:
            print 'object{}=({}, {})'.format(i, ob.d, ob.a)
            i = i + 1
        rospy.loginfo('objects = {}'.format(len(objects)))
        return objects

    # LaserScan -> Polar
    def navigate(self, scan):
        if len(self.waypointQueue) == 0:
            rospy.loginfo('No waypoints in queue!')
            return Polar(0,0)
        if self.timeDeployed == -1:
            rospy.loginfo('deploying at time={}'.format(rospy.get_time()))
            self.timeDeployed = rospy.get_time()
        else:
            rospy.loginfo('current time={}'.format(rospy.get_time()))
        self.timeLastScanned = scan.header.stamp.secs
        self.angleMin = scan.angle_min
        self.angleMax = scan.angle_max
        delta = sqrt((self.previousRobotPose.x - self.pose.x)**2 + (self.previousRobotPose.y - self.pose.y)**2) 
        if delta < .2:
            self.distanceTraveled += sqrt((self.previousRobotPose.x - self.pose.x)**2 + (self.previousRobotPose.y - self.pose.y)**2) 
        if self.recovering:
            rospy.loginfo('[DEBUG] about to recover')
            return self.recoveryNavigate(scan)
        rospy.loginfo('[DEBUG] Not recovering')
        if abs(self.previousRobotPose.x - self.pose.x) > 1.0 or \
                        abs(self.previousRobotPose.y - self.pose.y) > 1.0 or \
                        abs(self.previousRobotPose.theta - self.pose.theta) > 1.0:
            self.cachedObstacles = []
            self.publishers['obstacle'].publish(name = 'obs cleared: {}'.format(len(self.cachedObstacles)))
            rospy.loginfo('[DEBUG] clearing ObstacleList: {}'.format(len(self.cachedObstacles)))
        rospy.loginfo('[DEBUG] Pose:\t({:.2f}, {:.2f}), <{:.2f}>'.format(self.pose.x, self.pose.y, self.pose.theta))
        rospy.loginfo('[DEBUG] Traveled Distance: {:.2f}'.format(self.distanceTraveled))
        rospy.loginfo('[DEBUG] Travel Time: {:.2f}'.format(rospy.get_time() - self.timeDeployed))
        goalInMap = self.waypointQueue[0]
        rospy.loginfo('[DEBUG] ABS Goal:\t({:.2f}, {:.2f})'.format(goalInMap.x, goalInMap.y))
        self.publishers['absGoal'].publish(name = '({}, {})'.format(goalInMap.x, goalInMap.y))
        goalWRTrobot = rCoordTransform(goalInMap, self.pose)
        rospy.loginfo('[DEBUG] Rel Goal: \t ({}, {})'.format(goalWRTrobot.x, goalWRTrobot.y))
        objects = self.findLocalMinima(self.findObjects(self.scanToList(scan)))
        rospy.loginfo('passing {} objects'.format(len(objects)))
        self.updateObstacleList(objects)
        [netForce, netForceOld] = self.calculateNetForce(goalWRTrobot, self.timeLastScanned)
        cmdInitial = Polar(euclideanDistance(netForce, Point(0, 0, 0)), 0)
        cmdInitialOld = Polar(euclideanDistance(netForceOld, Point(0, 0, 0)), 0)
        if abs(netForce.y) <= 0.099 and abs(netForce.x) <= 0.099:
            rospy.loginfo('[DEBUG] zero net force detected')
        else:
            cmdInitial.a = atan2(netForce.y, netForce.x)
        if abs(netForceOld.y) <= 0.099 and abs(netForceOld.x) <= 0.099:
            rospy.loginfo('[DEBUG] old zero net force detected')
        else:
            cmdInitialOld.a = atan2(netForceOld.y, netForceOld.x)
        rospy.loginfo('[DEBUG] TotalF:\t<{:+.2f},{:.2f}>'.format(cmdInitial.d, cmdInitial.a))
        self.publishers['rawnav'].publish(name = '<{}, {}>'.format(cmdInitial.d, cmdInitial.a))
        self.publishers['rawnav'].publish(name = '<{}, {}> old'.format(cmdInitialOld.d, cmdInitialOld.a))
        cmd = self.commandTransform(cmdInitial)
        now = self.timeLastScanned
        if cmd.d > 0 or self.timeLastMoved == 0:
            self.timeLastMoved = now
        elif now - self.timeLastMoved > 10:
            self.waypointQueue.pop(0)
            self.failedQueue.append(goalInMap)
            self.timeLastMoved = 0.0
        if len(self.waypointQueue) > 0:
            self.recoveryCheck(scan.header.stamp.secs)
        if cmd.d < 0:
            cmd.d = 0
        self.previousCommand = cmd
        self.previousRobotPose.x = self.pose.x
        self.previousRobotPose.y = self.pose.y
        self.previousRobotPose.theta = self.pose.theta
        self.publishers['repForce'].publish(name = 'clear')
        self.publishers['perpForce'].publish(name = 'clear')

        return cmd

    # Polar -> Pose
    def convertRobotCoorToGlobalCoor(self, polarPoint):
        sp = Pose()
        sp.x = self.pose.x + polarPoint.d * cos(self.pose.theta + polarPoint.a)
        sp.y = self.pose.y + polarPoint.d * sin(self.pose.theta + polarPoint.a)
        sp.theta = 0
        return sp

    # Pose -> Boolean
    def pushIfUnique(self, simplePose):
        for cachedPoint in self.cachedObstacles:
            if euclideanDistance(cachedPoint, simplePose) <= OBS_MATCH_DIST:
                rospy.loginfo('[DEBUG] APF, PushUnique returns False')
                cachedPoint.time = self.timeLastScanned
                return False
        cp = CachedPoint(simplePose.x, simplePose.y, self.timeLastScanned)
        self.cachedObstacles.append(cp)
        self.publishers['obstacle'].publish(name = '({}, {}), add : {}'.format(simplePose.x, simplePose.y, len(self.cachedObstacles)))
        rospy.loginfo('[DEBUG] APF, current list Size: {}'.format(len(self.cachedObstacles)))
        return True

    # Pose -> Double
    def distanceFromRobot(self, simplePose):
        return sqrt((simplePose.x-self.pose.x)**2 + (simplePose.y-self.pose.y)**2)

    # Point -> Polar
    def convertGlobalCoorToRobotCoor(self, point):
        p = Polar(0,0)
        if abs(point.y - self.pose.y) <= 0.099 and abs(point.x - self.pose.x) <= 0.099:
            rospy.loginfo('[DEBUG] APF, Returning GToR: zeros detected')
        else:
            p.a = atan2(point.y - self.pose.y, point.x - self.pose.x) - self.pose.theta
            p.d = self.distanceFromRobot(point)
        rospy.loginfo('[DEBUG] CONVERTING(globalCoor=({},{}), polarCoor=({},{})'.format(point.x, point.y, p.a, p.d))
        return p

    # Polar -> Polar
    def commandTransform(self, initialCommand):
        command = Polar(0,0)
        if initialCommand.a > ANGLE_WINDOW: #Dont try to go fwd if the angle is more then fixed value
            command.a = MAX_OMEGA
            if initialCommand.a < M_PI/2.0:
                command.d = cos(initialCommand.a)*initialCommand.d
                command.a = MIN_OMEGA + (MAX_OMEGA - MIN_OMEGA)*(initialCommand.a-ANGLE_WINDOW)/(M_PI/2-ANGLE_WINDOW)
        elif initialCommand.a < -ANGLE_WINDOW:
            command.a = -MAX_OMEGA
            if initialCommand.a > - M_PI/2.0:
                command.d = initialCommand.d * cos(initialCommand.a)
                command.a = -MIN_OMEGA - (MAX_OMEGA - MIN_OMEGA)*(ANGLE_WINDOW-initialCommand.a)/(M_PI/2+ANGLE_WINDOW)
        else:
            command.d = initialCommand.d
        if command.d > MAX_VEL:
            command.d = MAX_VEL
        command.d = bound(command.d, self.previousCommand.d, 0.050)
        self.publishers['velCmd'].publish(name = '<{}, {}>'.format(command.d, command.a))
        rospy.loginfo('[DEBUG] NavVel:\t<{:+.2f},{:2f}>\n'.format(command.d, command.a))
        return command


    # Point, time -> Point
    def calculateNetForce(self, goalWRTrobot, time):
        netForce = Point(0, 0, 0)
        netForceOld = Point(0, 0, 0)
        attForce = Point(0, 0, 0)
        ################
        # Attractive F #
        ################
        goalDistance = euclideanDistance(goalWRTrobot, Point(0, 0, 0))
        if goalDistance <= D_GOAL:
            attForce.x = (K_GOAL / D_GOAL) * goalWRTrobot.x
            attForce.y = (K_GOAL / D_GOAL) * goalWRTrobot.y
        else:
            attForce.x = K_GOAL * goalWRTrobot.x / goalDistance
            attForce.y = K_GOAL * goalWRTrobot.y / goalDistance
        netForce.x = attForce.x
        netForce.y = attForce.y
        netForceOld.x = attForce.x
        netForceOld.y = attForce.y
        rospy.loginfo('[DEBUG] GoalF:\t{:.2f}, {:.2f},'.format(attForce.x, attForce.y))
        objIndex = 0
        for cachedPoint in self.cachedObstacles:
            repForce = Point(0, 0, 0)
            #if time - cachedPoint.time < .2: continue
            ################
            # Repulsive  F #
            ################
            od = self.convertGlobalCoorToRobotCoor(cachedPoint)
            if od.d != 0:
                fmag = K_OBS * (1.0/D_OBS - 1.0/od.d) / (od.d**2)
            else:
                fmag = 0
            repForce.x = fmag * cos(od.a)
            repForce.y = fmag * sin(od.a)
            netForceOld.x += repForce.x
            netForceOld.y += repForce.y
            netForce.x += repForce.x
            netForce.y += repForce.y
            rospy.loginfo('[DEBUG] Obj({})repF:\t{:.2f}, {:.2f}'.format(objIndex, repForce.x, repForce.y))
            ###################
            # Perpendicular F #
            ###################
            theta = degrees(atan2(goalWRTrobot.y, goalWRTrobot.x))
            delta = abs(self.pose.theta - od.a) * 180/M_PI % 180
            repF = [[repForce.x], [repForce.y]]
            M_RS = [[0, -1], [1, 0]]
            M_LS = [[0, 1], [-1, 0]]
            d = degrees(od.a) % 360
            deltadeg = min(d, 360-d)
            #check left or right side
            A = [1, self.pose.x, self.pose.y]
            B = [1, self.pose.x + cos(od.a), self.pose.y + sin(od.a)]
            C = [1, goalWRTrobot.x, goalWRTrobot.y]
            isRight = (linalg.det([A,B,C]) > 0)
            K = (2*KMAX) / (1+exp(delta/TAU))
            if isRight:
                perpF = dot(dot(K, M_RS), repF)
            else:
                perpF = dot(dot(K, M_LS), repF)
            # perpF angle and repF and perpF magnitudes
            dist1 = sqrt(repF[0][0]**2 + repF[1][0]**2)
            dist2 = sqrt(perpF[0][0]**2 + perpF[1][0]**2)
            ang = atan2(perpF[1][0], perpF[0][0])
            rospy.loginfo('[DEBUG] Obj({})PerpF:\t{:.2f}, {:.2f} Theta:\t{:.2f}'.format(objIndex, perpF[0][0], perpF[1][0], theta))
            self.publishers['perpForce'].publish(name = '({}, {})'.format(dist1, ang))
            self.publishers['repForce'].publish(name = '({}, {})'.format(dist1, atan2(repF[1][0], repF[0][0])))
            netForce.x += dist2 * cos(ang)
            netForce.y += dist2 * sin(ang)
            objIndex = objIndex + 1
        self.publishers['netForce'].publish(name = '({}, {})'.format(netForce.x, netForce.y))
        return [netForce, netForce]

    # Point -> Double
    def polarDistance(self, polarPoint):
        x = polarPoint.d*cos(polarPoint.a)
        y = polarPoint.d*sin(polarPoint.a)
        return sqrt(x**2 + y**2)
        
    # [Polar] -> None
    def updateObstacleList(self, objects):
        objIndex = 0
        cachedObstacles = []
        for obj in self.cachedObstacles:
            objDistance = euclideanDistance(obj, self.pose)
            objAbsoluteAngle = atan2(obj.y - self.pose.y, obj.x - self.pose.x)
            objRelativeAngle = self.pose.theta - objAbsoluteAngle
            while objRelativeAngle > 2*M_PI:
                objRelativeAngle -= 2*M_PI
            while objRelativeAngle <= 0:
                objRelativeAngle += 2*M_PI
            rospy.loginfo('[DEBUG] APF, Obj({}) Distance:\t{:.2f} Angle:\t{:.3f}'.format(++objIndex, objDistance, objRelativeAngle))
            if ((objDistance > D_OBS) or
                (self.timeLastScanned - obj.time > OBS_CACHE_TIMEOUT) or
                (M_PI/2 < objRelativeAngle < 3*M_PI/2) or
                (self.angleMin <= objRelativeAngle <= self.angleMax)):
                rospy.loginfo('[DEBUG] APF: Object({}) removed'.format(objIndex))
                self.publishers['obstacle'].publish(name = '({}, {}), rem : {}'.format(obj.x, obj.y, objIndex))
            else:
                cachedObstacles.append(obj)
        self.cachedObstacles = cachedObstacles
        for pointWRTrobot in objects:
            if pointWRTrobot.d <= D_OBS:
                pointWRTglobal = self.convertRobotCoorToGlobalCoor(pointWRTrobot)
                rospy.loginfo('[DEBUG] Polar Object that might be added:\t({:.2f}, {:.2f})'.format(pointWRTrobot.d, pointWRTrobot.a))
                rospy.loginfo('[DEBUG] Object in global coor:\t({:.2f}, {:.2f})'.format(pointWRTglobal.x,pointWRTglobal.y))
                self.pushIfUnique(pointWRTglobal)
            else:
                rospy.loginfo('[DEBUG] {} is not <= {}'.format(pointWRTrobot.d, D_OBS))
        rospy.loginfo('cachedObstacles = {}'.format(len(self.cachedObstacles)))
        i = 0
        for obs in self.cachedObstacles:
            print 'obstacle{}={}'.format(i, obs)
            i = i + 1

    # Double -> None
    def recoveryCheck(self, recov_time_now):
        if len(self.waypointQueue) - self.previousWaypointQueueLength == 0:
            if recov_time_now - self.timeSinceLastWaypoint > 60:
                self.recoverRobot()
        elif self.goal.x != 0 and self.goal.y != 0 and -.1 < self.pose.x < .1 and  -.1 < self.pose.y < .1:
            self.recoverRobot()
        else:
            self.timeSinceLastWaypoint = recov_time_now
            self.previousWaypointQueueLength = len(self.waypointQueue)
            self.publishers['recovery'].publish(name = 'Not in Recovery')

    # None -> None
    def recoverRobot(self):
        self.cachedObstacles = []
        self.publishers['obstacle'].publish('obs cleared: {}'.format(len(self.cachedObstacles)))
        rospy.loginfo('[DEBUG] clearing ObstacleList: {}'.format(len(self.cachedObstacles)))
        rospy.loginfo('[DEBUG] Recovery protocol triggered')
        self.publishers['recovery'].publish(name = 'Recovery Started')
        self.recovering = True

    # LaserScan -> Polar
    def recoveryNavigate(self, scan):
        command = Polar(0.1, 0)
        objects = self.findLocalMinima(self.findObjects(self.scanToList(scan)))
        for obj in objects:
            if obj.d <= 1 and abs(obj.a) <= .6:
                command.a = 0.4
                command.d = bound(0, self.previousCommand.d, 0.05)
                self.previousCommand = command
                return command
        command.d = bound(command.d, self.previousCommand.d, 0.01)
        self.previousCommand = command
        return command

if __name__ == '__main__':
    obstacle_avoidance()
    rospy.spin()
