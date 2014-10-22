#include <cmath>
#include <iostream>

#include "ros/console.h"
#include "corobot_common/Pose.h"
#include "corobot.h"
#include "apf.h"

using corobot::bound;
using corobot::length;
using corobot::rCoordTransform;
using corobot_common::Pose;
using sensor_msgs::LaserScan;

bool Polar::isValid() const {
    return d >= 0.0;
}

/**
 * {@inheritDoc}
 */
list<Polar> APF::scanToList(LaserScan scan) {
    list<Polar> points;
    // Create a placeholder with the initial angle.
    Polar p;
    p.a = scan.angle_min;
    // Convert scan.ranges into a list.
    for (unsigned int i = 0; i < scan.ranges.size() - 1; i++) {
        float d = scan.ranges[i];
        if (d > scan.range_min && d < scan.range_max) {
            p.d = d;
        } else {
            // New convention: < 0 means invalid.
            p.d = -1.0;
        }
        points.push_back(p);
        p.a += scan.angle_increment;
    }
    return points;
}

/**
 * {@inheritDoc}
 */
list<Polar> APF::findLocalMinima(list<Polar> points) {
    // List of local minima that have been found.
    list<Polar> localMinima;
    // Whether the last angle's distance was smaller than the current one.
    bool lastWasCloser = false;
    // The previous point; init to an invalid point.
    Polar prev = {-1.0, 0.0};
    for (list<Polar>::iterator i = points.begin(); i != points.end(); i++) {
        Polar p = *i;
        // If p is a valid point and closer than the previous one.
        if (p.isValid() && (!prev.isValid() || p.d < prev.d)) {
            // We mark it as a potential candidate for being a local minima.
            lastWasCloser = true;
        } else {
            // Otherwise if i-1 was closer than i-2, i-1 is a local minima.
            if (lastWasCloser) {
                localMinima.push_back(prev);
            }
            lastWasCloser = false;;
        }
        prev = p;
    }
    // Check in case the last point was a minima.
    if (lastWasCloser) {
        localMinima.push_back(prev);
    }
    return localMinima;
}

/**
 * {@inheritDoc}
 */
list<Polar> APF::findObjects(list<Polar> points) {
    // List of "object" points: local minima of the scan.
    list<Polar> objects;
    // Point of the current object's minimum, for setting and adding to objects.
    Polar objMin;
    Polar last;
    last.d = -1.0;
    for (list<Polar>::iterator i = points.begin(); i != points.end(); i++) {
        Polar p = *i;
        if (p.d >= 0) {
            // if this is a valid point
            if (last.d >= 0) {
                // and there is an obj in progress
                if (abs(p.d - last.d) < 0.2) {
                    // check if this point is still in the object
                    if (p.d < objMin.d) {
                        // if this point is closer than objMin, save it.
                        objMin = p;
                    }
                } else {
                    // not in an object; add the previous object to the list and
                    // make a new one.
                    objects.push_back(objMin);
                    objMin = p;
                }
            } else {
                // no object in progress; start a new one.
                objMin = p;
            }
        } else if (last.d >= 0) {
            // else if there was an object in progress, add it to the list.
            objects.push_back(objMin);
        }
        last = p;
    }
    if (last.d >= 0) {
        objects.push_back(objMin);
    }
    return objects;
}

/**
 * {@inheritDoc}
 * Navigates using an artificial potential field technique with cached
 * obstacles to assist with the narrow field of view of the Kinect.
 * Order of operations:
 * 1. Compute goal location relative to robot
 * 2. Compute goal force: calcGoalForce
 * 3. Turn Kinect scan into list of discrete obstacles: findObjects
 * 4. Updates obstacle cache - delete old/far-away obstacles, 
 *     add new obstacles: updateObstacleList
 * 5. Add force due to obstacles: updateNetForce
 * 6. Convert (x,y) force vector to polar (relative to robot heading)
 * 7. Convert force vector to command velocity: cmdTransform
 * 8. Check for timeout to reach current waypoint
 * 9. Check for recovery (is robot lost) - not sure this check is
 *      being done properly at present
 */
Polar APF::nav(LaserScan scan) {
    // Can't do anything without a goal.
    if (waypointQueue.empty()) {
        ROS_INFO("No waypoints in queue!");
        Polar p; p.a = 0; p.d = 0;
        return p;
    }

    //ROS_WARN("WayPoint curr length: %d", waypointQueue.size());
    lastScanTime = scan.header.stamp.toSec();
    angleMin = scan.angle_min;
    angleMax = scan.angle_max;

    if(inRecovery){
        return doRecoveryNav(scan);
    }

    stringstream ss; corobot_common::Goal topicMsg;

    //clear obstacleList for if the robot sees a barcode and there is a difference in odometer and the qrcode's localization
    if(abs(prevRobotPose.x - pose.x) > 1.0 || abs(prevRobotPose.y - pose.y) > 1.0 || abs(prevRobotPose.a - pose.theta) > 1.0){
        activeObstacleList.clear();

        ss.str(""); ss << "obs cleared: " << activeObstacleList.size();
        topicMsg.name = ss.str(); obsPublisher.publish(topicMsg);
        ROS_DEBUG("clearing ObstacleList: %u", activeObstacleList.size());
    }

    ROS_DEBUG("Pose:\t(%.2f, %.2f), <%.2f>", pose.x, pose.y, pose.theta);

    // The goal is the head of the waypoint queue.
    Point goalInMap = waypointQueue.front();
    ROS_DEBUG("Abs Goal:\t(%.2f, %.2f)", goalInMap.x, goalInMap.y);
    ss.str(""); ss << "(" << goalInMap.x << ", " << goalInMap.y << ")";
    topicMsg.name = ss.str(); absGoalPublisher.publish(topicMsg);

    // Convert the goal into the robot reference frame.
    Point goalWrtRobot = rCoordTransform(goalInMap, pose);
    ROS_DEBUG("Rel Goal:\t(%.2f, %.2f)", goalWrtRobot.x, goalWrtRobot.y);

    // Stores the object force vector summation. z is ignored.
    Point netForce = calcGoalForce(goalWrtRobot);

    // The list of "objects" found; already in the robot reference frame.
    list<Polar> objects = findLocalMinima(findObjects(scanToList(scan)));

    updateObstacleList(objects);
    updateNetForce(netForce);

    // Resulting command vector.
    Polar cmdInitial;
    cmdInitial.d = length(netForce.x, netForce.y);
    if(abs(netForce.y) <= 0.099 && abs(netForce.x) <= 0.099) {
        ROS_DEBUG("zero net force detected");
        cmdInitial.a = 0;
    }else {
        cmdInitial.a = atan2(netForce.y, netForce.x);
    }
    
    ROS_DEBUG("TotalF:\t<%+.2f, %.2f>", cmdInitial.d, cmdInitial.a);
    // publishing raw navigation command to the ch_rawnav topic
    ss.str(""); ss << "<" << cmdInitial.d << ", " << cmdInitial.a << ">";
    topicMsg.name = ss.str(); rawnavPublisher.publish(topicMsg);

    Polar cmd = cmdTransform(cmdInitial);

    double now = lastScanTime;
    if (cmd.d > 0.0 || timeLastMoved == 0.0) {
        timeLastMoved = now;
    } else if (now - timeLastMoved > 10.0) {
        // Haven't moved in too long; give up on this waypoint.
        waypointQueue.pop();
        failedQueue.push(goalInMap);
        // Reset the timestamp so we don't give up on subsequent waypoints.
        timeLastMoved = 0.0;
    }

    if(waypointQueue.size() != 0)
        recoveryCheck(scan.header.stamp.toSec());
 
    //capture things for the next cycle
    cmdPrev = cmd;
    prevRobotPose.x = pose.x; prevRobotPose.y = pose.y; prevRobotPose.a = pose.theta; 

    return cmd;
}

/**
 * Convert the given polar (relative to robot) coordinate to the
 * global coordinate system.
 */
corobot::SimplePose* APF::convertRobotToGlobal(Polar &polarPoint){
    corobot::SimplePose *sp = new corobot::SimplePose();
    sp->x = pose.x + polarPoint.d * cos(pose.theta + polarPoint.a);
    sp->y = pose.y + polarPoint.d * sin(pose.theta + polarPoint.a);
    sp->a = 0;
    //ROS_DEBUG("APF, Returning RToG Coordinates");
    return sp;
}

/**
 * Add the given point to the obstacle cache if it is farther than
 * OBS_MATCH_DIST away from all other cached obstacles.
 */
bool APF::pushIfUnique(corobot::SimplePose *sp){
    for (std::vector<CachedPoint>::iterator it = activeObstacleList.begin() ; it != activeObstacleList.end(); ++it){
        Point obsPt = it->p;
        if(length(sp->x - obsPt.x,sp->y - obsPt.y) <= OBS_MATCH_DIST) { //if the point approx matches the points in the list
            ROS_DEBUG("APF, PushUnique returns False");
            it->lastT = lastScanTime;
            return false;
        }
    }
    
    //ROS_DEBUG("APF, Pushing Object into activeObstacleList: %.2f, %.2f", sp->x, sp->y);
    CachedPoint newObs;
    newObs.p.x = sp->x;
    newObs.p.y = sp->y;
    newObs.lastT = lastScanTime;
    activeObstacleList.push_back(newObs);

    //publishing the obstacle's coordinates to the ch_obstacle topic
    stringstream ss; corobot_common::Goal topicMsg;
    ss << "(" << sp->x << ", " << sp->y << "), add : " << activeObstacleList.size();
    topicMsg.name = ss.str(); obsPublisher.publish(topicMsg);

    ROS_DEBUG("APF, current list Size: %u", activeObstacleList.size());
    return true;
}

double APF::distanceFromRobot(corobot::SimplePose &sp){
    return sqrt((sp.x-pose.x)*(sp.x-pose.x) + (sp.y-pose.y)*(sp.y-pose.y));
}

/**
 * Convert the given point in the global coordinates to the robot's
 * relative coordinate system.
 */
Polar* APF::convertFromGlobalToRobotInPolar(Point &sp){
    Polar *p = new Polar();
    if(abs(sp.y-pose.y) <= 0.099 && abs(sp.x - pose.x) <= 0.099){
        p->a = 0;
        p->d = 0;
        ROS_DEBUG("APF, Returning GToR: zeros detected");
    }
    else{ 
        p->a = atan2(sp.y-pose.y, sp.x - pose.x) - pose.theta;
        p->d = sqrt((sp.x-pose.x)*(sp.x-pose.x) + (sp.y-pose.y)*(sp.y-pose.y));
    }
    //ROS_DEBUG("APF, Returning GToR coordinates");
    return p;
}

double APF::min(double a, double b){
    if(a<=b) 
        return a;
    return b;
}

/**
 * Convert a force vector into fwd/angular velocity
 * cmd.d is forward vel command, cmd.a is angular vel command
 */
Polar APF::cmdTransform(Polar &cmdInitial){
    Polar cmd;
    // Don't try to go forward if the angle is more than fixed value.
    if (cmdInitial.a > ANGLE_WINDOW) {
        cmd.a = MIN_OMEGA;
        if (cmdInitial.a < M_PI/2.0) {
            cmd.d = cos(cmdInitial.a)*cmdInitial.d;
        } else
            cmd.d = 0;
    } else if (cmdInitial.a < -ANGLE_WINDOW) {
        cmd.a = -MIN_OMEGA;
        if (cmdInitial.a > -M_PI/2.0) {
            cmd.d = cos(cmdInitial.a)*cmdInitial.d;
        } else
            cmd.d = 0;
    } else {
        // if force is near straight, just head straight for now.
        cmd.a = 0;
        // forward velocity equal to force (for now, then gets capped below)
        cmd.d = cmdInitial.d;
    }

    if(cmd.d > MAX_VEL)
        cmd.d = MAX_VEL;
    cmd.d = bound(cmd.d, cmdPrev.d, 0.010);

    stringstream ss; corobot_common::Goal topicMsg;
    ss << "<" << cmd.d << ", " << cmd.a << ">";
    topicMsg.name = ss.str(); velCmdPublisher.publish(topicMsg);
    ROS_DEBUG("NavVel:\t<%+.2f, %.2f>\n", cmd.d, cmd.a);
    
    return cmd;
}

/**
 * Compute the goal force for the given relative goal location.
 */
Point APF::calcGoalForce(Point &goalWrtRobot){
    Point netForce;
    double goalDist = length(goalWrtRobot.x, goalWrtRobot.y);
    if (goalDist <= D_GOAL) {
        netForce.x = (K_GOAL / D_GOAL) * goalWrtRobot.x;
        netForce.y = (K_GOAL / D_GOAL) * goalWrtRobot.y;
    } else {
        netForce.x = K_GOAL * goalWrtRobot.x / goalDist;
        netForce.y = K_GOAL * goalWrtRobot.y / goalDist;
    }
    ROS_DEBUG("GoalF:\t%.2f, %.2f", netForce.x, netForce.y);
    return netForce;
}

/**
 * Add obstacle force from all cached obstacles to the given (goal) 
 * currently active force.
 */
void APF::updateNetForce(Point &netForce){
    stringstream ss; corobot_common::Goal topicMsg;
    int objIndex = 0;
    for (std::vector<CachedPoint>::iterator it = activeObstacleList.begin() ; it != activeObstacleList.end(); ++it){
        Polar *od = convertFromGlobalToRobotInPolar(it->p);
        double f = K_OBS * (1.0/D_OBS - 1.0/od->d) / (od->d * od->d);
        netForce.x += f * cos(od->a);
        netForce.y += f * sin(od->a);
        ROS_DEBUG("Obj(%d)F:\t%.2f, %.2f", ++objIndex, f * cos(od->a), f * sin(od->a));
    }
    ss.str(""); ss << "(" << netForce.x << ", " << netForce.y << ")";
    topicMsg.name = ss.str(); netForcePublisher.publish(topicMsg);      
}

/**
 * Update the obstacle cache: 
 * - Eliminate any obstacles not seen in OBS_CACHE_TIMEOUT seconds
 * - Eliminate any obstacles farther than D_OBS away
 * - Eliminate any obstacles behind the robot (necessary?)
 * - Add any new unique obstacles
 * - Update time-last-seen of present obstacles already in cache
 */
void APF::updateObstacleList(list<Polar>& objects){
    stringstream ss; corobot_common::Goal topicMsg;
    int objIndex = 0;
    // First throw away old obstacles, or those behind us or far away.
    for (std::vector<CachedPoint>::iterator it = activeObstacleList.begin() ; it != activeObstacleList.end(); ++it){
        Point obs = it->p;
        double obsD = length(obs.x-pose.x,obs.y-pose.y);
        double obsAbsA = atan2(obs.y-pose.y,obs.x-pose.x);
        double obsRelA = pose.theta - obsAbsA;
        while (obsRelA > 2*M_PI) obsRelA -= 2*M_PI;
        while (obsRelA <= 0) obsRelA += 2*M_PI;
        ROS_DEBUG("APF, Obj(%d) Distance:\t%.2f Angle:\t%.3f ", ++objIndex, obsD, obsRelA);

        if ((obsD > D_OBS) || (lastScanTime - it->lastT > OBS_CACHE_TIMEOUT) ||
           ((obsRelA > M_PI/2) && (obsRelA < 3*M_PI/2)) || (obsRelA >= angleMin && obsRelA <= angleMax)) {
            ss.str(""); ss << "(" << obs.x << ", " << obs.y << "), rem : " << objIndex;
            activeObstacleList.erase(it);
            
            ROS_DEBUG("APF: Object(%d) removed", objIndex);
            topicMsg.name = ss.str(); obsPublisher.publish(topicMsg);
            
            if(it == activeObstacleList.end())
                break;
        }
    }
    // now add in new ones if they are close enough to worry about.
    for (list<Polar>::iterator it = objects.begin(); it != objects.end(); ++it) {
        Polar pointWrtRobot = *it;
        if (pointWrtRobot.d <= D_OBS) {
            corobot::SimplePose *pointWrtGlobal = convertRobotToGlobal(pointWrtRobot);
            ROS_DEBUG("Polar Object that might be added:\t(%.2f, %.2f)", pointWrtRobot.d, pointWrtRobot.a);
            ROS_DEBUG("Object in global cood:\t(%.2f, %.2f)", pointWrtGlobal->x, pointWrtGlobal->y);
            pushIfUnique(pointWrtGlobal);
        }
    } 

}

/**
 * Supposedly used to check whether the robot needs to ignore the
 * potential field and go into recovery mode.
 */
void APF::recoveryCheck(const double &recov_time_now){
    if( ( waypointQueue.size() - prevWayPointQuelen ) == 0)
    {
       if( recov_time_now - timeSinceLastWayPoint > 60)
            recoverRobot();
    }
    else if( goal.x != 0 && goal.y != 0 && (pose.x > -.1 && pose.x < .1 ) && (pose.y > -.1 && pose.y < .1)){
        recoverRobot();
    }
    else
    {
         timeSinceLastWayPoint = recov_time_now;
         prevWayPointQuelen = waypointQueue.size();
         stringstream ss; corobot_common::Goal topicMsg;
         ss << "Not in Recovery";
         topicMsg.name = ss.str(); recoveryPublisher.publish(topicMsg); 
    }
}

/**
 * Start the recovery process.
 */
void APF::recoverRobot(){
    activeObstacleList.clear(); //clear obstacles for when recovery started because the robot might be surrounded by obstacles

    ROS_DEBUG("Recovery protocol triggered");
    stringstream ss; corobot_common::Goal topicMsg;
    ss << "Recovery Started";
    topicMsg.name = ss.str(); recoveryPublisher.publish(topicMsg);
    recoveryPublisher.publish(topicMsg);

    //some recovery loop here in which the robot just moves around until it sees a barcode
    //
    inRecovery = true;
}

/**
 * Wander safely (until we find a barcode and reorient ourselves)
 */
Polar APF::doRecoveryNav(LaserScan &scan){
    //Polar cmd; cmd.d = 0.1; cmd.a = 0;
    Polar cmd; cmd.d = 0.1; cmd.a = 0;
    list<Polar> objects = findLocalMinima(findObjects(scanToList(scan)));
    for (list<Polar>::iterator it = objects.begin(); it != objects.end(); ++it) {
        Polar objWrtRobot = *it;
        if (objWrtRobot.d <= 1 && abs(objWrtRobot.a) <= 0.6) { // if there's any object within the defined window, then turn in place
            cmd.d = 0; cmd.a = 0.4;
            cmd.d = bound(cmd.d, cmdPrev.d, 0.05); // for decelerating faster
            //ROS_DEBUG("In recovery, NavVel = \t(%.2f,%.2f)",cmd.d,cmd.a);
            cmdPrev = cmd;
            return cmd;
            //corobot::SimplePose *pointWrtGlobal = convertRobotToGlobal(objWrtRobot);
            //ROS_DEBUG("Polar Object that might be added:\t(%.2f, %.2f)", objWrtRobot.d, objWrtRobot.a);
            //ROS_DEBUG("Object in global cood:\t(%.2f, %.2f)", pointWrtGlobal->x, pointWrtGlobal->y);
        }
    }
    cmd.d = bound(cmd.d, cmdPrev.d, 0.010);
    //ROS_DEBUG("In recovery, NavVel = \t(%.2f,%.2f)",cmd.d,cmd.a);
    cmdPrev = cmd;
    return cmd;
}

