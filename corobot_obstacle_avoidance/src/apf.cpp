#include <cmath>
#include <iostream>
#include <list>

#include "ros/console.h"
#include "corobot_common/Pose.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/LaserScan.h"

#include "corobot.h"
#include "apf.h"

using namespace std;
using corobot::bound;
using corobot::length;
using corobot::rCoordTransform;
using corobot_common::Pose;
using geometry_msgs::Point;
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
 */
Polar APF::nav(LaserScan scan) {

    // Can't do anything without a goal.
    if (waypointQueue.empty()) {
        ROS_INFO("No waypoints in queue!");
        Polar p;
        return p;
    }

    ROS_DEBUG("Pose:\t%.2f, %.2f, %.2f", pose.x, pose.y, pose.theta);
    // The goal is the head of the waypoint queue.
    Point goalInMap = waypointQueue.front();
    // Convert the goal into the robot reference frame.
    Point goal = rCoordTransform(goalInMap, pose);

    ROS_DEBUG("Abs Goal:\t%.2f, %.2f", goalInMap.x, goalInMap.y);
    ROS_DEBUG("Rel Goal:\t%.2f, %.2f", goal.x, goal.y);

    // The list of "objects" found; already in the robot reference frame.
    list<Polar> objects = findLocalMinima(findObjects(scanToList(scan)));

    // Stores the object force vector summation. z is ignored.
    Point netForce;
    double goalDist = length(goal.x, goal.y);
    if (goalDist <= D_GOAL) {
        netForce.x = K_GOAL * goal.x;
        netForce.y = K_GOAL * goal.y;
    } else {
        netForce.x = D_GOAL * K_GOAL * goal.x / goalDist;
        netForce.y = D_GOAL * K_GOAL * goal.y / goalDist;
    }
    ROS_DEBUG("GoalF:\t%.2f, %.2f", netForce.x, netForce.y);

    ////construct an obstacle list 
    for (list<Polar>::iterator p = objects.begin(); p != objects.end(); ++p) {
      Polar pointWrtRobot = *p;
      if (pointWrtRobot.d <= D_OBS) {
        corobot::SimplePose *pointWrtGlobal = convertRobotToGlobal(pointWrtRobot);
        ROS_DEBUG("Polar Object that might be added:\t%.2f, %.2f", pointWrtRobot.d, pointWrtRobot.a);
        ROS_DEBUG("Object in global cood:\t%.2f, %.2f", pointWrtGlobal->x, pointWrtGlobal->y);
        pushIfUnique(pointWrtGlobal);
      }
    }
    
    ///// throw out inactive obstacle (which are not in zone anymore).
    ////improve: call only when the robot has changed its position
    for (std::vector<corobot::SimplePose>::iterator it = activeObstacleList.begin() ; it != activeObstacleList.end(); ++it){
      ROS_DEBUG("APF, Obj Distance :\t%.2f", distanceFromRobot(*it));
      if(distanceFromRobot(*it) > D_OBS ){
        activeObstacleList.erase(it);
        ROS_DEBUG("APF: Object removed");
        if(it == activeObstacleList.end())
          break;
        
      } else {
        Polar *od = convertFromGlobalToRobotInPolar(*it);
        Polar o = *od;
        double f = K_OBS * (1.0/D_OBS - 1.0/o.d) / (o.d * o.d);
        netForce.x += f * cos(o.a);
        netForce.y += f * sin(o.a);
        ROS_DEBUG("ObjF:\t%.2f, %.2f", f * cos(o.a), f * sin(o.a));
      }
    }
    
    // Sum over all obstacles.
    /*for (list<Polar>::iterator p = objects.begin(); p != objects.end(); ++p) {
      Polar o = *p;
      ROS_DEBUG("Obj:\t%.2f, %.2f", o.d * cos(o.a), o.d * sin(o.a));
      if (o.d <= D_OBS) {
      // Principles of Robot Motion, pg. 83
      double f = K_OBS * (1.0/D_OBS - 1.0/o.d) / (o.d * o.d);
      netForce.x += f * cos(o.a);
      netForce.y += f * sin(o.a);
      ROS_DEBUG("ObjF:\t%.2f, %.2f", f * cos(o.a), f * sin(o.a));
      }
      }*/
    
    // Resulting command vector.
    Polar cmdInitial;
    cmdInitial.d = length(netForce.x, netForce.y);
    if(abs(netForce.y) <= 0.099 && abs(netForce.x) <= 0.099) {
      ROS_DEBUG("zero net force detected");
      cmdInitial.a = 0;
    }else {
      cmdInitial.a = atan2(netForce.y, netForce.x);
    }
    
    //setting the angles and velocities based on the net force
    /*if(cmd.a >= -ANGLE_WINDOW && cmd.a < ANGLE_WINDOW ){ // quadV, if the net Force is not from the side, but from the front
      cmd.d = min(MAX_FORCE, cmd.d);
      cmd.a = 0;
      } else if(cmd.a >=-1.57 && cmd.a < -ANGLE_WINDOW) { // quad1
      cmd.d = min(MAX_FORCE, cmd.d * cos(cmd.a));
      cmd.a = -MIN_OMEGA;
      } else if (cmd.a < -1.57 && cmd.a >= -3.15) // quad4
      { 
      cmd.d = 0;
      cmd.a = -MIN_OMEGA;
      } else if (cmd.a < 3.15 && cmd.a > 1.57 ) // quad3
      { 
      cmd.d = 0;
      cmd.a = MIN_OMEGA;
      }
      else{ // quad2
      cmd.d = min(MAX_FORCE, cmd.d * cos(cmd.a));
      cmd.a = MIN_OMEGA;
      }*/
    
    ROS_DEBUG("RawNav:\t<%+.2f, %.2f>", cmdInitial.a, cmdInitial.d);
    
    Polar cmd;
    // Don't try to go forward if the angle is more than fixed value.
    if (cmdInitial.a > ANGLE_WINDOW) {
      cmd.a = MIN_OMEGA;
      if (cmdInitial.a < M_PI/2.0) {
        cmd.d = cos(cmdInitial.a)*cmdInitial.d;
      } else
        cmd.d = 0;
    } else if (cmdInitial.a < -1*ANGLE_WINDOW) {
      cmd.a = -1*MIN_OMEGA;
      if (cmdInitial.a > -M_PI/2.0) {
        cmd.d = cos(cmdInitial.a)*cmdInitial.d;
      } else
        cmd.d = 0;
    } else {
      cmd.a = 0;
      if (cmdInitial.d > 0.25)
	cmd.d = 0.25;
      else
	cmd.d = cmdInitial.d;
      cmd.d = bound(cmd.d, cmdPrev.d, 0.05);
    }
    ROS_DEBUG("NavVel:\t<%+.2f, %.2f>\n", cmd.a, cmd.d);

    double now = scan.header.stamp.toSec();
    if (cmd.d > 0.0 || timeLastMoved == 0.0) {
        timeLastMoved = now;
    } else if (now - timeLastMoved > 10.0) {
        // Haven't moved in too long; give up on this waypoint.
        waypointQueue.pop();
        failedQueue.push(goalInMap);
        // Reset the timestamp so we don't give up on subsequent waypoints.
        timeLastMoved = 0.0;
    }
 
    cmdPrev = cmd;
    return cmd;
}

corobot::SimplePose* APF::convertRobotToGlobal(Polar polarPoint){
        corobot::SimplePose *sp = new corobot::SimplePose();
        sp->x = pose.x + polarPoint.d * cos(pose.theta + polarPoint.a);
        sp->y = pose.y + polarPoint.d * sin(pose.theta + polarPoint.a);
        sp->a = 0;
        ROS_DEBUG("APF, Returning RToG Coordinates");
        return sp;
}

bool APF::pushIfUnique(corobot::SimplePose *sp){
        for (std::vector<corobot::SimplePose>::iterator it = activeObstacleList.begin() ; it != activeObstacleList.end(); ++it){
                corobot::SimplePose itPose = *it;
                if(abs(sp->x - itPose.x) <= 1 && abs(sp->y - itPose.y) <= 1){ //if the point approx matches the points in the list
                        ROS_DEBUG("APF, PushUnique returns False");
                        return false;
                }
        }
        
        //ROS_DEBUG("APF, Pushing Object into activeObstacleList: %.2f, %.2f", sp->x, sp->y);
        activeObstacleList.push_back(*sp);
        ROS_DEBUG("APF, current list Size: %d", activeObstacleList.size());
        return true;
}

double APF::distanceFromRobot(corobot::SimplePose sp){
        return sqrt((sp.x-pose.x)*(sp.x-pose.x) + (sp.y-pose.y)*(sp.y-pose.y));
}

Polar* APF::convertFromGlobalToRobotInPolar(corobot::SimplePose sp){
  Polar *p = new Polar();
  if(abs(sp.y-pose.y) <= 0.099 && abs(sp.x - pose.x) <= 0.099){
    p->a = 0;
    p->d = 0;
    ROS_DEBUG("APF, Returning GToR: zeros detected");
  }
  else{ 
    p->a = atan2(sp.y-pose.y, sp.x - pose.x) - pose.theta;
    p->d = distanceFromRobot(sp);
  }
  ROS_DEBUG("APF, Returning GToR coordinates");
  return p;
}

double APF::min(double a, double b){
        if(a<=b) 
                return a;
        return b;
}
