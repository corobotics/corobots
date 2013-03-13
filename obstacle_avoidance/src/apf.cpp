#include <cmath>
#include <iostream>
#include <list>

#include "corobot_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/LaserScan.h"

#include "apf.h"

using namespace std;
using corobot_msgs::Pose;
using geometry_msgs::Point;
using sensor_msgs::LaserScan;

/**
 * {@inheritDoc}
 */
float InversePowerForce::calc(const float& dist) {
    return pow(dist, -exp);
}

bool Polar::isValid() const {
    return d >= 0.0;
}

/**
 * {@inheritDoc}
 */
APF::APF(const float& ko, const float& kg) :
    ko(ko), kg(kg)
{
    distForce = new InversePowerForce(2.0);
};

/**
 * {@inheritDoc}
 */
APF::APF(const float& ko, const float& kg, ForceCalc* distForce) :
    ko(ko), kg(kg), distForce(distForce) {};

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
Point APF::nav(LaserScan scan) {

    // Can't do anything without a goal.
    if (waypointQueue.empty()) {
        Point p;
        return p;
    }

    // The goal is the head of the waypoint queue.
    Point goal = waypointQueue.front();

    cout << "Goal: (" << goal.x << ", " << goal.y << ")" << endl;
    cout << "Pose: (" << pose.x << ", " << pose.y << ") " << pose.theta << endl;

    // The list of "objects" found.
    list<Polar> objects = findLocalMinima(findObjects(scanToList(scan)));

    // Stores the object force vector summation. z is ignored.
    Point sum;
    sum.x = 0;
    sum.y = 0;
    double force;

    cout << endl << endl;
    // Sum over all obstacles.
    for (list<Polar>::iterator p = objects.begin(); p != objects.end(); ++p) {
        cout << "Obj: " << p->a << ", " << p->d << endl;
        force = distForce->calc(p->d);
        sum.x += force * cos(p->a);
        sum.y += force * sin(p->a);
    }

    // Obstacle gain.
    sum.x *= ko;
    sum.y *= ko;

    // Angle between the robot and the goal.
    double theta = atan2(goal.y - pose.y, goal.x - pose.x) - pose.theta;

    // Resulting vector.
    Point res;
    res.x = kg * cos(theta) - sum.x;
    res.y = kg * sin(theta) - sum.y;

    return res;
}
