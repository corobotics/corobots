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

list<Polar APF::scanToList(LaserScan scan) {
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
list<Polar> APF::findObjects(LaserScan scan) {
    // List of "object" points: local minima of the scan.
    list<Polar> objLocs;
    // Object for setting and adding to objLocs.
    Polar objMin;
    Polar lastObjectMin;
    bool lastObject = false;
    bool lastObjectWasLess = false;
    // Loop variable for current distance.
    float d;
    // Loop variable for the current angle.
    float a = scan.angle_min;
    // State variable for whether we're currently in a contiguous object.
    bool inObject = false;
    cout << endl;
    for (unsigned int i = 0; i < scan.ranges.size(); i++) {
        d = scan.ranges[i];
        // If d is a valid distance and closer than [i+1],
        if (d > scan.range_min && d < scan.range_max) {
            if (inObject) {
                if (d < objMin.d) {
                    objMin.d = d;
                    objMin.a = a;
                }
            } else {
                objMin.a = a;
                objMin.d = d;
                inObject = true;
            }
            // Check whether the next point is also in the object.
            if (i < scan.ranges.size() - 1 && Math.abs(d - scan.ranges[i + 1]) < 0.2) {
                inObject = true;
            } else {
                // At the end of an object.
                inObject = false;
                // If there was an adjacent previous object...
                if (lastObject) {
                    // and it was an object local minima, store it.
                    if (lastObjectMin.d < objMin.d && lastObjectWasLess) {
                        objLocs.push_back(lastObjectMin);
                        lastObjectWasLess = false;
                    } else {
                        lastObjectWasLess = true;
                    }
                } else {
                    lastObjectWasLess = true;
                }
                lastObject = true;
                lastObjectMin = objMin;
            }
        } else {
            inObject = false;
            lastObject = false;
        }
        a += scan.angle_increment;
    }
    cout << endl;
    return objLocs;
}

/**
 * {@inheritDoc}
 */
Point APF::nav(LaserScan scan) {
    // The goal is the head of the waypoint queue. TODO: Handle empty queue?
    Point goal = waypointQueue.front();

    cout << "Goal: (" << goal.x << ", " << goal.y << ")" << endl;
    cout << "Pose: (" << pose.x << ", " << pose.y << ") " << pose.theta << endl;

    // The list of "objects" found; right now using local minima of the scan.
    list<Polar> objLocs = findLocalMinima(scan);

    // Stores the object force vector summation. z is ignored.
    Point sum;
    sum.x = 0;
    sum.y = 0;
    double force;

    // Sum over all obstacles.
    for (list<Polar>::iterator p = objLocs.begin(); p != objLocs.end(); ++p) {
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
