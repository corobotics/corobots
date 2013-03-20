#include <cmath>
#include <iostream>
#include <list>

#include "corobot_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/LaserScan.h"

#include "corobot.h"
#include "apf.h"

using namespace std;
using corobot::rCoordTransform;
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
Polar APF::nav(LaserScan scan) {

    // Can't do anything without a goal.
    if (waypointQueue.empty()) {
        cout << "No waypoints in queue!" << endl;
        Polar p;
        return p;
    }

    // The goal is the head of the waypoint queue.
    Point goal = waypointQueue.front();
    // Convert the goal into the robot reference frame.
    Point g = rCoordTransform(goal, pose);

    cout << endl;
    printf("Goal:\t%.2f, %.2f\n", g.x, g.y);

    // The list of "objects" found; already in the robot reference frame.
    list<Polar> objects = findLocalMinima(findObjects(scanToList(scan)));

    // Stores the object force vector summation. z is ignored.
    Point sum;
    sum.x = kg * g.x;
    sum.y = kg * g.y;

    // Sum over all obstacles.
    for (list<Polar>::iterator p = objects.begin(); p != objects.end(); ++p) {
        double force = distForce->calc(p->d);
        printf("Obj:\t%.2f, %.2f (%.2f)\n", p->d * cos(p->a), p->d * sin(p->a), force);
        sum.x -= ko * force * cos(p->a);
        sum.y -= ko * force * sin(p->a);
    }

    // Resulting vector.
    Polar res;
    res.d = sqrt(sum.x * sum.x + sum.y * sum.y);
    res.a = atan2(sum.y, sum.x);

    // Don't try to go forward if the angle is more than 45 degrees.
    if (abs(res.a) > PI / 4) {
        res.d = 0;
    }

    printf("Nav:\t<%+.2f, %.2f>\n", res.a, res.d);

    return res;
}
