#include <cmath>
#include <list>

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose2D.h"
#include "sensor_msgs/LaserScan.h"

#include "apf.h"

using namespace std;
using geometry_msgs::Point;
using geometry_msgs::Pose2D;
using sensor_msgs::LaserScan;

float InversePowerForce::calc(const float& dist) {
    return pow(dist, -exp);
}

APF::APF(const float& ko, const float& kg) :
    ko(ko), kg(kg)
{
    distForce = new InversePowerForce(2.0);
};

APF::APF(const float& ko, const float& kg, ForceCalc* distForce) :
    ko(ko), kg(kg), distForce(distForce) {};

list<Polar> APF::findLocalMinima(LaserScan scan) {
    // List of "object" points: local minima of the scan.
    list<Polar> objLocs;
    // Object for setting and adding to objLocs.
    Polar objLoc;
    // Loop variable for current distance.
    float d;
    // Loop variable for the current angle.
    float a = scan.angle_min;
    // Whether the last angle's distance was bigger than the current one.
    bool lastWasFurther = true;
    // Search each data point.
    for (unsigned int i = 0; i < scan.ranges.size() - 1; i++) {
        d = scan.ranges[i];
        // If d is a valid distance and closer than [i+1],
        if (d > scan.range_min && d < scan.range_max && d <= scan.ranges[i + 1]) {
            // If the previous range was also further, we have a minima.
            if (lastWasFurther) {
                objLoc.d = d;
                objLoc.a = a;
                objLocs.push_back(objLoc);
            }
            lastWasFurther = false;
        } else {
            lastWasFurther = true;
        }
        a += scan.angle_increment;
    }
    return objLocs;
}



Point APF::nav(LaserScan scan) {
    // The goal is the head of the waypoint queue. TODO: Handle empty queue?
    Point goal = waypointQueue.front();

    // The list of "objects" found; right now using local minima of the scan.
    list<Polar> objLocs = findLocalMinima(scan);

    // Stores the object force vector summation. z is ignored.
    Point sum;
    sum.x = 0;
    sum.y = 0;
    double force;

    // Sum over all obstacles.
    for (list<Polar>::iterator p = objLocs.begin(); p != objLocs.end(); ++p) {
        force = 0; // f(p.d);
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
