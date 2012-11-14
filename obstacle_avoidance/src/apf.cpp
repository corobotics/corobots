#include "sensor_msgs/LaserScan.h"
#include "turtlebot_node/Turtle.h"

#include "apf.h"

using namespace std;
using geometry_msgs::Point;
using geometry_msgs::Pose2D;
using sensor_msgs::LaserScan;
using turtlebot_node::Turtle;

typedef struct {
    float d;
    float a;
} Polar;

list<Polar> findLocalMinima(LaserScan scan) {
    list<Polar> objLocs; // List of "object" points: local minima of the scan.
    Polar objLoc; // Object for setting and adding to objLocs.
    float d; // Loop variable for current distance.
    float a = scan.angle_min; // Loop variable for the current angle.
    bool lastWasBigger = true; // Whether the last angle's distance was bigger than the current one.
    for (unsigned int i = 0; i < scan.ranges.size() - 1; i++) {
        d = scan.ranges[i];
        if (d > scan.range_min && d < scan.range_max && d <= scan.ranges[i + 1]) {
            if (lastWasBigger) {
                objLoc.d = d;
                objLoc.a = a;
                objLocs.push_back(objLoc);
            }
            lastWasBigger = false;
        } else {
            lastWasBigger = true;
        }
        a += scan.angle_increment;
    }
    return objLocs;
}

Turtle APF::nav(LaserScan scan) {
    Turtle t;
    return t;
}
