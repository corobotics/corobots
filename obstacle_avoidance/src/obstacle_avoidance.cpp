#include <iostream>

#include "ros/ros.h"
#include "corobot_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

#include "obstacle_avoidance.h"
#include "apf.h"

/** The goal constant to use for APF. */
#define KGOAL 1.8

/** The obstacle constant to use for APF. */
#define KOBS 0.5

using namespace std;
using corobot_msgs::Pose;
using geometry_msgs::Point;
using geometry_msgs::Twist;
using sensor_msgs::LaserScan;

/**
 * {@inheritDoc}
 */
bool ObstacleAvoider::hasWaypoint() {
    return !waypointQueue.empty();
}

/**
 * {@inheritDoc}
 */
void ObstacleAvoider::updatePose(Pose newPose) {
    pose = newPose;
    // Check if reached waypoint.
}

/**
 * {@inheritDoc}
 */
void ObstacleAvoider::addWaypoint(Point waypoint) {
    waypointQueue.push(waypoint);
}

/**
 * The publisher for movement velocity commands.
 */
ros::Publisher cmdVelPub;

/**
 * The obstacle avoider to use.
 */
ObstacleAvoider* oa;

/**
 * Callback for laserscan messages.
 */
void scanCallback(LaserScan scan) {
    Twist t;
    if (!oa->hasWaypoint()) {
        cout << "No waypoints; returning." << endl;
        cmdVelPub.publish(t);
        return;
    }
    Point p = oa->nav(scan);
    cout << "Nav vector: <" << p.x << ", " << p.y << ">" << endl;
    // Publish.
    t.linear.x = sqrt(p.x * p.x + p.y * p.y);
    t.angular.z = atan2(p.y, p.x);
    cmdVelPub.publish(t);
}

/**
 * Callback for pose messages.
 */
void poseCallback(Pose pose) {
    oa->updatePose(pose);
}

/**
 * Callback for new waypoint messages.
 */
void waypointCallback(Point waypoint) {
    oa->addWaypoint(waypoint);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "obstacle_avoidance");
    ros::NodeHandle n;
    cmdVelPub = n.advertise<Twist>("cmd_vel", 1000);
    oa = new APF(KOBS, KGOAL);
    ros::Subscriber scanSub = n.subscribe("scan", 1000, scanCallback);
    ros::Subscriber poseSub = n.subscribe("pose", 1000, poseCallback);
    ros::Subscriber waypointSub = n.subscribe("waypoints", 1000, waypointCallback);
    ros::spin();
    delete oa;
    return 0;
}
