#include <iostream>

#include "ros/ros.h"
#include "corobot_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

#include "corobot.h"
#include "obstacle_avoidance.h"
#include "apf.h"

using namespace std;
using corobot::length;
using corobot_msgs::Pose;
using geometry_msgs::Point;
using geometry_msgs::Twist;
using sensor_msgs::LaserScan;

/**
 * {@inheritDoc}
 */
void ObstacleAvoider::updatePose(Pose newPose) {
    pose = newPose;
    if (!waypointQueue.empty()) {
        Point goal = waypointQueue.front();
        while (!waypointQueue.empty() &&
                length(pose.x - goal.x, pose.y - goal.y) < ARRIVED_DISTANCE) {
            waypointQueue.pop();
            arrivedQueue.push(goal);
            goal = waypointQueue.front();
        }
    }
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
 * The publisher for waypoints the robot has reached.
 */
ros::Publisher waypointsReachedPub;

/**
 * The obstacle avoider to use.
 */
ObstacleAvoider* oa;

/**
 * Callback for laserscan messages.
 */
void scanCallback(LaserScan scan) {
    Twist t;
    Polar p = oa->nav(scan);
    t.linear.x = p.d;
    t.angular.z = p.a;
    cmdVelPub.publish(t);
}

/**
 * Callback for pose messages.
 */
void poseCallback(Pose pose) {
    oa->updatePose(pose);
    while (!oa->arrivedQueue.empty()) {
        waypointsReachedPub.publish(oa->arrivedQueue.front());
        oa->arrivedQueue.pop();
    }
}

/**
 * Callback for new waypoint messages.
 */
void waypointCallback(Point waypoint) {
    oa->addWaypoint(waypoint);
    cout << "Waypoint added: " << waypoint.x << ", " << waypoint.y << endl;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "obstacle_avoidance");
    ros::NodeHandle n;
    cmdVelPub = n.advertise<Twist>("cmd_vel", 1000);
    waypointsReachedPub = n.advertise<Point>("waypoints_reached", 1000);
    oa = new APF();
    ros::Subscriber scanSub = n.subscribe("scan", 1000, scanCallback);
    ros::Subscriber poseSub = n.subscribe("pose", 1000, poseCallback);
    ros::Subscriber waypointSub = n.subscribe("waypoints", 1000, waypointCallback);
    ros::spin();
    delete oa;
    return 0;
}
