#include <iostream>

#include "ros/console.h"
#include "ros/ros.h"
#include "corobot_common/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

#include "corobot.h"
#include "obstacle_avoidance.h"
#include "apf.h"

using namespace std;
using corobot::length;
using corobot_common::Pose;
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
    ROS_INFO("Cmd: %.2f, %.2f rads", p.d, p.a);
}

/**
 * Callback for pose messages.
 */
void poseCallback(Pose pose) {
    oa->updatePose(pose);
    while (!oa->arrivedQueue.empty()) {
        Point reached = oa->arrivedQueue.front();
        waypointsReachedPub.publish(reached);
        oa->arrivedQueue.pop();
        ROS_INFO("Waypoint reached: (%.2f, %.2f)", reached.x, reached.y);
    }
}

/**
 * Callback for new waypoint messages.
 */
void waypointCallback(Point waypoint) {
    oa->addWaypoint(waypoint);
    ROS_INFO("Waypoint added: (%.2f, %.2f)", waypoint.x, waypoint.y);
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
