#include "obstacle_avoidance.h"

#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose2D.h"
#include "sensor_msgs/LaserScan.h"
#include "turtlebot_node/Turtle.h"

using geometry_msgs::Point;
using geometry_msgs::Pose2D;
using sensor_msgs::LaserScan;
using turtlebot_node::Turtle;

void ObstacleAvoider::updatePose(const Pose2D::ConstPtr& newPose) {
    pose = newPose;
    // Check if reached waypoint.
}

void ObstacleAvoider::addWaypoint(const Point::ConstPtr& waypoint) {
    waypointQueue.push(waypoint);
}

Turtle APF::nav(const LaserScan::ConstPtr& scan) {
    Turtle t;
    return t;
}

ObstacleAvoider* oa = NULL;

void scanCallback(const LaserScan::ConstPtr& scan) {
    Turtle t = oa->nav(scan);
    // Publish.
}

void poseCallback(const Pose2D::ConstPtr& pose) {
    oa->updatePose(pose);
}

void waypointCallback(const Point::ConstPtr& waypoint) {
    oa->addWaypoint(waypoint);
}

ros::Publisher pub;

int main(int argc, char** argv) {
    ros::init(argc, argv, "obstacle_avoidance");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<turtlebot_node::Turtle>("cmd_vel", 1000);
    oa = new APF();
    ros::Subscriber scanSub = n.subscribe("scan", 1000, scanCallback);
    ros::Subscriber poseSub = n.subscribe("pose", 1000, poseCallback);
    ros::Subscriber waypointSub = n.subscribe("waypoints", 1000, waypointCallback);
    ros::spin();
    delete oa;
    return 0;
}
