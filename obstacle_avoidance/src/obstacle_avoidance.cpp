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

void ObstacleAvoider::updatePose(const Pose2D::ConstPtr& pose) {

}

void ObstacleAvoider::addWaypoint(const Point::ConstPtr& waypoint) {

}

Point APF::nav(const LaserScan::ConstPtr& scan) {
    Point p;
    return p;
}

void scanCallback(const LaserScan::ConstPtr& scan) {

}

void poseCallback(const Pose2D::ConstPtr& pose) {

}

void waypointCallback(const Point::ConstPtr& waypoint) {

}

ros::Publisher pub;

int main(int argc, char** argv) {
    ros::init(argc, argv, "obstacle_avoidance");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<turtlebot_node::Turtle>("cmd_vel", 1000);
    APF obsAvoider;
    ros::Subscriber scanSub = n.subscribe("scan", 1000, scanCallback);
    ros::Subscriber poseSub = n.subscribe("pose", 1000, poseCallback);
    ros::Subscriber waypointSub = n.subscribe("waypoints", 1000, waypointCallback);
    ros::spin();
    return 0;
}
