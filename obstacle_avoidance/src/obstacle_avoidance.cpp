#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "turtlebot_node/Turtle.h"

using namespace std;
using namespace ros;
using namespace geometry_msgs;
using namespace sensor_msgs;

void ObstacleAvoider::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
void ObstacleAvoider::poseCallback(const geometry_msgs::Pose2D::ConstPtr& pose);
void ObstacleAvoider::waypointCallback(const geometry_msgs::Point::ConstPtr& waypoint);

Publisher pub;

int main(int argc, char** argv) {
    init(argc, argv, "obstacle_avoidance");
    NodeHandle n;
    Publisher pub = n.advertise<turtlebot_node::Turtle>("cmd_vel", 1000);
    APF obsAvoider;
    Subscriber scanSub = n.subscribe("scan", 1000, obsAvoider.scanCallback);
    Subscriber poseSub = n.subscribe("pose", 1000, obsAvoider.poseCallback);
    Subscriber waypointSub = n.subscribe("waypoints", 1000, obsAvoider.waypointCallback);
    spin();
    return 0;
}
