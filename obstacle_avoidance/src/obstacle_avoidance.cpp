#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "turtlebot_node/Turtle.h"

using namespace ros;

Publisher pub;

void obsAvoidCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
}

int main(int argc, char** argv) {
    init(argc, argv, "obstacle_avoidance");
    NodeHandle n;
    pub = n.advertise<turtlebot_node::Turtle>("cmd_vel", 1000);
    Subscriber sub = n.subscribe("scan", 1000, obsAvoidCallback);
    spin();
    return 0;
}
