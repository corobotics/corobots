#include <iostream>

#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"

using namespace std;
using geometry_msgs::PoseWithCovariance;
using nav_msgs::Odometry;
using sensor_msgs::LaserScan;

ros::Publisher pub;

void odomCallback(Odometry odom) {
    pub.publish(odom.pose);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "localization");
    ros::NodeHandle n;
    pub = n.advertise<PoseWithCovariance>("pose", 1000);
    ros::Subscriber odomSub = n.subscribe("odom", 1000, odomCallback);
    ros::spin();
    return 0;
}
