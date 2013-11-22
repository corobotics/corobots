#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include <string>
#include <sstream>

/*void batteryCallback(const ros::TimerEvent&){
    
    ros::NodeHandle n;
    ros::Publisher p = n.advertise<std_msgs::String>("battery", 1024);
    std_msgs::String msg;
    std::stringstream ss;

    ss<<"Hello! I'm Batman!";
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());
    p.publish(msg);
}*/

void batteryCallback(const std_msgs::String::ConstPtr& msg){
    ROS_INFO("Battery : %s", msg->data.c_str());
}

void laptopCallback(const std_msgs::String::ConstPtr& msg){
    std::string s = msg->data;
    std::stringstream ss(s);
    int x;
    ss >> x;
    ROS_INFO("Laptop Battery : %d", x);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "corobot_battery");
    ros::NodeHandle n;
    ros::Subscriber s = n.subscribe("laptopBatman", 1000, laptopCallback);
    ros::Subscriber s1 = n.subscribe("batman", 1000, batteryCallback);
    //ros::Timer t = n.createTimer(ros::Duration(1.0), batteryCallback);
    ros::spin();

    return 0;
}
