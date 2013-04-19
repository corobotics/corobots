#include <iostream>
#include <zbar.h>
#include "ros/ros.h"
#include <sstream>
#include <cmath>
#include <string>
#include "std_msgs/String.h"
#include "../include/CSVReader.h"
#include "barcodeHandler.h"
#include "corobot_common/Pose.h"

#define PI 3.14159265

using namespace std;
using namespace zbar;
using corobot_common::Pose;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "corobot_qrcode");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<Pose>("qrcode_pose", 1000);

    // std_msgs::String
    // create and initialize a Processor
    const char* device = "/dev/video0";

    Processor proc(false, device, false);
    // Don't change the resolution, will screw up everything!
    // Don't know if this is actually taking effect.
    proc.request_size(1600,1200);
    //proc.init(device, false);

    // configure the Processor
    proc.set_config(ZBAR_QRCODE, ZBAR_CFG_ENABLE, 1);

    // setup a callback
    BarcodeHandler my_handler(chatter_pub);

    proc.set_handler(my_handler);

    proc.set_active();

    // keep scanning until user provides key/mouse input
    while (ros::ok()) {
        proc.process_one(10);
        ros::spinOnce();
    }

    return 0;
}

