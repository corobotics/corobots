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

int main(int argc, char **argv) {
    // Initialize the ROS node.
    ros::init(argc, argv, "corobot_qrcode");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<Pose>("qrcode_pose", 1000);

    // Create our barcode detected handler.
    BarcodeHandler my_handler(chatter_pub);

    // Create the zbar processor; this will run in its own thread.
    Processor proc;
    // Don't change the resolution, will screw up everything!
    proc.request_size(1600, 1200);
    // Initialize after setting size; no X window.
    proc.init("dev/video0", false);
    // Configure the processor to detect QR codes.
    proc.set_config(ZBAR_QRCODE, ZBAR_CFG_ENABLE, 1);
    // Set the handler.
    proc.set_handler(my_handler);
    // Start the processor in "free-running video mode".
    proc.set_active();

    // ROS loop.
    ros::spin();
    return 0;
}

