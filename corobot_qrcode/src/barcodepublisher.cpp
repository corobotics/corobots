#include <zbar.h>
#include <ros/ros.h>
#include <corobot_common/Pose.h>
#include <ros/console.h>

#include "CSVReader.h"
#include "barcodeHandler.h"

#include <string>

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
    ros::NodeHandle nh("~");
    char  dn;
    nh.getParam("device", dn);
    cout<<device_number;
    // Don't change the resolution, will screw up everything!
    proc.request_size(1600, 800);
    // Initialize after setting size; no X window.
    // setting it to true works on local machine.
    proc.init("/dev/videoleft", true);
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

