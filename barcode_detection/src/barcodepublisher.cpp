#include <iostream>
#include <zbar.h>
#include "ros/ros.h"
#include <sstream>
#include <cmath>
#include <string>
#include "std_msgs/String.h"
#include "../include/CSVReader.h"
#include "barcodeHandler.h"
#include "corobot_msgs/Pose.h"


#define PI 3.14159265

using namespace std;
using namespace zbar;
using corobot_msgs::Pose;


int main(int argc, char **argv)
{
    
     
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise <Pose> ("chatter", 1000);

    // std_msgs::String 
    // create and initialize a Processor
    const char *device = "/dev/video0";

   
    Processor proc;
    //Don't change the resolution, will screw up everything!
    proc.request_size(1600,1200);
    proc.init(device,true);

    // configure the Processor
    proc.set_config(ZBAR_QRCODE, ZBAR_CFG_ENABLE, 1);
	 
    // setup a callback
    BarcodeHandler my_handler(chatter_pub);
    proc.set_handler(my_handler);

    // enable the preview window
    proc.set_visible();
    proc.set_active();
	
    try {
	// keep scanning until user provides key/mouse input
	
	proc.process_one();
    }
    catch(ClosedError & e) {
    }
	
	
    return (0);
}

