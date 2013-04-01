#ifndef BARCODEHANDLER_H
#define BARCODEHANDLER_H

#include <iostream>
#include <zbar.h>
#include <sstream>
#include "ros/ros.h"
#include <cmath>
#include <string>
#include "std_msgs/String.h"
#include "../include/CSVReader.h"
#include "corobot_msgs/Pose.h"



#define PI 3.14159265

class pt{
	public: 
	 int x,y;
};


using namespace std;
using namespace zbar;
using corobot_msgs::Pose;

class BarcodeHandler:public Image::Handler {

    public:

    ros::Publisher test;
    pt point[4];
    int lengthPixelL,lengthPixelR;
    float distanceL, distanceR,squareDistanceL, squareDistanceR, angleR, angleL, angleAvg, distanceAvg, offsetDistance;
    int barcodeX, barcodeY;
    string barcodeOrientation;
    CSVReader csvreader;
    

    BarcodeHandler(ros::Publisher & chatter_pub);
    void image_callback(Image & image);

};

#endif
