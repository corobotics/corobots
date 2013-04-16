#ifndef BARCODEHANDLER_H
#define BARCODEHANDLER_H

#include <iostream>
#include <sstream>
#include <cmath>
#include <string>
#include <zbar.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "../include/CSVReader.h"
#include "corobot_common/Pose.h"

#define PI 3.14159265

typedef struct {
	 int x, y;
} Pt;

using namespace std;
using namespace zbar;
using corobot_common::Pose;

class BarcodeHandler : public Image::Handler {
public:

    ros::Publisher publisher;
    Pt point[4];
    int lengthPixelL, lengthPixelR;
    float distanceL, distanceR,squareDistanceL, squareDistanceR, angleR, angleL, angleAvg, distanceAvg, offsetDistance, barcodeXavg;
    float cbx, cby, cbtheta, bcx, bcy, bctheta, alpha, gamma;
    float barcodeX, barcodeY;
    string barcodeOrientation;
    CSVReader csvreader;
    Pose msg;
    
    BarcodeHandler(ros::Publisher & chatter_pub);
    void image_callback(Image & image);

};

#endif
