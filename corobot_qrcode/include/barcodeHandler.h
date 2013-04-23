#ifndef BARCODEHANDLER_H
#define BARCODEHANDLER_H

#include <zbar.h>
#include <ros/ros.h>
#include <corobot_common/Pose.h>

#include "CSVReader.h"

#define PI 3.14159265

typedef struct {
	 int x, y;
} Pt;

class BarcodeHandler : public zbar::Image::Handler {
public:

    ros::Publisher publisher;
    Pt point[4];
    int lengthPixelL, lengthPixelR;
    float distanceL, distanceR, squareDistanceL, squareDistanceR, angleR, angleL, angleAvg, distanceAvg, offsetDistance, barcodeXavg;
    float cbx, cby, cbtheta, bcx, bcy, bctheta, alpha, gamma;
    float barcodeX, barcodeY;
    std::string barcodeOrientation;
    CSVReader csvreader;
    corobot_common::Pose msg;
    
    BarcodeHandler(ros::Publisher & chatter_pub);
    void image_callback(zbar::Image & image);

};

#endif
