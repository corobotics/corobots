#ifndef BARCODEHANDLER_H
#define BARCODEHANDLER_H

#include <zbar.h>
#include <ros/ros.h>
#include <corobot_common/Pose.h>
#include "corobot_common/Goal.h"

#include "CSVReader.h"

#define PI 3.14159265

typedef struct {
	 int x, y;
} Pt;

class BarcodeHandler : public zbar::Image::Handler {
public:

	int qrCount;
	ros::Publisher qrCodeCountPublisher;
    ros::Publisher publisher;
    Pt point[4];
    int lengthPixelL, lengthPixelR;
    float distanceL, distanceR, squareDistanceL, squareDistanceR, angleR, angleL, angleAvg, distanceAvg, offsetDistance, barcodeXavg;
    float cbx, cby, cbtheta, bcx, bcy, bctheta, alpha, gamma;
    float barcodeX, barcodeY;
    std::string barcodeOrientation, device_name;
    CSVReader csvreader;
    corobot_common::Pose msg;

	//std::vector<corobot_common::Pose> qrCodeList;
	corobot_common::Pose seenQRPose;
    
    BarcodeHandler(ros::Publisher & chatter_pub,std::string dev,std::string csvfile);
    void image_callback(zbar::Image & image);
    bool isLeft(std::string dev);
	bool checkIfNewQR(corobot_common::Pose qrPose);

};

#endif
