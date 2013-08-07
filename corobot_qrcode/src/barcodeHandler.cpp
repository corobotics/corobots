#include <zbar.h>
#include "barcodeHandler.h"

using namespace std;
using namespace zbar;

BarcodeHandler::BarcodeHandler(ros::Publisher &chatter_pub,string dev,string csvfile) {
    publisher = chatter_pub;
    device_name = dev;    
    // Read csv file
    csvreader.init(csvfile);

	ros::NodeHandle nodeHandle;
	qrCodeCountPublisher = nodeHandle.advertise<corobot_common::Goal>("ch_qrcodecount", 1);
	qrCount = 0;
	seenQRPose.x = -1.0; seenQRPose.y = -1.0;
}

bool BarcodeHandler::isLeft(string dev){
    if(dev.compare("/dev/videoleft") == 0)
	    return true; 
    return false;                                                                                          
}  

void BarcodeHandler::image_callback(Image &image) {

    for (SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol) {

        for (int i = 0; i < 4; i++) {
            point[i].x = symbol->get_location_x(i);
            point[i].y = symbol->get_location_y(i);
        }

        // Get data
        istringstream(csvreader.getX(symbol->get_data())) >> barcodeX;
        istringstream(csvreader.getY(symbol->get_data())) >> barcodeY;
        barcodeOrientation = csvreader.getOrientation(symbol->get_data());

        // focal length(calculated before) and test distance
        float f = 275.0, D = 25.0;

        // Length of pixels top left and bottom left
        lengthPixelL = abs(point[1].y - point[0].y);

        // Length of pixels top right and bottom right
        lengthPixelR = abs(point[3].y - point[2].y);

        ROS_INFO_STREAM(lengthPixelL << " " << lengthPixelR);
        // Calculate the distance from the barcode to camera
        distanceL = (f * D) / lengthPixelL;
        distanceR = (f * D) / lengthPixelR;

        barcodeXavg = (point[0].x + point[1].x + point[2].x + point[3].x) / 4;
        offsetDistance = -(5 * (960 - (barcodeXavg)) / lengthPixelL) / 39.3701;
        squareDistanceL = pow(distanceL, 2);
        squareDistanceR = pow(distanceR, 2);

        //Calculating the angle from the barcode to camera
        angleR = acos((squareDistanceR + 25 - squareDistanceL) / (2 * distanceR * 5));
        angleL = (PI) - (acos((squareDistanceL + 25 - squareDistanceR) / (2 * distanceL * 5)));
        angleAvg = (angleR + angleL) / 2;

        // Calculate Average and convert to meters
        distanceAvg = ((distanceL + distanceR) / 2) / 39.3701;

        cbx = offsetDistance;
        cby = sqrt((distanceAvg * distanceAvg) - (offsetDistance * offsetDistance));
        cbtheta = angleAvg;

        alpha = acos(offsetDistance / distanceAvg);
        gamma = ((PI) - (alpha + angleAvg));

        bcx = distanceAvg * cos((PI / 2) - gamma);
        bcy = distanceAvg * sin((PI / 2) - gamma);
        bctheta = PI / 2 + cbtheta;

        float realx, realy = 0.0;
        ROS_INFO_STREAM("Barcode: " << symbol->get_data());
        ROS_INFO_STREAM("cbx " << cbx << " " << "cby " << cby << " " << "cbo " << cbtheta << " " << "alpha " << alpha << " " << "gamma " << gamma << " " << "bcx " << bcx << " " << "bcy " << bcy << " " << "bco " << bctheta);

        if (barcodeOrientation.compare("N") == 0) {
            realx = barcodeX + bcx;
            realy = barcodeY + bcy;
        } else if (barcodeOrientation.compare("S") == 0) {
            realx = barcodeX - bcx;
            realy = barcodeY - bcy;
            bctheta += PI;
        } else if (barcodeOrientation.compare("E") == 0) {
            realx = barcodeX + bcy;
            realy = barcodeY - bcx;
            bctheta += PI * 1.5;
        } else if (barcodeOrientation.compare("W") == 0) {
            realx = barcodeX - bcy;
            realy = barcodeY + bcx;
            bctheta += PI * 0.5;
        }

        if(!(isLeft(device_name))){
          bctheta+=PI;
          ROS_INFO_STREAM("Right Camera");
        }
        else{
          ROS_INFO_STREAM("Left Camera");
        }
	// Publishing the msg
        
        ROS_INFO_STREAM("realx " << realx << " " << "realy " << realy << " " << "bctheta " <<  bctheta);
        
        msg.x = realx;
        msg.y = realy;
        msg.theta = bctheta;
        for (int i = 0; i < 9; i++) {
            msg.cov[i] = 0;
        }

        msg.cov[0] = 0.05;
        msg.cov[4] = 0.05;      

        if(cbtheta > 1.3 && cbtheta < 1.8){
        msg.cov[8] = 0.1;
        }
        else{
        msg.cov[8] = 0.3;
        }

		checkIfNewQR(msg); // do the publisher.publish(msg); inside chechIfNewQR once it Qrcode counting works perfect
        publisher.publish(msg);

    }

}

bool BarcodeHandler::checkIfNewQR(corobot_common::Pose qrPose){
    /*for (std::vector<corobot_common::Pose>::iterator it = qrCodeList.begin() ; it != qrCodeList.end(); ++it){
        corobot_common::Pose itPose = *it;
        if(abs(qrPose.x - itPose.x) <= 1.5 && abs(qrPose.y - itPose.y) <= 1.5){ //if the point approx matches the points in the list
            return false;
        }
    }*/

	if(seenQRPose.x != -1.0)
		if(abs(qrPose.x - seenQRPose.x) <= 1.5 && abs(qrPose.y - seenQRPose.y) <= 1.5 && abs(qrPose.theta - seenQRPose.theta) <= 0.7) //if the point approx matches the points in the list
            return false;
        
	seenQRPose.x = qrPose.x; seenQRPose.y = qrPose.y; seenQRPose.theta = qrPose.theta;

	stringstream ss; corobot_common::Goal topicMsg;
	if(isLeft(device_name))
		ss << "L" << ++qrCount;
	else
		ss << "R" << ++qrCount;
	topicMsg.name = ss.str(); qrCodeCountPublisher.publish(topicMsg);
    
	return true;
}