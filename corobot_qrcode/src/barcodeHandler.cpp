#include "barcodeHandler.h"

BarcodeHandler::BarcodeHandler(ros::Publisher &chatter_pub) {
    publisher = chatter_pub;
}

void BarcodeHandler::image_callback(Image &image) {

    for (SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol) {

        for (int i = 0; i < 4; i++) {
            point[i].x = symbol->get_location_x(i);
            point[i].y = symbol->get_location_y(i);
        }

        // Read csv file
        csvreader.init();
        csvreader.readFile();

        // Get data
        istringstream(csvreader.getX(symbol->get_data())) >> barcodeX;
        istringstream(csvreader.getY(symbol->get_data())) >> barcodeY;
        barcodeOrientation = csvreader.getOrientation(symbol->get_data());

        // Close csv file
        csvreader.close();

        // focal length(calculated before) and test distance
        float f = 270.0, D = 25.0;

        // Length of pixels top left and bottom left
        lengthPixelL = abs(point[1].y - point[0].y);

        // Length of pixels top right and bottom right
        lengthPixelR = abs(point[3].y - point[2].y);

        // Calculate the distance from the barcode to camera
        distanceL = (f * D) / lengthPixelL;
        distanceR = (f * D) / lengthPixelR;

        barcodeXavg = (point[0].x + point[1].x + point[2].x + point[3].x) / 4;
        offsetDistance = -(5 * (800 - (barcodeXavg)) / lengthPixelL) / 39.3701;
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
        bctheta = ((3 * PI) / 2) - cbtheta;

        float realx, realy = 0.0;

        if (barcodeOrientation.compare("N") == 0) {
            realx = (barcodeX) + bcx;
            realy = (barcodeY) + bcy;
        } else if (barcodeOrientation.compare("S") == 0) {
            realx = (barcodeX) - bcx;
            realy = (barcodeY) - bcy;
            bctheta += PI;
        } else if (barcodeOrientation.compare("E") == 0) {
            realx = (barcodeY) - bcx;
            realy = (barcodeX) + bcy;
            bctheta += 3 * (PI / 2);
        } else if (barcodeOrientation.compare("W") == 0) {
            realx = (barcodeY) + bcx;
            realy = (barcodeX) - bcy;
            bctheta += PI / 2;
        }
        bctheta = ((3 * PI) / 2) - bctheta;

        cout << "cbx " << cbx << " " << "cby " << cby << " " << "cbo " << cbtheta << " " << "alpha " << alpha << " " << "gamma " << gamma << " " << "bcx " << bcx << " " << "bcy " << bcy << " " << "bco " << bctheta << endl;

        cout << realx << " " << realy << " " << bctheta << endl;

        // Publishing the msg
        msg.x = realx;
        msg.y = realy;
        msg.theta = angleAvg;
        for (int i = 0; i < 9; i++) {
            msg.cov[i] = 0;
        }

        msg.cov[0] = 0.05;
        msg.cov[3] = 0.05;
        msg.cov[8] = 0.1;
        publisher.publish(msg);

    }

}
