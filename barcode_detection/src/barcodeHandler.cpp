  #include "../include/barcodeHandler.h"

    BarcodeHandler::BarcodeHandler(ros::Publisher & chatter_pub) {

	test = chatter_pub;

    } 
    void BarcodeHandler::image_callback(Image & image) {

	
	for (SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol) {

	    for(int i=0;i<4;i++){
		point[i].x=symbol->get_location_x(i);
		point[i].y=symbol->get_location_y(i);
	    }
	
	// Read csv file 
	csvreader.init();
        csvreader.readFile();
        
	// Get data 
        istringstream ( csvreader.getX(symbol->get_data()) ) >> barcodeX;
	istringstream ( csvreader.getY(symbol->get_data()) ) >> barcodeY;
	barcodeOrientation=csvreader.getOrientation(symbol->get_data());

	// Close csv file 
	csvreader.close();
	
	// focal length( calculated before) and test distance 	
	float f = 270,D=25;

	//Length of pixels top left and bottom left
	lengthPixelL=abs (point[1].y-point[0].y);

	//Length of pixels top right and bottom right
	lengthPixelR=abs (point[3].y-point[2].y);

	//Calculating the distance from the barcode to camera
	distanceL=(f*D)/lengthPixelL;
	distanceR=(f*D)/lengthPixelR;

	cout<<"length of pixel:"<<lengthPixelL;
	
	//offsetDistance= ( (f*D) / abs( 800 -  ( ( point[0].x + point[1].x + point[2].x + point[3].x ) / 4 ) ) ) / 39.3701 ;
	offsetDistance= ( 5 * abs( 800 -  ( ( point[0].x + point[1].x + point[2].x + point[3].x ) / 4 ) ) / lengthPixelL ) / 39.3701;
	cout<<endl<<point[0].x<<" "<<point[1].x<<" "<<point[2].x<<" "<<point[3].x;
	cout<<endl<<"Offset="<<offsetDistance<<endl;

	squareDistanceL=pow(distanceL,2);
	squareDistanceR=pow(distanceR,2);

	//Calculating the angle from the barcode to camera
	angleR=acos ((squareDistanceR+25-squareDistanceL)/(2*distanceR*5)) * 180.0 / PI ;
	angleL=180 - (acos ((squareDistanceL+25-squareDistanceR)/(2*distanceL*5)) * 180.0 / PI);
	
	angleAvg= (angleR + angleL) / 2;

	// Calculate Average and convert to meters 
	distanceAvg= ((distanceL + distanceR) / 2) / 39.3701;
	cout << angleR <<" "<< angleL << endl;

	float realx, realy= 0.0;
	
	if(barcodeOrientation.compare("S")==0){
	// S orientation 
	realx= (barcodeX) + distanceAvg * cos (angleAvg+180);
	realy= (barcodeY) + distanceAvg * sin (angleAvg+180);
	}
	if(barcodeOrientation.compare("N")==0){
	// N orientation 
	realx= (barcodeX) + distanceAvg * cos (angleAvg);
	realy= (barcodeY) + distanceAvg * sin (angleAvg);
	}
	if(barcodeOrientation.compare("W")==0){
	// W orientation 
	realx= (barcodeX) + distanceAvg * cos (angleAvg+270);
	realy= (barcodeY) + distanceAvg * sin (angleAvg+270);
	}
	if(barcodeOrientation.compare("E")==0){
	// E orientation 
	realx= (barcodeX) + distanceAvg * cos (angleAvg+90);
	realy= (barcodeY) + distanceAvg * sin (angleAvg+90);	
	}
	
	//Publishing the msg
       std_msgs::String msg1;

	Pose msg;
	msg.x=realx;
	msg.y=realy;
	msg.theta=angleAvg;
	std::stringstream ss;
	ss <<distanceAvg<<" "<<angleAvg<<" Position: "<< realx <<" "<< realy;
	msg1.data = ss.str();
	ROS_INFO("%s", msg1.data.c_str());
	test.publish(msg);
	
	}

	
    }

