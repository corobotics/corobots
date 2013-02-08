#include <iostream>
#include <zbar.h>
#include "ros/ros.h"
#include <sstream>
#include <cmath>
#include <string>
#include "std_msgs/String.h"

#define PI 3.14159265

using namespace std;
using namespace zbar;

class pt{
	public: 
	 int x,y;
};



class MyHandler:public Image::Handler {

    public:

    ros::Publisher test;
    pt point[4];
    int lengthPixelL,lengthPixelR;
    float distanceL, distanceR,squareDistanceL, squareDistanceR, angleR, angleL, angleAvg;
    

    MyHandler(ros::Publisher & chatter_pub) {

	test = chatter_pub;

    } 
    void image_callback(Image & image) {


	for (SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol) {

	    for(int i=0;i<4;i++){
		point[i].x=symbol->get_location_x(i);
		point[i].y=symbol->get_location_y(i);
	    }
	
	// focal length( calculated before) and test distance 	
	float f = 270,D=25;

	//Length of pixels top left and bottom left
	lengthPixelL=abs (point[1].y-point[0].y);

	//Length of pixels top right and bottom right
	lengthPixelR=abs (point[3].y-point[2].y);

	//Calculating the distance from the barcode to camera
	distanceL=(f*D)/lengthPixelL;
	distanceR=(f*D)/lengthPixelR;
	cout<<"Distance left"<<distanceL<<endl;
	cout<<"Distance right="<<distanceR<<endl;

	squareDistanceL=pow(distanceL,2);
	squareDistanceR=pow(distanceR,2);

	//Calculating the angle from the barcode to camera
	angleR=acos ((squareDistanceR+25-squareDistanceL)/(2*distanceR*5)) * 180.0 / PI ;
	angleL=180 - (acos ((squareDistanceL+25-squareDistanceR)/(2*distanceL*5)) * 180.0 / PI);
	angleAvg= (angleR + angleL)/2;

	//Publishing the msg
        std_msgs::String msg;
	std::stringstream ss;
	ss <<distanceL<<" "<<distanceR<<" "<<angleR<<" "<<angleL<<" "<<angleAvg;
	msg.data = ss.str();
	ROS_INFO("%s", msg.data.c_str());
	test.publish(msg);
	}
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise < std_msgs::String > ("chatter", 1000);

    // create and initialize a Processor
    const char *device = "/dev/video0";


    Processor proc;
    proc.request_size(1600,1200);
    proc.init(device,true);

    // configure the Processor
    proc.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);

    // setup a callback
    MyHandler my_handler(chatter_pub);
    proc.set_handler(my_handler);

    // enable the preview window
    proc.set_visible();
    proc.set_active();

    try {
	// keep scanning until user provides key/mouse input
	proc.user_wait();
    }
    catch(ClosedError & e) {
    }
    return (0);
}

