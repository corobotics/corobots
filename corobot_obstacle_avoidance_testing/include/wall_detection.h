#ifndef wall_detection_h
#define wall_detection_h
#include <iostream>

#include "ros/console.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "corobot_obstacle_avoidance_testing/Wall.h"
#include "geometry_msgs/Twist.h"
#include "corobot_common/Pose.h"



using namespace std;
using sensor_msgs::LaserScan;
using corobot_obstacle_avoidance_testing::Wall;
using geometry_msgs::Twist;
using corobot_common::Pose;

#define K_OMEGA (0.5/90.00)

#define K_TRANS (0.3/1.2)

#define SET_DIST 0.2

/**
 * Helper class for 2D rectangular coordinates
 */
class CartesianPoint{
public:
	float x,y;
	CartesianPoint(float _x,float _y){
		x = _x;
		y = _y;
	}
};

/**
 * Declaration of wall detector class
 */
class WallDetector {
public:

    int wallThreshold;

    int turnSwitchCount;
    bool lastTurnLeft;
    bool isDeadlock;
    bool isInit;
    float toFollow_r;
    float toFollow_t;

    ros::Publisher wallPublisher;
    ros::Publisher cmdVelPub;

	WallDetector(){
        turnSwitchCount = 0;
        lastTurnLeft = false;
        isDeadlock = false;
        isInit = true;
        wallThreshold = 45;
	}
	void testMethod(){ cout << "Hello!\n";}

    void scanCallback(sensor_msgs::LaserScan);

    Wall houghTransform(sensor_msgs::LaserScan);
};

#endif /* wall_detection_h */
