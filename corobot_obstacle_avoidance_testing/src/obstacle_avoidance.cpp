#include <iostream>

#include "ros/console.h"
#include "ros/ros.h"
#include "corobot_common/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "corobot_common/Goal.h"

#include "corobot.h"
#include "obstacle_avoidance.h"
#include "apf.h"
#include "wall_detection.h"

using namespace std;
using corobot::length;
using corobot_common::Pose;
using geometry_msgs::Point;
using geometry_msgs::Twist;
using sensor_msgs::LaserScan;

/**
 * {@inheritDoc}
 */
void ObstacleAvoider::updatePose(Pose newPose) {
    pose = newPose;
    bool clearedInHallway = false;
    if (!waypointQueue.empty()) {
        Point goal = waypointQueue.front();
        double dist = ARRIVED_INTER_DISTANCE;
        if (waypointQueue.size() == 1) 
    	   dist = ARRIVED_DISTANCE;
        stringstream ss;
        for (int i=0; i<waypointQVector.size(); i++){
            if(i != 0)
                ss << ",";
            ss << "[" << waypointQVector[i].x << "," << waypointQVector[i].y << "]";
        }
        std::string s = ss.str();
        ROS_INFO("%s",s.c_str());
        ROS_INFO("Previously cleared waypoint: %f,%f",goal.x,goal.y);
        if (waypointQueue.size() > 1){
            if (previousWaypoint.x < 0 && previousWaypoint.y < 0){
                previousWaypoint.x = newPose.x;
                previousWaypoint.y = newPose.y;
            }
            if (previousWaypoint.x!=-1 && previousWaypoint.y!=-1){
                if (abs(previousWaypoint.x - waypointQVector[0].x) < 2.0 && abs(waypointQVector[0].x - waypointQVector[1].x) < 2.0){
                    inOneLine = true;
                    float y1=waypointQVector[0].y,y2=waypointQVector[1].y,ygoal=newPose.y;
                    if (((y2-y1)>0 && (ygoal-y1)>0) || ((y2-y1)<0 && (ygoal-y1)<0))
                        clearedInHallway = true;
                }
                else if (abs(previousWaypoint.y - waypointQVector[0].y) < 2.0 && abs(waypointQVector[0].y - waypointQVector[1].y) < 2.0){
                    inOneLine = true;
                    float x1=waypointQVector[0].x,x2=waypointQVector[1].x,xgoal=newPose.x;
                    if (((x2-x1)>0 && (xgoal-x1)>0) || ((x2-x1)<0 && (xgoal-x1)<0))
                        clearedInHallway = true;
                }
                else
                    inOneLine = false;
                if (inOneLine)
                    ROS_INFO("Waypoints in a straight line...");
            }
        }
        while (!waypointQueue.empty() && (length(pose.x - goal.x, pose.y - goal.y) < dist || clearedInHallway)) {
        	ROS_INFO("Arrived at waypoint (%.2f, %.2f)", goal.x, goal.y);
            clearedInHallway = false;
            previousWaypoint.x = goal.x;
            previousWaypoint.y = goal.y;
        	waypointQueue.pop();
            waypointQVector.erase(waypointQVector.begin());
        	arrivedQueue.push(goal);
        	goal = waypointQueue.front();
        	if (waypointQueue.size() == 1) 
        	   dist = ARRIVED_DISTANCE;
        }
    }
}

/**
 * {@inheritDoc}
 */
void ObstacleAvoider::addWaypoint(Point waypoint) {
    waypointQueue.push(waypoint);
    waypointQVector.push_back(waypoint);
}

/**
 * The publisher for movement velocity commands.
 */
ros::Publisher cmdVelPub;

/**
 * The publisher for waypoints the robot has reached.
 */
ros::Publisher waypointsReachedPub;

/**
 * The publisher for waypoints the robot has given up on reaching.
 */
ros::Publisher waypointsFailedPub;

/**
 * The publisher for the goal when the robot is ready to try again
 */
ros::Publisher goalsNav;

/**
 * The obstacle avoider to use.
 */
ObstacleAvoider* oa;

/**
 * Callback for laserscan messages.
 */
void scanCallback(LaserScan scan) {
    Twist t;
    Polar p = oa->nav(scan);
    t.linear.x = p.d;
    t.angular.z = p.a;
    cmdVelPub.publish(t);
    while (!oa->failedQueue.empty()) {
        Point failed = oa->failedQueue.front();
        waypointsFailedPub.publish(failed);
        oa->failedQueue.pop();
        ROS_INFO("Waypoint failed: (%.2f, %.2f)", failed.x, failed.y);
    }
}

/**
 * Callback for pose messages.
 */
void poseCallback(Pose pose) {
    oa->updatePose(pose);
    while (!oa->arrivedQueue.empty()) {
        Point reached = oa->arrivedQueue.front();
        waypointsReachedPub.publish(reached);
        oa->arrivedQueue.pop();
        ROS_INFO("Waypoint reached: (%.2f, %.2f)", reached.x, reached.y);
    }
}


//callback when the robot sees a barcode
void stopRecovery(corobot_common::Goal topicMsg){
	//settings to set when the robot has recovered
        if((dynamic_cast<APF*>(oa))->inRecovery){
	    (dynamic_cast<APF*>(oa))->inRecovery = false;
	    (dynamic_cast<APF*>(oa))->prevWayPointQuelen = 0;
            //ROS_INFO("%.2f, %.2f", dynamic_cast<APF*>(oa)) -> goal.x, goal.y);
            ros::Duration(0.5).sleep();
	    goalsNav.publish(((dynamic_cast<APF*>(oa)) -> goal));	
	    // this should also set timeSinceLastWayPoint
        }
}

/**
 * Callback for new waypoint messages.
 */
void waypointCallback(Point waypoint) {
    oa->addWaypoint(waypoint);
    ROS_INFO("Waypoint added: (%.2f, %.2f)", waypoint.x, waypoint.y);
}
void goalCallback(Point waypoint) {
    queue<geometry_msgs::Point> empty;
    swap(empty, oa -> waypointQueue);
    (dynamic_cast<APF*>(oa))-> goal = waypoint;
    //clearng the queue by swapping out the old queue with an empty one
    std::queue<geometry_msgs::Point> emptyQ;
    std::swap( oa -> waypointQueue, emptyQ );
    oa -> waypointQVector.clear();
    //ROS_WARN("******* WayPoint cleared. Curr length: %d *******", waypointQueue.size());
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "obstacle_avoidance");
    ros::NodeHandle n;
    cmdVelPub = n.advertise<Twist>("cmd_vel", 1);
    waypointsReachedPub = n.advertise<Point>("waypoints_reached", 1000);
    waypointsFailedPub = n.advertise<Point>("waypoints_failed", 1000);
    goalsNav = n.advertise<Point>("goals_nav", 1);
    oa = new APF();
    ros::Subscriber scanSub = n.subscribe("scan", 1, scanCallback);
    ros::Subscriber poseSub = n.subscribe("pose", 1, poseCallback);
    ros::Subscriber waypointSub = n.subscribe("waypoints", 1000, waypointCallback);
	ros::Subscriber qrCountSubscriber = n.subscribe("ch_qrcodecount", 1, stopRecovery);
    ros::Subscriber goal = n.subscribe("goals", 1, goalCallback);
    ros::Subscriber goalNav = n.subscribe("goals_nav", 1, goalCallback);
    ros::spin();
    delete oa;
    return 0;
}
