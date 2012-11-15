#ifndef obstable_avoidance_h
#define obstable_avoidance_h

#include <queue>

#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose2D.h"
#include "sensor_msgs/LaserScan.h"

class ObstacleAvoider {
public:
    virtual geometry_msgs::Point nav(sensor_msgs::LaserScan scan) = 0;
    void updatePose(geometry_msgs::Pose2D pose);
    void addWaypoint(geometry_msgs::Point waypoint);

protected:
    geometry_msgs::Pose2D pose;
    std::queue<geometry_msgs::Point> waypointQueue;
};

#endif /* obstable_avoidance_h */
