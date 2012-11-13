#ifndef obstable_avoidance_h
#define obstable_avoidance_h

#include <queue>

#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose2D.h"
#include "sensor_msgs/LaserScan.h"
#include "turtlebot_node/Turtle.h"

class ObstacleAvoider {
public:
    virtual turtlebot_node::Turtle nav(const sensor_msgs::LaserScan::ConstPtr& scan) = 0;
    void updatePose(const geometry_msgs::Pose2D::ConstPtr& pose);
    void addWaypoint(const geometry_msgs::Point::ConstPtr& waypoint);

protected:
    geometry_msgs::Pose2D pose;
    std::queue<geometry_msgs::Point> waypointQueue;
};

class APF : public ObstacleAvoider {
public:
    virtual turtlebot_node::Turtle nav(const sensor_msgs::LaserScan::ConstPtr& scan);
};

#endif /* obstable_avoidance_h */
