#ifndef obstable_avoidance_h
#define obstable_avoidance_h

#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose2D.h"

class ObstacleAvoider {
public:
    virtual Vector2 nav(const sensor_msgs::LaserScan::ConstPtr& scan);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    void poseCallback(const geometry_msgs::Pose2D::ConstPtr& pose);
    void waypointCallback(const geometry_msgs::Point::ConstPtr& waypoint);

protected:
    geometry_msgs::Pose2D::ConstPtr pose;
    queue<geometry_msgs::Point::ConstPtr> waypointQueue;
}

class APF : public ObstacleAvoider {
    virtual Vector2 nav(const sensor_msgs::LaserScan::ConstPtr& scan);
}

#endif /* obstable_avoidance_h */
