#ifndef corobots_apf_h
#define corobots_apf_h

#include "sensor_msgs/LaserScan.h"
#include "turtlebot_node/Turtle.h"

#include "obstacle_avoidance.h"

class APF : public ObstacleAvoider {
public:
    virtual turtlebot_node::Turtle nav(sensor_msgs::LaserScan scan);
};

#endif /* corobots_apf_h */
