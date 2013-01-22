#ifndef laser_localization_h
#define laser_localization_h

#include "ros/ros.h"
#include "corobot_msgs/Pose.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"

#define PI 3.14159265

typedef struct {
    int x;
    int y;
    double a;
} GridPose;

class LaserLocalization {
public:
    LaserLocalization(nav_msgs::OccupancyGrid grid);
    void updateGrid(nav_msgs::OccupancyGrid grid);

private:
    nav_msgs::OccupancyGrid grid;
    int w;
    int h;

    GridPose randomPose();
    int8_t gridLookup(GridPose pose);
    sensor_msgs::LaserScan poseToScan(GridPose pose);
    float compareScans(sensor_msgs::LaserScan s1, sensor_msgs::LaserScan s2);
    corobot_msgs::Pose find(sensor_msgs::LaserScan scan);
};

#endif /* laser_localization_h */
