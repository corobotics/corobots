#ifndef laser_localization_h
#define laser_localization_h

#include "ros/ros.h"
#include "corobot_msgs/Pose.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"

#define PI 3.14159265

#define OCCUPANCY_THRESH 30

#define NUM_GUESSES 10000

typedef struct {
    int x;
    int y;
    float a;
} GridPose;

typedef struct GridPoseP {
    int x;
    int y;
    float a;
    float p;
    GridPoseP() {}
    GridPoseP(GridPose pose, float p) : x(pose.x), y(pose.y), a(pose.a), p(p) {}
} GridPoseP;

class LaserLocalization {
public:
    LaserLocalization(nav_msgs::OccupancyGrid grid);
    void updateGrid(nav_msgs::OccupancyGrid grid);

private:
    nav_msgs::OccupancyGrid grid;
    int w;
    int h;

    GridPose randomPose();
    int8_t gridLookup(int x, int y);
    int8_t gridLookup(GridPose pose);
    float findObstacle(int x1, int y2, float a);
    float comparePoseToScan(GridPose pose, sensor_msgs::LaserScan scan);
    corobot_msgs::Pose find(sensor_msgs::LaserScan scan);
};

#endif /* laser_localization_h */
