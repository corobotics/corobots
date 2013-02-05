#ifndef laser_localization_h
#define laser_localization_h

#include <vector>

#include "ros/ros.h"
#include "corobot_msgs/Pose.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"

#include "corobot.h"

#define PI 3.14159265

#define OCCUPANCY_THRESH 30

#define NUM_GUESSES 10000

// TODO: I have no idea what this value should be.
#define GUESS_ACCEPT 0.00001

/**
 * Represents a pose on the OccupancyGrid.
 */
typedef struct {
    int x;
    int y;
    /** Angle counter-clockwise from +x, in rads. */
    float a;
} GridPose;

/**
 * Represents a GridPose with an added probability field.
 */
typedef struct GridPoseP {
    int x;
    int y;
    float a;
    /** A probability (0 <= p <= 1). */
    float p;
    GridPoseP() {}
    GridPoseP(GridPose pose, float p) : x(pose.x), y(pose.y), a(pose.a), p(p) {}
} GridPoseP;

/**
 * Attempts to generate a Pose from a map and a LaserScan.
 */
class LaserLocalization {
public:
    LaserLocalization(nav_msgs::OccupancyGrid grid);
    void updateGrid(nav_msgs::OccupancyGrid grid);
    float findObstacle(int x1, int y2, float a);

private:
    nav_msgs::OccupancyGrid grid;
    int w;
    int h;

    GridPose randomPose();
    GridPoseP randomPoseP(sensor_msgs::LaserScan scan);
    int8_t gridLookup(int x, int y);
    int8_t gridLookup(GridPose pose);

    /**
     * A piecewise function with the following values:
     *
     *     P1 when x < -X1
     *     P2 when -X2 < x < X2
     *     P3 when X1 < x
     *     and line segments between -X1/-X2 and X2/X1.
     *
     * @param obsv_d    The observed distance; x.
     * @param map_d     The distance the map indicates; shifts the function.
     * @return          The probability of seeing obsv_d when you should see map_d.
     */
    double rangeProbability(float obsv_d, float map_d);
    double comparePoseToScan(GridPose pose, sensor_msgs::LaserScan scan);
    corobot::SimplePose calculateStats(std::vector<GridPoseP> poses, float* cov);
    corobot_msgs::Pose find(sensor_msgs::LaserScan scan);
};

#endif /* laser_localization_h */
