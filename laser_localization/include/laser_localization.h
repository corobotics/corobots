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

#define SAMPLE_XY_COUNT 100
#define SAMPLE_THETA_COUNT 20
#define SAMPLE_XY_GRANULARITY 0.02

/**
 * Represents a pose on the OccupancyGrid.
 */
typedef struct {
    /** x coordinate in the map reference frame. */
    int x;
    /** y coordinate in the map reference frame. */
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

    /**
     * @param grid  The map data to work with.
     * @return      A new LaserLocalization object.
     */
    LaserLocalization(nav_msgs::OccupancyGrid grid);

    /**
     * @return      Whether a scan has been received or not.
     */
    bool hasScan();

    /**
     * @param scan  A new laser scan.
     */
    void updateScan(sensor_msgs::LaserScan scan);

    /**
     * @param grid  An updated grid.
     */
    void updateGrid(nav_msgs::OccupancyGrid grid);

    /**
     * Find the next obstacle on the map in a line from the given pose.
     *
     * @param pose  The x, y coordinates and angle to trace a line out from.
     * @return      The distance to an obstacle.
     */
    float findObstacle(GridPose pose);

    /**
     * Try to place the laser scan in the area near the given pose.
     *
     * @param pose  The estimated pose of the robot.
     * @param scan  A laser scan to place.
     * @return      A best-guess of where the laser scan goes in the area around
     *              the pose.
     */
    corobot_msgs::Pose find(corobot_msgs::Pose pose);

private:

    /** The most recent scan. */
    sensor_msgs::LaserScan scan;

    /** The map data. */
    nav_msgs::OccupancyGrid grid;

    /** The width of occupancy grid. */
    int w;

    /** The height of the occupancy grid. */
    int h;

    /** Sentinel value to indicate that a scan has been received. */
    bool _hasScan;

    /**
     * Use a laser scan to calculate the probability of a GridPose.
     * This is done by simulating what a robot at the given pose would
     * actually see and comparing it to the actual laser scan.
     *
     * @param pose  The pose to calculate the probability of.
     * @param scan  The sensor input.
     * @return      A GridPoseP object with the calculated p value.
     */
    GridPoseP makeGridPoseP(GridPose pose);

    /**
     * Generate a random pose on the grid.
     *
     * @return  A random pose.
     */
    GridPose randomPose();

    /**
     * Generates a random pose and calculates its probability.
     * This function is equal to makeGridPoseP(randomPose(), scan).
     *
     * @param scan  The laser scan to use.
     * @return      A random pose with corresponding probability.
     */
    GridPoseP randomPoseP();

    /**
     * Retrieve the value for a specific point in the map reference frame.
     *
     * @param x     The x coordinate.
     * @param y     The y coordinate.
     * @return      The occupancy likelihood for this cell; 0-100.
     */
    int8_t gridLookup(int x, int y);

    /**
     * Convenience version of the above that takes a GridPose.
     */
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

    /**
     * Computes the probability of being at a given pose given a laser scan.
     *
     * @param pose  A hypothetical pose of the robot.
     * @param scan  A real laser scan from the robot.
     * @return      The probability of pose given scan.
     */
    double comparePoseToScan(GridPose pose);

    /**
     * Calculate the mean vector and covariance matrix of a list of poses.
     *
     * @param poses     The list of poses
     * @param cov       Optional output array for the covariance matrix.
     * @return          The mean pose of all poses.
     */
    corobot::SimplePose calculateStats(std::vector<GridPoseP> poses, float* cov);

    /**
     * Generate a grid of samples around a given pose.
     *
     * @param pose  The pose to generate samples around.
     * @param scan  Used to calculate the probability of each sample.
     * @return      A list of GridPoseP objects.
     */
    std::vector<GridPoseP> generateSamples(corobot_msgs::Pose pose);

};

#endif /* laser_localization_h */
