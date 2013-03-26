#ifndef obstable_avoidance_h
#define obstable_avoidance_h

/**
 * Generic obstacle avoidance things.
 */

#include <queue>

#include "ros/ros.h"
#include "corobot_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/LaserScan.h"

/** How close in meters to get to a waypoint before considered arrived. */
#define ARRIVED_DISTANCE 0.2

/**
 * Helper struct for 2D polar coordinates.
 */
typedef struct {
    /** Distance (radius) in meters. */
    float d;
    /** Angle in radians. */
    float a;
    /** Only positive distances are allowed by convention. */
    bool isValid() const;
} Polar;

/**
 * Abstract class to handle avoiding obstacles.
 */
class ObstacleAvoider {
public:

    /**
     * A queue of waypoints for the robot to navigate to.
     */
    std::queue<geometry_msgs::Point> waypointQueue;

    /**
     * A queue of waypoints that the robot has navigated to.
     * The waypoints are here until they are passed back along
     * waypoints_reached.
     */
    std::queue<geometry_msgs::Point> arrivedQueue;

    /**
     * Abstract method to provide a navigation vector based off a laser scan.
     *
     * @param scan  The laser scan to navigate with.
     * @returns     A movement vector in polar coordinates.
     */
    virtual Polar nav(sensor_msgs::LaserScan scan) = 0;

    /**
     * Update the known pose of the robot.
     *
     * @param pose  The new pose of the robot.
     */
    void updatePose(corobot_msgs::Pose pose);

    /**
     * Add a waypoint to the queue.
     *
     * @param waypoint  The new waypoint for the queue.
     */
    void addWaypoint(geometry_msgs::Point waypoint);

protected:

    /** The current pose of the robot. */
    corobot_msgs::Pose pose;

};

#endif /* obstable_avoidance_h */
