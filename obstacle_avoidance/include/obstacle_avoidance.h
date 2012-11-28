#ifndef obstable_avoidance_h
#define obstable_avoidance_h

#include <queue>

#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose2D.h"
#include "sensor_msgs/LaserScan.h"

/**
 * Abstract class to handle avoiding obstacles.
 */
class ObstacleAvoider {
public:
    /**
     * Abstract method to provide a navigation vector based off a laser scan.
     *
     * @param scan  The laser scan to navigate with.
     * @returns     A 2D movement vector.
     */
    virtual geometry_msgs::Point nav(sensor_msgs::LaserScan scan) = 0;

    /**
     * Update the known pose of the robot.
     *
     * @param pose  The new pose of the robot.
     */
    void updatePose(geometry_msgs::Pose2D pose);

    /**
     * Add a waypoint to the queue.
     *
     * @param waypoint  The new waypoint for the queue.
     */
    void addWaypoint(geometry_msgs::Point waypoint);

protected:
    /**
     * The current pose of the robot.
     */
    geometry_msgs::Pose2D pose;

    /**
     * A queue of waypoints for the robot to navigate to.
     */
    std::queue<geometry_msgs::Point> waypointQueue;
};

#endif /* obstable_avoidance_h */
