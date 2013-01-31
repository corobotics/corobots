#ifndef corobot_h
#define corobot_h

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"

namespace corobot {

    /**
     * As simple as a pose gets.
     */
    typedef struct {
        double x;
        double y;
        double a;
    } SimplePose;

    /**
     * Calculates the distance from the origin to a point at (x, y).
     *
     * @param x     The x coordinate.
     * @param y     The y coordinate.
     * @return      The straight-line distance from (0, 0) to (x, y).
     */
    double dist(double x, double y);

    /**
     * @param q     A unit quaternion (x and y should be 0).
     * @return      The +z angle in radians.
     */
    double quaternionToRads(geometry_msgs::Quaternion q);

    /**
     * Helper function to convert a geometry_msgs::Pose into a SimplePose.
     *
     * @param p     The geometry_msgs::Pose object.
     * @return      A SimplePose.
     */
    SimplePose geomPoseToSimplePose(geometry_msgs::Pose p);

    /**
     * Transforms a vector from one reference frame to another.
     *
     * @param state     The original state in the first coordinate system.
     * @param offset    The origin of the second coord system in the first one.
     * @return          The state converted into the second coord system.
     */
    SimplePose coordTransform(SimplePose state, SimplePose offset);

}

#endif /* corobot_h */
