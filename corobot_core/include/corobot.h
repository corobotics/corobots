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
     * Calculates the length of the vector <x, y>.
     *
     * @param x     The x coordinate.
     * @param y     The y coordinate.
     * @return      The straight-line distance from (0, 0) to (x, y).
     */
    double length(double x, double y);

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

    /**
     * Perform a matrix multiplication with matrices stored as flat arrays.
     *
     * @param a     An n*m length array representing the first matrix.
     * @param b     An m*p length array representing the second matrix.
     * @param c     An n*p length array to store the result in.
     * @param n     Number of rows in A.
     * @param m     Number of cols in A and rows in B.
     * @param p     Number of cols in B.
     */
    void matProd(float* a, float* b, float* c, int n, int m, int p);

    /**
     * Transform a covariance matrix to a different coordinate frame.
     *
     * @param cov   The covariance matrix in and out.
     * @param offset    The location of the origin of the new reference frame.
     */
    void covTransform(float* cov, SimplePose offset);

}

#endif /* corobot_h */
