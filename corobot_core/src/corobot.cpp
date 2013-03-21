#include <cmath>

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"

#include "corobot.h"

namespace corobot {

    double length(double x, double y) {
        return sqrt(x * x + y * y);
    }

    double quaternionToRads(geometry_msgs::Quaternion q) {
        return atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z);
    }

    SimplePose geomPoseToSimplePose(geometry_msgs::Pose pose) {
        SimplePose simplePose;
        simplePose.x = pose.position.x;
        simplePose.y = pose.position.y;
        simplePose.a = quaternionToRads(pose.orientation);
        return simplePose;
    }

    SimplePose coordTransform(SimplePose aPose, SimplePose aOrigin) {
        SimplePose bPose;
        bPose.x = aPose.x * cos(aOrigin.a) - aPose.y * sin(aOrigin.a) + aOrigin.x;
        bPose.y = aPose.x * sin(aOrigin.a) + aPose.y * cos(aOrigin.a) + aOrigin.y;
        bPose.a = aPose.a + aOrigin.a;
        return bPose;
    }

    geometry_msgs::Point rCoordTransform(geometry_msgs::Point aPoint, corobot_msgs::Pose bOrigin) {
        geometry_msgs::Point bPoint;
        float dx = aPoint.x - bOrigin.x;
        float dy = aPoint.y - bOrigin.y;
        float theta = -bOrigin.theta;
        bPoint.x = dx * cos(theta) - dy * sin(theta);
        bPoint.y = dx * sin(theta) + dy * cos(theta);
        return bPoint;
    }

    void matProd(float* a, float* b, float* c, int n, int m, int p) {
        for (int i = 0; i < n * p; i++) {
            c[i] = 0;
        }
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < p; j++) {
                for (int k = 0; k < m; k++) {
                    c[i * p + j] += a[i * m + k] * b[k * p + j];
                }
            }
        }
    }

    void covTransform(float* cov, SimplePose offset) {
        float c[9];
        float sa = sin(offset.a);
        float ca = cos(offset.a);
        float rotation[9] = {
            ca, -sa, 0,
            sa,  ca, 0,
             0,   0, 1};
        float rotationTranspose[9] = {
             ca, sa, 0,
            -sa, ca, 0,
              0,  0, 1};
        matProd(rotationTranspose, cov, c, 3, 3, 3);
        matProd(c, rotation, cov, 3, 3, 3);
    }

}
