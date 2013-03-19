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

    geometry_msgs::Point coordTransform(geometry_msgs::Point point, corobot_msgs::Pose offset) {
        geometry_msgs::Point result;
        result.x = point.x * cos(offset.theta) - point.y * sin(offset.theta) + offset.x;
        result.y = point.x * sin(offset.theta) + point.y * cos(offset.theta) + offset.y;
        return result;
    }

    SimplePose coordTransform(SimplePose state, SimplePose offset) {
        SimplePose result;
        result.x = state.x * cos(offset.a) - state.y * sin(offset.a) + offset.x;
        result.y = state.x * sin(offset.a) + state.y * cos(offset.a) + offset.y;
        result.a = state.a + offset.a;
        return result;
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
