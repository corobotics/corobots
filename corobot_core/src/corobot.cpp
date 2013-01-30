#include <cmath>

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"

#include "corobot.h"

namespace corobot {

    double dist(double x, double y) {
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

    SimplePose coordTransform(SimplePose state, SimplePose offset) {
        SimplePose result;
        result.x = state.x * cos(offset.a) - state.y * sin(offset.a) + offset.x;
        result.y = state.x * sin(offset.a) + state.y * cos(offset.a) + offset.y;
        result.a = state.a + offset.a;
        return result;
    }

}
