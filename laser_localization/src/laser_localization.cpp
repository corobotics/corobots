#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"

#include "laser_localization.h"

using corobot_msgs::Pose;
using nav_msgs::OccupancyGrid;
using sensor_msgs::LaserScan;

LaserLocalization::LaserLocalization(OccupancyGrid grid) {
    updateGrid(grid);
}

void LaserLocalization::updateGrid(OccupancyGrid grid_) {
    grid = grid_;
    w = grid.info.width;
    h = grid.info.height;
}

GridPose LaserLocalization::randomPose() {
    GridPose pose;
    pose.x = rand() % w;
    pose.y = rand() % h;
    while (gridLookup(pose) > 20) {
        pose.x = rand() % w;
        pose.y = rand() % h;
    }
    pose.a = rand() / (double)RAND_MAX * 2 * PI;
    return pose;
}

int8_t LaserLocalization::gridLookup(GridPose pose) {
    return grid.data[pose.y * w + pose.x];
}

LaserScan LaserLocalization::poseToScan(GridPose pose) {
    LaserScan scan;
    return scan;
}

float LaserLocalization::compareScans(LaserScan s1, LaserScan s2) {
    return 0.0;
}

Pose LaserLocalization::find(LaserScan scan) {
    Pose pose;
    return pose;
}

int main(int argc, char** argv) {
    return 0;
}
