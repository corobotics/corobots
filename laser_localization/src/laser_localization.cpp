#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"

#include "laser_localization.h"

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
    while (grid.data[pose.y * w + pose.x] > 20) {
        pose.x = rand() % w;
        pose.y = rand() % h;
    }
    pose.a = rand() / (double)RAND_MAX * PI;
    return pose;
}

int8_t gridLookup(GridPose pose) {
    return 0;
}

LaserScan poseToScan(GridPose pose) {
    LaserScan scan;
    return scan;
}
