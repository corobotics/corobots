#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"

#include "laser_localization.h"

using corobot_msgs::Pose;
using nav_msgs::OccupancyGrid;
using sensor_msgs::LaserScan;

float dist(float x, float y) {
    return sqrt(x * x + y * y);
}

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

int8_t LaserLocalization::gridLookup(int x, int y) {
    return grid.data[y * w + x];
}

int8_t LaserLocalization::gridLookup(GridPose pose) {
    return gridLookup(pose.x, pose.y);
}

float LaserLocalization::findObstacle(int x1, int y1, float a) {

    // All the variables.
    int x2, y2, dx, dy, x, y;
    int d, incSide, incDiag, incX, incY;

    // Init line endpoints.
    x2 = x1 + (int)(10.0 * cos(a));
    y2 = y1 + (int)(10.0 * sin(a));

    // Used to find slope.
    dx = x2 - x1;
    dy = y2 - y1;

    // Handle negative slopes.
    if (dx < 0) {
        incX = -1;
        dx = -dx;
    } else {
        incX = 1;
    }
    if (dy < 0) {
        incY = -1;
        dy = -dy;
    } else {
        incY = 1;
    }

    // Two versions of the algorithm, one that goes along x and one along y.
    if (dx >= dy) {
        incSide = 2 * dy;
        incDiag = 2 * dy - 2 * dx;
        d = 2 * dy - dx;
        y = y1;
        for (x = x1; x <= x2; x += incX) {
            if (gridLookup(x, y) > OCCUPANCY_THRESH) {
                return dist(x - x1, y - y1);
            }
            if (d <= 0) {
                d += incSide;
            } else {
                d += incDiag;
                y += incY;
            }
        }
    } else {
        incSide = 2 * dx;
        incDiag = 2 * dx - 2 * dy;
        d = 2 * dx - dy;
        x = x1;
        for (y = y1; y <= y2; y += incY) {
            if (gridLookup(x, y) > OCCUPANCY_THRESH) {
                return dist(x - x1, y - y1);
            }
            if (d <= 0) {
                d += incSide;
            } else {
                d += incDiag;
                x += incX;
            }
        }
    }
    return -1;
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
