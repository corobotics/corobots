#include <algorithm>
#include <vector>

#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"

#include "corobot.h"
#include "laser_localization.h"

using corobot_msgs::Pose;
using nav_msgs::OccupancyGrid;
using sensor_msgs::LaserScan;

using namespace corobot;

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

GridPoseP LaserLocalization::randomPoseP(LaserScan scan) {
    GridPose guess = randomPose();
    float p = comparePoseToScan(guess, scan);
    GridPoseP guessP(guess, p);
    return guessP;
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
    return -1.0;
}

float rangeProbability(float d1, float d2) {
    float d = d1 - d2;
    if (d1 < -0.1) {

    }
    return d;
}

float LaserLocalization::comparePoseToScan(GridPose pose, LaserScan scan) {
    int n = (int)((scan.angle_max - scan.angle_min) / scan.angle_increment) + 1;
    float a = scan.angle_min;
    float p = 1.0;
    float d1, d2;
    for (int i = 0; i < n; i++) {
        // Assume independence and just multiply the probabilities.
        d1 = findObstacle(pose.x, pose.y, pose.a + a);
        d2 = scan.ranges[i];
        // TODO: Should we ignore unknown values? Or do something else?
        if (d1 >= 0 && d2 >= scan.range_min && d2 <= scan.range_max) {
            p *= rangeProbability(d1, d2);
        }
        a += scan.angle_increment;
    }
    return p;
}

int gridPoseCmp(GridPoseP p1, GridPoseP p2) {
    if (p1.p > p2.p) {
        return -1;
    } else if (p1.p == p2.p) {
        return 0;
    } else {
        return 1;
    }
}

Pose LaserLocalization::find(LaserScan scan) {
    std::vector<GridPoseP> guesses(NUM_GUESSES);
    for (int i = 0; i < NUM_GUESSES; i++) {
        guesses[i] = randomPoseP(scan);
    }
    std::sort(guesses.begin(), guesses.end(), gridPoseCmp);
    while (guesses[0].p < GUESS_ACCEPT) {
        for (int i = NUM_GUESSES / 2; i < NUM_GUESSES; i++) {
            guesses[i] = randomPoseP(scan);
        }
        std::sort(guesses.begin(), guesses.end(), gridPoseCmp);
    }
    SimplePose sp = {guesses[0].x, guesses[0].y, guesses[0].a};
    SimplePose transformedPose = coordTransform(sp, geomPoseToSimplePose(grid.info.origin));
    Pose result;
    result.header = scan.header;
    result.x = transformedPose.x;
    result.y = transformedPose.y;
    result.theta = transformedPose.a;
    return result;
}

int main(int argc, char** argv) {
    return 0;
}
