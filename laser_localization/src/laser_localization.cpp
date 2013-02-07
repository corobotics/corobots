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

GridPoseP LaserLocalization::makeGridPoseP(GridPose pose, LaserScan scan) {
    float p = comparePoseToScan(pose, scan);
    GridPoseP poseP(pose, p);
    return poseP;
}

GridPose LaserLocalization::randomPose() {
    GridPose pose;
    pose.x = rand() % w;
    pose.y = rand() % h;
    while (gridLookup(pose) > OCCUPANCY_THRESH) {
        pose.x = rand() % w;
        pose.y = rand() % h;
    }
    pose.a = rand() / (double)RAND_MAX * 2 * PI;
    return pose;
}

GridPoseP LaserLocalization::randomPoseP(LaserScan scan) {
    return makeGridPoseP(randomPose(), scan);
}

int8_t LaserLocalization::gridLookup(int x, int y) {
    return grid.data[y * w + x];
}

int8_t LaserLocalization::gridLookup(GridPose pose) {
    return gridLookup(pose.x, pose.y);
}

float LaserLocalization::findObstacle(GridPose pose) {

    // All the variables.
    int x1 = pose.x;
    int y1 = pose.y;
    int x2, y2, dx, dy, x, y;
    int d, incSide, incDiag, incX, incY;

    // Init line endpoints.
    x2 = x1 + (int)(10.0 * cos(pose.a));
    y2 = y1 + (int)(10.0 * sin(pose.a));

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
        for (x = x1; incX > 0 ? x <= x2 : x >= x2; x += incX) {
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
        for (y = y1; incY > 0 ? y <= y2 : y >= y2; y += incY) {
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

double LaserLocalization::rangeProbability(float obsv_d, float map_d) {
    const static float X1 = 0.5;
    const static float X2 = 0.1;
    const static float P1 = 0.25;
    const static float P2 = 0.75;
    const static float P3 = 0.1;
    const static float M1 = (P2 - P1) / (X1 - X2);
    const static float M2 = (P3 - P2) / (X2 - X1);
    float b;
    float d = obsv_d - map_d;
    if (d < -X1) {
        return P1;
    } else if (d < -X2) {
        b = P2 - M1 * (map_d - X2);
        return M1 * obsv_d + b;
    } else if (d < X2) {
        return P2;
    } else if (d < X1) {
        b = P2 - M2 * (map_d + X2);
        return M2 * obsv_d + b;
    } else {
        return P3;
    }
}

double LaserLocalization::comparePoseToScan(GridPose pose, LaserScan scan) {
    int n = (int)((scan.angle_max - scan.angle_min) / scan.angle_increment) + 1;
    pose.a += scan.angle_min;
    double p = 1.0;
    float d1, d2;
    for (int i = 0; i < n; i++) {
        // Assume independence and just multiply the probabilities.
        d1 = findObstacle(pose);
        d2 = scan.ranges[i];
        // TODO: Should we ignore unknown values? Or do something else?
        if (d1 >= 0 && d2 >= scan.range_min && d2 <= scan.range_max) {
            p *= rangeProbability(d1, d2);
        }
        pose.a += scan.angle_increment;
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

SimplePose LaserLocalization::calculateStats(std::vector<GridPoseP> poses, float* cov) {
    // The mean values.
    SimplePose m;
    float pTotal = 0.0;
    float pSquareTotal = 0.0;
    for (std::vector<GridPoseP>::iterator i = poses.begin(); i != poses.end(); i++) {
        pTotal += i->p;
        pSquareTotal += i->p * i->p;
    }
    const float W = pTotal / (pTotal * pTotal - pSquareTotal);
    for (std::vector<GridPoseP>::iterator i = poses.begin(); i != poses.end(); i++) {
        m.x += i->p / pTotal * i->x;
        m.y += i->p / pTotal * i->y;
        m.a += i->p / pTotal * i->a;
    }
    if (cov != NULL) {
        for (unsigned int i = 0; i < 9; i++) {
            cov[i] = 0.0;
        }
        for (std::vector<GridPoseP>::iterator i = poses.begin(); i != poses.end(); i++) {
            GridPoseP p = *i;
            cov[0] += p.p * (p.x - m.x) * (p.x - m.x);
            cov[1] += p.p * (p.x - m.x) * (p.y - m.y);
            cov[2] += p.p * (p.x - m.x) * (p.a - m.a);
            cov[3] += p.p * (p.y - m.y) * (p.x - m.x);
            cov[4] += p.p * (p.y - m.y) * (p.y - m.y);
            cov[5] += p.p * (p.y - m.y) * (p.a - m.a);
            cov[6] += p.p * (p.a - m.a) * (p.x - m.x);
            cov[7] += p.p * (p.a - m.a) * (p.y - m.y);
            cov[8] += p.p * (p.a - m.a) * (p.a - m.a);
        }
        for (unsigned int i = 0; i < 9; i++) {
            cov[i] *= W;
        }
    }
    return m;
}

std::vector<GridPoseP> LaserLocalization::generateSamples(const Pose& pose, const LaserScan& scan) {
    float step = SAMPLE_XY_GRANULARITY / grid.info.resolution;
    float thetaStep = 2.0 * PI / SAMPLE_THETA_COUNT;
    std::vector<GridPoseP> samples(SAMPLE_XY_COUNT * SAMPLE_XY_COUNT * SAMPLE_THETA_COUNT);
    GridPose p;
    float x = pose.x - step * SAMPLE_XY_COUNT / 2;
    for (int i = 0; i < SAMPLE_XY_COUNT; i++) {
        float y = pose.y - step * SAMPLE_XY_COUNT / 2;
        for (int j = 0; j < SAMPLE_XY_COUNT; j++) {
            float theta = pose.theta - thetaStep * SAMPLE_THETA_COUNT / 2;
            for(int k = 0; k < SAMPLE_THETA_COUNT; k++) {
                p.x = (int)x;
                p.y = (int)y;
                p.a = theta;
                samples.push_back(makeGridPoseP(p, scan));
                theta += thetaStep;
            }
            y += step;
        }
        x += step;
    }
    return samples;
}

Pose LaserLocalization::find(const Pose& pose, const LaserScan& scan) {
    std::vector<GridPoseP> samples = generateSamples(pose, scan);
    float cov[9];
    SimplePose sp = calculateStats(samples, cov);
    SimplePose offset = geomPoseToSimplePose(grid.info.origin);
    SimplePose transformedPose = coordTransform(sp, offset);
    covTransform(cov, offset);
    Pose result;
    result.header = scan.header;
    result.x = transformedPose.x;
    result.y = transformedPose.y;
    result.theta = transformedPose.a;
    for (int i = 0; i < 9; i++) {
        result.cov[i] = cov[i];
    }
    return result;
}

int main(int argc, char** argv) {
    return 0;
}
