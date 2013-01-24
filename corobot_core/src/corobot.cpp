#include <cmath>

#include "corobot.h"

namespace corobot {
    double dist(double x, double y) {
        return sqrt(x * x + y * y);
    }
}
