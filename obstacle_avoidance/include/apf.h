#ifndef corobots_apf_h
#define corobots_apf_h

#include <list>

#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Point.h"

#include "obstacle_avoidance.h"

typedef struct {
    float d;
    float a;
} Polar;

class ForceCalc {
public:
    virtual float calc(const float& dist) = 0;
};

class InversePowerForce : public ForceCalc {
public:
    InversePowerForce(float exp) : exp(exp) {};
    virtual float calc(const float& dist);
private:
    float exp;
};

class APF : public ObstacleAvoider {
public:
    APF(const float& ko, const float& kg);
    APF(const float& ko, const float& kg, ForceCalc* distForce);
    virtual geometry_msgs::Point nav(sensor_msgs::LaserScan scan);
private:
    float ko;
    float kg;
    ForceCalc* distForce;
    std::list<Polar> findLocalMinima(sensor_msgs::LaserScan scan);
};

#endif /* corobots_apf_h */
