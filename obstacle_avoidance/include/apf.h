#ifndef corobots_apf_h
#define corobots_apf_h

/**
 * Defines the artificial potential field (APF) obstacle avoidance algorithm.
 */

#include <list>

#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Point.h"

#include "obstacle_avoidance.h"

/**
 * Helper struct for 2D polar coordinates.
 */
typedef struct {
    /** Distance (radius) in meters. */
    float d;
    /** Angle in radians. */
    float a;
    /** Only positive distances are allowed by convention. */
    bool isValid() const;
} Polar;

/**
 * Interface for a "force calculation", which is used to determine the
 * relationship between the distance of an object and its effect on the APF.
 */
class ForceCalc {
public:
    /**
     * Performs a force calculation.
     *
     * @param dist  The distance of the object.
     * @returns     A scalar for the weight of this object.
     */
    virtual float calc(const float& dist) = 0;
};

/**
 * Implementation of ForceCalc that represents raising the distance to an
 * inverse power.
 */
class InversePowerForce : public ForceCalc {
public:
    /**
     * @param exp   The exponent of the inverse power (positive number).
     */
    InversePowerForce(float exp) : exp(exp) {};

    /**
     * {@inheritDoc}
     */
    virtual float calc(const float& dist);

private:
    /**
     * The exponent to use in the force calc.
     */
    float exp;
};

/**
 * APF implementation of the ObstacleAvoider interface.
 */
class APF : public ObstacleAvoider {
public:
    /**
     * @param ko    Obstacle constant in the APF algorithm.
     * @param kg    Goal constant in the APF algorithm.
     */
    APF(const float& ko, const float& kg);

    /**
     * @param ko    Obstacle constant in the APF algorithm.
     * @param kg    Goal constant in the APF algorithm.
     * @param distForce A custom force calculator.
     */
    APF(const float& ko, const float& kg, ForceCalc* distForce);

    /**
     * {@inheritDoc}
     */
    virtual geometry_msgs::Point nav(sensor_msgs::LaserScan scan);

private:
    /**
     * The object constant for this APF.
     */
    float ko;

    /**
     * The goal constant for this APF.
     */
    float kg;

    /**
     * The force calculation to use for this APF.
     */
    ForceCalc* distForce;

    /**
     * Converts a laser scan to a list of polar coordinates.
     *
     * @param scan  The laser scan to convert.
     * @returns     The list of polar coords.
     */
    std::list<Polar> scanToList(sensor_msgs::LaserScan scan);

    /**
     * Finds the local minima (by distance) in a list of polar coords.
     * It is assumed the coords are sorted by angle.
     *
     * @param scan  The laser scan to use.
     * @returns     A list of minima in polar coordinates.
     */
    std::list<Polar> findLocalMinima(std::list<Polar> points);

    /**
     * Finds "objects" in a list of polar coords. An object is defined as
     * a series of points whose distance doesn't vary by more than OBJ_DIST
     * from point to point. The returned coord for the object is the minimal
     * point in the object.
     *
     * @param points    The list of points to start with.
     * @returns         The nearest point of each object.
     */
    std::list<Polar> findObjects(std::list<Polar> points);
};

#endif /* corobots_apf_h */
