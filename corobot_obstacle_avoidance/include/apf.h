#ifndef corobots_apf_h
#define corobots_apf_h

/**
 * Defines the artificial potential field (APF) obstacle avoidance algorithm.
 */

#include <list>

#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Point.h"

#include "obstacle_avoidance.h"
#include "ros/ros.h"
#include "corobot_common/Goal.h"
#include <sstream>

using geometry_msgs::Point;
using namespace std;

/** The distance at which to switch from conical to constant goal attraction. */
#define D_GOAL 0.2

/** The distance at which to start paying attention to obstacles. */
#define D_OBS 0.7

/** Goal gain (constant factor). */
#define K_GOAL 0.35

/** Obstacle gain (constant factor). */
#define K_OBS 0.6

/** Minimum rotational velocity. */
#define MIN_OMEGA 0.35

/** Force angle beyond which we will turn instead of go straight. */
#define ANGLE_WINDOW 0.28

/** Max allowed forward velocity. */
#define MAX_VEL 0.3

/** Num seconds after which obstacles leave the cache (unless seen again). */
#define OBS_CACHE_TIMEOUT 10

/** Threshold distance for matching an obstacle in the cache. */
#define OBS_MATCH_DIST 0.5

/**
 * Struct to represent an obstacle in the cache: Point and time last seen
 */
typedef struct {
  Point p;
  double lastT;
} CachedPoint;

/**
 * APF implementation of the ObstacleAvoider interface.
 */
class APF : public ObstacleAvoider {
 public:
    int prevWayPointQuelen;
    bool inRecovery;
    
    APF()
        {		
            ros::NodeHandle n;
            rawnavPublisher = n.advertise<corobot_common::Goal>("ch_rawnav", 1);
            obsPublisher = n.advertise<corobot_common::Goal>("ch_obstacle", 1);
            absGoalPublisher = n.advertise<corobot_common::Goal>("ch_absgoal", 1);
            netForcePublisher = n.advertise<corobot_common::Goal>("ch_netforce", 1);
            velCmdPublisher = n.advertise<corobot_common::Goal>("ch_velcmd", 1);
            recoveryPublisher = n.advertise<corobot_common::Goal>("ch_recovery", 1);
            
            cmdPrev.a = 0;
            cmdPrev.d = 0;
            timeSinceLastWayPoint = 0;
            prevWayPointQuelen = 0;
            inRecovery = false;
            
	}
    
    /**
     * {@inheritDoc}
     */
    virtual Polar nav(sensor_msgs::LaserScan scan);

protected:

    /** The last command given by nav(). */
    Polar cmdPrev;
	
    /** The previous pose of the robot is stored here*/
    corobot::SimplePose prevRobotPose;

    ros::Publisher rawnavPublisher, obsPublisher, absGoalPublisher,	netForcePublisher,
      velCmdPublisher, recoveryPublisher;
    
    /** The last time nav() produced a positive forward velocity. */
    double timeLastMoved, timeSinceLastWayPoint;

    /** Time of most recent scan, used for various timeouts. */
    double lastScanTime;

    std::vector<CachedPoint> activeObstacleList;
    
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

    double distanceFromRobot(corobot::SimplePose &sp);
    Polar* convertFromGlobalToRobotInPolar(Point &sp);
    corobot::SimplePose* convertRobotToGlobal(Polar &polarPoint);
    bool pushIfUnique(corobot::SimplePose *sp);
    double min(double a, double b);

    void recoveryCheck(const double &recov_time_now);
    void recoverRobot();
    Polar cmdTransform(Polar &cmdInitial);
    Point calcGoalForce(Point &goalWrtRobot);
    void updateNetForce(Point &netForce);
    void updateObstacleList(list<Polar>& objects);
    //void publishTopic1(stringstream *ss, ros::Publisher *pub);
    
    Polar doRecoveryNav(sensor_msgs::LaserScan &scan);
    
};

#endif /* corobots_apf_h */
