#ifndef corobots_ekf_h
#define corobots_ekf_h

/**
 * Implements the Extended Kalman Filter as described in Robotic Motion
 */
class EKF {
public:

    void update(geometry_msgs::PoseWithCovarianceStamped pose);
    void predict();
    mat33 R();

private:
    # The time between 
    const double dt;
    vec3 x;
    mat33 P;
    vec3 xp;
    mat33 PP;
    double k;
}

#endif /* corobots_ekf_h */
