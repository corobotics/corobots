#!/usr/bin/env python
import roslib
import math

import rospy
import particle

from corobot_common.msg import Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from ekf import EKF
from corobot_qrcode_python import CSVReader
from corobot_common.srv import GetCoMap


def pf_initialize(pose):
    # Get pose information
    x_real = pose.x
    y_real = pose.y
    orientation = pose.theta
    covariance = tuple(pose.cov.flat)

    particle_count = 0
    particles = []
    # Initialize 500 objects of particle
    while particle_count < 500:
        particles[particle_count] = Particle(x_real, y_real, orientation, 0,
                                             covariance[0], covariance[4], covariance[8])
        particle_count += 1


def prediction(odom):
    odom_delta = ekf.get_odom_delta(odom)
    if odom_delta is not None:
        tuple_odom = odom_delta.T.tolist()[0]
    else:
        tuple_odom = tuple(0, 0, 0)
    delta_x = tuple_odom[0]
    delta_y = tuple_odom[1]
    delta_theta = tuple_odom[2]
    trv_dist = math.sqrt((delta_x ** 2) + (delta_y ** 2))

    particle_count = 0
    mu = 0
    trv_sigma = 0.05
    orient_sigma = (0.5 / 360) * math.pi * 2
    for each_particle in particles:
        each_particle.predict_update(trv_dist, delta_theta, 0, trv_sigma, orient_sigma)
        if each_particle.loc_check(map) is False:
            each_particle.probability = 0
        particle_count += 1


def update_model(scan):
    particle_count = 0
    while particle_count < len(particles):
        if particles[particle_count].probability == 0:
            particles.pop(particle_count)
        else:
            particles[particle_count].probability_update(scan, map)
            if particles[particle_count].probability <= 0.2:
                particles.pop(particle_count)
            else:
                particle_count += 1

    sorted_particles = sorted(particles, key=lambda particle: particle.probability, reverse=True)
    prtcle_idx = 0
    while len(particles) < 500:
        particles.append(sorted_particles[prtcle_idx])
        prtcle_idx += 1


def main():
    global pose_pub, ekf, particles, map
    rospy.wait_for_service('get_map')
    map_srv = rospy.ServiceProxy('get_map', GetCoMap)
    map = map_srv().map
    pose = Pose()
    roslib.load_manifest("corobot_localization")
    map_reader = CSVReader
    rospy.init_node("localization")
    ekf = EKF()
    pose_pub = rospy.Publisher("pose", Pose)
    # rospy.Subscriber("odom", Odometry, odom_callback)
    # rospy.Subscriber("qrcode_pose", Pose, qrcode_callback)
    rospy.Subscriber("qrcode_pose", Pose, pf_initialize)
    rospy.Subscriber("odom", Odometry, prediction)
    rospy.Subscriber("scan", LaserScan, update_model)
    rospy.spin()

    pf_initialize()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
