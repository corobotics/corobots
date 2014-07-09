#!/usr/bin/env python

import roslib; roslib.load_manifest("corobot_localization")
import rospy
import sys
import copy

from sensor_msgs.msg import LaserScan
from corobot_common.srv import GetCoMap
from corobot_common.msg import Pose
from corobot_common.srv import GetPixelOccupancyResponse
#from corobot_map import map
import math
from bresenhem import bresenhem

# Get static map once
rospy.wait_for_service('get_map')
map_srv = rospy.ServiceProxy('get_map', GetCoMap)
my_map = map_srv().map

# Initializing pose
pose = Pose() 

# PROXIMITY WIDTH AND LENGTH
proxWidth = 1000 # in X direction
proxHeight = 1000 # in y direction

# Which laser scans do we consider?
laser_samples_inc = 10 

# KINECT LASER SCAN RANGES AND INCREMENT
startTheta = -0.513185441494
endTheta = 0.49990695715
thetaIncrement = 0.00158295687288 * laser_samples_inc

probThresh = 0.44 # per good scan sample 

kinectOffset = -0.09 # distance of Kinect in front of robot center (negative for behind)

lockedOn = False

def get_distance(x1, y1, x2, y2):
    return math.sqrt(((x2 - x1)*(x2 - x1)) + ((y2 - y1)*(y2 - y1)))

def bres_condition(x, y):
    i = x + (my_map.info.height - y - 1) * my_map.info.width
    occ = my_map.data[i]
    if occ > 50:
        return True

def get_expected_scan(pose):
    """ Calculated the expected Laser Scan results
    using bresenhem algorithm """
    

    # I am doing all the calculations w.r.t the images pixels
    res = my_map.info.resolution
    ht = my_map.info.height

    # should convert robot pose into kinect pose (offset backward ~9 cm) first
    x1 = pose[0]/res
    y1 = ht - (pose[1]/res)

    theta = math.fmod(pose[2], (2*math.pi))
    # Now x1, y1 represent location on map.png
    #rospy.loginfo("Sampling pose: %f %f %f",pose[0],pose[1],pose[2])
    curRHTheta = theta + startTheta

    scan = []

    # note theta is in original (right-handed) coords
    # to follow the scan order, we will increment in this coord system
    # but then negate the theta (or 2pi-theta) to do the image testing
    while( curRHTheta <= theta + endTheta ):

        # here is the theta conversion into image theta
        curTheta = 2*math.pi - curRHTheta
        #curTheta = curRHTheta
        '''
        # Using y = mx + y-intercept
        y_int = y1 - (math.tan(curTheta) * x1)
        
        # Betwen 45 to 135.. looking North in map
        if curTheta > (math.pi / 4) and curTheta <= ((3 * math.pi) / 4):
            y2 = y1 - proxHeight
            if (y2 < 0): y2 = 0
            x2 = (y2 - y_int) / math.tan(curTheta)
        
        # Between 135 to 225.. looking West in map
        elif curTheta > ((3 * math.pi) / 4) and curTheta <= ((5 * math.pi) / 4):
            x2 = x1 - proxWidth
            if x2 < 0: x2 = 0
            y2 = (math.tan(curTheta)*x2) + y_int

        # Between 225 to 315.. looking South in map
        elif curTheta > ((5 * math.pi) / 4) and curTheta <= ((7 * math.pi) / 4):
            y2 = y1 + proxHeight
            if y2>=my_map.info.height: y2 = my_map.info.height - 1
            x2 = (y2 - y_int) / math.tan(curTheta)

        # Between 315 to 45.. looking East in map
        else:
            x2 = x1 + proxHeight
            if x2>=my_map.info.width: x2 = my_map.info.width - 1
            y2 = (math.tan(curTheta)*x2) + y_int
        '''
        # screw all that, let's try this instead:
        x2 = x1 + 1000*math.cos(curTheta)
        y2 = y1 + 1000*math.sin(curTheta)
        
        if x1 < 0: x1 = 0
        if x2 < 0: x2 = 0
        if y1 < 0: y1 = 0
        if y2 < 0: y2 = 0
        if x1 > my_map.info.width: x1 = my_map.info.width
        if x2 > my_map.info.width: x2 = my_map.info.width
        if y1 > my_map.info.height: y1 = my_map.info.height
        if y2 > my_map.info.height: y2 = my_map.info.height
    
        r = bresenhem(x1, y1, x2, y2, bres_condition)
        
        if not r[2]:
            scan.append(100)
        else:
            dist = get_distance(int(x1), int(y1), r[0], r[1])*my_map.info.resolution
            scan.append(dist)
            #rospy.loginfo("Scan %f hit wall at %d %d dist %f",curTheta,r[0],r[1],dist)

        curRHTheta = curRHTheta + thetaIncrement

    return scan

def get_sample_points(pose):
    dx = max(0.1,math.sqrt(pose.cov[0]))
    dy = max(0.1,math.sqrt(pose.cov[4]))
    dt = max(0.05,math.sqrt(pose.cov[8]))
    if lockedOn:
        sample_offsets = [(0, 0, 0), (-dx, 0, 0), (dx, 0, 0), (0, -dy, 0), (0, dy, 0), \
          (0, 0, -3*dt), (0, 0, -2*dt), (0, 0, -dt), (0, 0, dt), (0, 0, 2*dt), (0, 0, 3*dt)]
    else:
        sample_offsets = [(0, 0, 0), (-2*dx, 0, 0), (-dx, 0, 0), (dx, 0, 0), (2*dx, 0, 0), \
        (0, -2*dy, 0), (0, -dy, 0), (0, dy, 0), (0, 2*dy, 0), \
        (0,0,-4*dt),(0, 0, -3*dt), (0, 0, -2*dt), (0, 0, -dt), (0, 0, dt), (0, 0, 2*dt), (0, 0, 3*dt), (0,0,4*dt),\
        (-dx, -dy, 0), (-dx, dy, 0), (dx, dy, 0), (dx, -dy, 0), \
        (-dx, 0, -2*dt), (-dx, 0, 2*dt), (dx, 0, -2*dt), (dx, 0, 2*dt), \
        (0, -dy, -2*dt), (0, -dy, 2*dt), (0, dy, -2*dt), (0, dy, 2*dt)]

    samples = []
    #pose = [i[0] for i in mat_pose.tolist()]
    #samples.append(pose)

    for off in sample_offsets:
        tmp = [pose.x, pose.y, pose.theta]
        for i in range(3):
            tmp[i] += off[i]
        samples.append(tmp)

    '''
    x1 = pose[0] + 0.5
    x2 = pose[0] - 0.5
    y1 = pose[1] + 0.5
    y2 = pose[1] - 0.5
    theta1 = pose[2] + 0.2
    theta2 = pose[2] - 0.2

    tmp = copy.deepcopy(pose)
    tmp[0] = x1
    samples.append(tmp)
    
    tmp = copy.deepcopy(pose)
    tmp[0] = x2
    samples.append(tmp)

    tmp = copy.deepcopy(pose)
    tmp[1] = y1
    samples.append(tmp)

    tmp = copy.deepcopy(pose)
    tmp[1] = y2
    samples.append(tmp)

    tmp = copy.deepcopy(pose)
    tmp[2] = theta1
    samples.append(tmp)

    tmp = copy.deepcopy(pose)
    tmp[2] = theta2
    samples.append(tmp)
    '''
    return samples

def get_laser_probability(obs, exp):
    #rospy.loginfo("Obsv: %f Exp: %f",obs,exp)

    # probability on y axis
    p1 = 0.4 # obstacle closer than expected
    p2 = 0.6 # obstacle near expected
    p3 = 0.2 # obstacle farther than expected

    # differences on x axis
    d1 = 0.8
    d2 = 0.95
    d3 = 1.05
    d4 = 1.2

# WHAT IF obs == NaN??????????????????????????????????????????????????????????????????????????????
    if exp == 0.0:
        exp = 0.00000001
    
    ratio = float(obs) / float(exp)
    
    # Using a trapeziodal function. Calculating using (y2 - y1)/(x2 - x1) = (y3 - y1)/(x3 - x1)
    if ratio < d1:
        return p1
    elif ratio > d1 and ratio < d2:
        return (((p2 - p1)*(ratio - d1))/(d2 - d1)) + p1
    elif ratio > d2 and ratio < d3:
        return p2
    elif ratio > d3 and ratio < d4:
        return (((p3 - p2)*(ratio - d3))/(d4 - d3)) + p2
    else:
        return p3

def get_sample_probability(obs, exp):
    
    sample_probability = 1
    goodscans = 0
    for i in range(0, 640, laser_samples_inc):
        if math.isnan(obs[i]):
            continue
        goodscans += 1
        prob = get_laser_probability(obs[i], exp[int(i/laser_samples_inc)])
        sample_probability *= prob 
        #rospy.loginfo("Obs %f Exp %f Prob %f",obs[i],exp[int(i/laser_samples_inc)],prob)
        
    #rospy.loginfo("%d good scan points",goodscans)
    return (sample_probability, goodscans)

def laser_callback(scan):
    rospy.loginfo("laser callback, pose (%.4g, %.4g, %.4g)",pose.x,pose.y,pose.theta)
    global posepub
    global lockedOn

    # upon startup we won't have a place to bootstrap from, so ignore
    if pose.x < 0.1 and pose.y < 0.1:
        return
    
    currentPose = pose
    samplePoints = get_sample_points(pose)

    #samplePoints = [(pose.x, pose.y, pose.theta)]
    sampleProbability = []
    #rospy.loginfo("%s", samplePoints)

    sample = pose
    for sample in samplePoints:
        newLaser = get_expected_scan(sample)
        (currentProbability, goodscans) = get_sample_probability(scan.ranges, newLaser)
        sampleProbability.append(currentProbability)
        #rospy.loginfo("Prob: %g",currentProbability)
    
    sum_x = 0.0
    sum_y = 0.0
    sum_theta = 0.0
    count = 0.0

    for i in range(0, len(samplePoints)):
        sample = samplePoints[i]
        sum_x += sampleProbability[i] * sample[0]
        sum_y += sampleProbability[i] * sample[1]
        sum_theta += sampleProbability[i] * sample[2]
        count = count + sampleProbability[i]

    rospy.loginfo("Prob: %6.3g from %d scan points", count, goodscans)
    mean_pose = [sum_x/count, sum_y/count, sum_theta/count]
    rospy.loginfo("Mean pose: (%f, %f, %f)",mean_pose[0],mean_pose[1],mean_pose[2])

    # if we don't like any of the samples, don't say anything.
    # this is probably wrong, but may help when lost or when
    # there are too many obstacles about.
    if (count/len(samplePoints)) < (probThresh**goodscans) or goodscans <= 10:
        rospy.loginfo("Skipping laser estimate, thresh = %6.3g",probThresh**goodscans)
        lockedOn = False
        # we'll spit out a bogus pose to let the GUI know we're still alive
        posemsg = Pose()
        posemsg.x = -1
        posemsg.y = -1
        posemsg.theta = 0
        posemsg.cov = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        posepub.publish(posemsg)
        return

    lockedOn = True
    sum_cx = 0.0
    sum_cy = 0.0
    sum_ctheta = 0.0
    sum_xy = 0.0
    sum_xt = 0.0
    sum_yt = 0.0
    #cCount = len(samplePoints) - 1
    #cCount = 0
    sum_wt = 0.0
    sum_wt2 = 0.0
    for i in range(0, len(samplePoints)):
        sample = samplePoints[i]
        xx = (sample[0] - mean_pose[0])
        yy = (sample[1] - mean_pose[1])
        tt = (sample[2] - mean_pose[2])
        wt = sampleProbability[i]/count
        #rospy.loginfo("%s at P = %5.3g",sample,wt)

        sum_cx += xx * xx * wt
        sum_cy += yy * yy * wt
        sum_ctheta += tt * tt * wt
        sum_xy += xx * yy * wt
        sum_xt += xx * tt * wt
        sum_yt += yy * tt * wt
        sum_wt += wt
        sum_wt2 += wt*wt

    #cCount = sum_wt/(sum_wt*sum_wt - sum_wt2)
    cCount = 1#sum_wt

    # if we are near a discontinuity in the underlying distribution,
    # the covariance will be bogus (not really Gaussian).  So for now
    # we just don't report, but could consider sending mean pose
    # with a default covariance matrix.
    if sum_cx/cCount < 1e-6 or sum_cy/cCount < 1e-6 or sum_ctheta/cCount < 1e-6:
        rospy.loginfo("Modifying laser estimate, diag(cov) = %.3g, %.3g, %.3g",\
        sum_cx/cCount, sum_cy/cCount, sum_ctheta/cCount) 
        # Use a sloppy independent covariance here, I guess?
        covariance = [max(sum_cx/cCount, 1e-6), 0, 0, \
                      0, max(sum_cy/cCount, 1e-6), 0, \
                      0, 0, max(sum_ctheta/cCount, 1e-6)]
    else:
        covariance = [sum_cx/cCount, sum_xy/cCount, sum_xt/cCount, \
               sum_xy/cCount, sum_cy/cCount, sum_yt/cCount, \
               sum_xt/cCount, sum_yt/cCount, sum_ctheta/cCount]
    #rospy.loginfo("Covariance: %s", coveriance_pose)
    posemsg = Pose()
    # convert from Kinect pose back to robot center pose
    posemsg.x = mean_pose[0] - kinectOffset*math.cos(mean_pose[2])
    posemsg.y = mean_pose[1] - kinectOffset*math.sin(mean_pose[2])
    posemsg.theta = mean_pose[2]
    posemsg.cov = covariance
    posepub.publish(posemsg)

def pose_callback(inPose):
    global pose
    pose = inPose
    pose.x += kinectOffset*math.cos(pose.theta)
    pose.y += kinectOffset*math.sin(pose.theta)

def main():
    global posepub
    rospy.init_node("raj_test_something")
    # our code takes more than 1/30 sec to run, but that's OK.  Just make sure the
    # queue can only hold one scan so that we are processing most recent one.
    # apparently this affects all subscribers to scan(?) but should be OK (?)
    rospy.Subscriber("scan", LaserScan, laser_callback, queue_size=1) 
    rospy.Subscriber("pose", Pose, pose_callback)
    posepub = rospy.Publisher("laser_pose", Pose)
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

#19876524-b482-11e3-ae28-f4b7e2132acf
