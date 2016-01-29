#include <wall_detection.h>
#include "ros/console.h"
#include "ros/ros.h"
#include <cmath>
#include <vector>
#include <limits>

#include <fstream>

using namespace std;
using sensor_msgs::LaserScan;

Wall WallDetector::houghTransform(LaserScan scan){
    int maxr,maxac=0,maxr2,maxac2=0;
    float maxtheta,maxtheta2;
    vector<CartesianPoint> points;
    float maxX=0,maxY=0;
    float minY=numeric_limits<float>::max();
    int thetaDivisions = 180;
    float pi = atan(1)*4.00;
    float thetaIncrement = pi/((float)(thetaDivisions));
    bool lefthalf;
    Wall oneWall;
    for(int i=0; i<scan.ranges.size();i++){
        float x,y;
        if(scan.ranges[i] < scan.range_min || scan.ranges[i] > scan.range_max)
            continue;
        float theta = scan.angle_min + (i*scan.angle_increment);
        x = scan.ranges[i]*cos(theta);
        y = scan.ranges[i]*sin(theta);
        maxX = (x>maxX)?(x):(maxX);
        maxY = (y>maxY)?(y):(maxY);
        minY = (y<minY)?(y):(minY);
        CartesianPoint point(x,y);
        points.push_back(point);
    }
    int imgHeight = (int)(ceil(maxY*100));
    int imgWidth = (int)(ceil(maxX*100));
    vector<vector<int> > accumulator(4*imgHeight,vector<int>(thetaDivisions));
    for(int i=0;i<4*imgHeight;i++){
        for(int j=0;j<thetaDivisions;j++){
            accumulator[i][j] = 0;
        }
    }
    for(int i=0;i<points.size();i++){
        float x,y;
        int actual_r;
        x = points[i].x;
        y = points[i].y;
        for(int j=0;j<thetaDivisions;j++){
            int r = (int)(100*((x*cos(j*thetaIncrement)) + (y*sin(j*thetaIncrement))));
            actual_r = r;
            r += 2*imgHeight;
            if (r>=4*imgHeight || r<0)
                continue;
            accumulator[r][j]++;
            if (accumulator[r][j] > maxac){
                maxac = accumulator[r][j];
                maxr = actual_r;
                maxtheta = ((float)(j))*thetaIncrement;
            }
        }
    }

    cout << "Max accumulator: " << maxac << "\n";

    for(int i=0;i<4*imgHeight;i++){
        for(int j=0;j<thetaDivisions;j++){
            accumulator[i][j] = 0;
        }
    }
    for(int i=0;i<points.size();i++){
        float x,y,actual_t,distance;
        int actual_r;
        x = points[i].x;
        y = points[i].y;
        distance = abs((100.0*x*cos(maxtheta)) + (100.0*y*sin(maxtheta)) - maxr);
        if (distance <= 25.00){
            //cout << "Not considering... " << x << "," << y << "; dist: " << distance << "\n";
            continue;
        }
        for(int j=0;j<thetaDivisions;j++){
            int r = (int)(100*((x*cos(j*thetaIncrement)) + (y*sin(j*thetaIncrement))));
            actual_r = r;
            actual_t = ((float)(j))*thetaIncrement;

            r += 2*imgHeight;
            if (r>=4*imgHeight || r<0)
                continue;
            accumulator[r][j]++;
            if (accumulator[r][j] > maxac2){
                maxac2 = accumulator[r][j];
                maxr2 = actual_r;
                maxtheta2 = actual_t;
            }
        }
    }

    if (maxr < 0 && maxr2 < 0){
        if (maxac < wallThreshold && maxac2 < wallThreshold){
            oneWall.is_wall_right = false;
        }
        else{
            oneWall.rright = (maxac>maxac2)?(maxr):(maxr2);
            oneWall.thetaright = (maxac>maxac2)?(maxtheta*(180.0/pi)):(maxtheta2*(180.0/pi));
            oneWall.conf_right = (maxac>maxac2)?(maxac):(maxac2);
            oneWall.is_wall_right = true;
        }
        oneWall.is_wall_left = false;
    }
    else if(maxr > 0 && maxr2 > 0){
        if (maxac < wallThreshold && maxac2 < wallThreshold){
            oneWall.is_wall_left = false;
        }
        else{
            oneWall.rleft = (maxac>maxac2)?(maxr):(maxr2);
            oneWall.thetaleft = (maxac>maxac2)?(maxtheta*(180.0/pi)):(maxtheta2*(180.0/pi));
            oneWall.conf_left = (maxac>maxac2)?(maxac):(maxac2);
            oneWall.is_wall_left = true;
        }
        oneWall.is_wall_right = false;
    }
    else{
        oneWall.rright = (maxr<0)?(maxr):(maxr2);
        oneWall.rleft = (maxr<0)?(maxr2):(maxr);

        oneWall.thetaright = (maxr<0)?(maxtheta*(180.0/pi)):(maxtheta2*(180.0/pi));
        oneWall.thetaleft = (maxr<0)?(maxtheta2*(180.0/pi)):(maxtheta*(180.0/pi));

        oneWall.conf_right = (maxr<0)?(maxac):(maxac2);
        oneWall.conf_left = (maxr<0)?(maxac2):(maxac);
        
        if (maxr < 0){
            oneWall.is_wall_right = (maxac<wallThreshold)?(false):(true);
            oneWall.is_wall_left = (maxac2<wallThreshold)?(false):(true);
        }
        else{
            oneWall.is_wall_right = (maxac2<wallThreshold)?(false):(true);
            oneWall.is_wall_left = (maxac<wallThreshold)?(false):(true);
        }
        
    }

    oneWall.tdiv = thetaDivisions;
    oneWall.height = 4*imgHeight;

    return oneWall;
}

/**
 * Callback for laserscan messages.
 */
void WallDetector::scanCallback(LaserScan scan) {
    cout << "In scan Callback\n";
    clock_t begin = clock();
	//cout << scan;
	cout << "here\n";
	Wall currWall = houghTransform(scan);

    wallPublisher.publish(currWall);

    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    cout << "Time taken: " << elapsed_secs << "\n";
    cout << "Right wall conf: " << currWall.conf_right << "\n";
    cout << "Left wall conf: " << currWall.conf_left << "\n";
    float theta = (currWall.conf_right > currWall.conf_left)?(currWall.thetaright):(currWall.thetaleft);
    float r = (currWall.conf_right > currWall.conf_left)?(currWall.rright):(currWall.rleft);
    r = abs(r)/100.00;
    bool isleft = (currWall.conf_right > currWall.conf_left)?(false):(true);

    float w,v;

    cout << "Theta is: " << theta << "\n";
    cout << "Distance is: " << r << "\n";
    Twist t;
    float pi = 4.00 * atan(1);
    // if (theta <= 85.00){
    //     t.angular.z = -0.1;
    // }
    // else if (theta >= 95.00){
    //     t.angular.z = 0.1;
    // }
    // else{
    //     t.linear.x = 0.1;
    // }

    // w = K_OMEGA * (theta - 90.00);
    // if (r > 1.2)
    //     v = 0.3;
    // else{
    //     v = max(0.05, K_TRANS * r);
    // }

    v = 0.4; 
    w = ((theta - 90.00) * v) / (r - SET_DIST);
    w = w * pi / 180.00;

    if (r <= 0.5)
        w = (w < 0)?(w-0.1):(w+0.1);

    cout << "Translation velocity: " << v << "\n";
    cout << "Rotational velocity: " << w << "\n";

    t.linear.x = v;
    t.angular.z = w;

    //cmdVelPub.publish(t);
    // cmdVelPub.publish(t);
    // cmdVelPub.publish(t);
}

void testWallSub(Wall wall){
    cout << "Got a Wall msg...\n";
    //cout << wall << "\n";
}

void poseCallback(Pose pose){
    cout << "Position is: " << pose << "\n";
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "wall_detection");
    ros::NodeHandle n;
    WallDetector* wd = new WallDetector();
    wd->wallPublisher = n.advertise<Wall>("wall",1000);
    wd->cmdVelPub = n.advertise<Twist>("mobile_base/commands/velocity",1);
    ros::Subscriber scanSub = n.subscribe("scan", 1, &WallDetector::scanCallback,wd);
    ros::Subscriber wallSub = n.subscribe("wall",1,testWallSub);
    //ros::Subscriber poseSub = n.subscribe("pose",1,poseCallback);
    ros::spin();
    delete wd;
    return 0;
}
