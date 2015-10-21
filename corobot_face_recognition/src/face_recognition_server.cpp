#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <corobot_face_recognition/FaceRecognitionAction.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <vector>
#include <map>
#include <utility>

#include "face_detector.cpp"
#include "face_rec.cpp"
#include "face_img_capturer.cpp"

class FaceRecognition
{
protected:
  ros::NodeHandle _nh, _ph;
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::SimpleActionServer<corobot_face_recognition::FaceRecognitionAction> _as; 
  std::string _action_name;
  // create messages that are used to published feedback/result
  corobot_face_recognition::FaceRecognitionFeedback _feedback;
  corobot_face_recognition::FaceRecognitionResult _result;
  image_transport::ImageTransport _it;
  image_transport::Subscriber _image_sub;
  int _goal_id;
  std::string _goal_argument;
  FaceDetector _fd;
  FaceRec* _fr;
  FaceImgCapturer* _fc;
  std::string _recognitionAlgo;

public:

  FaceRecognition(std::string name) :
    _as(_nh, name, boost::bind(&FaceRecognition::executeCB, this, _1), false),
    _it(_nh),
    _ph("~"),
    _action_name(name),
    _fd(),
    _fr(NULL),
    _fc(NULL)
  {
    _as.start();

    // Subscrive to input video feed and publish output video feed
    // _image_sub = _it.subscribe("/camera/image_raw", 1, &FaceRecognition::imageCb, this);
    // _image_sub = _it.subscribe("/usb_cam/image_raw", 1, &FaceRecognition::imageCb, this);
    _ph.param<std::string>("algorithm", _recognitionAlgo , "lbph");
    cv::namedWindow("testing...");
  }

  ~FaceRecognition(void)
  {
  }

  void executeCB(const corobot_face_recognition::FaceRecognitionGoalConstPtr &goal)
  {
    if (_as.isPreemptRequested() || !ros::ok()) {
      ROS_INFO("%s: Preempted", _action_name.c_str());
      // set the action state to preempted
      _as.setPreempted();
      // success = false;
      // break;
    }
    
    // helper variables
    ros::Rate r(1);
    bool success = true;

    _goal_id = goal->order_id;
    _goal_argument = goal->order_argument;

    _feedback.order_id = _goal_id;
    _feedback.names.clear();
    _feedback.confidence.clear();
    
    _result.order_id = _goal_id;
    _result.names.clear();
    _result.confidence.clear();

    // publish info to the console for the user
    ROS_INFO("%s: Executing. order_id: %i, order_argument: %s ", _action_name.c_str(), _goal_id, _goal_argument.c_str());

    switch (_goal_id) {
      // RECOGNIZE ONCE
      case 0:
        // cout << "case 0" << endl;
        _fr = new FaceRec(true);
        _image_sub = _it.subscribe("/usb_cam/image_raw", 1, &FaceRecognition::recognizeCb, this);
        
        while( _as.isActive() && !_as.isPreemptRequested() && !ros::isShuttingDown() )
          r.sleep();

        break;
      
      // RECOGNIZE CONTINUOUSLY
      case 1:
        //cout << "case 1" << endl;
        _fr = new FaceRec(true);
        _image_sub = _it.subscribe("/usb_cam/image_raw", 1, &FaceRecognition::recognizeCb, this);
        
        while( _as.isActive() && !_as.isPreemptRequested() && !ros::isShuttingDown() )
          r.sleep();
        
        break;
      
      // ADD TRAINING IMAGES
      case 2:
        // cout << "case 2" << endl;
        _fc = new FaceImgCapturer(_goal_argument);
        _image_sub = _it.subscribe("/usb_cam/image_raw", 1, &FaceRecognition::addTrainingImagesCb, this);
        
        while( _as.isActive() && !_as.isPreemptRequested() && !ros::isShuttingDown() )
          r.sleep();

        break;
      
      // TRAIN DATABASE
      case 3:
        // call training function and check if successful
        // bool trainingSuccessful = true;
        // cout << "case 3" << endl;
        if (true) 
          _as.setSucceeded(_result);
        else 
          _as.setAborted(_result);
        break;
      
      // EXIT
      case 4:
        // cout << "case 4" << endl;
        ROS_INFO("%s: Exit request.", _action_name.c_str());
        _as.setSucceeded(_result);
        r.sleep();
        ros::shutdown();
        break;
    }

  }

  // run face recognition on the recieved image 
  void recognizeCb(const sensor_msgs::ImageConstPtr& msg) {
    // cout << "entering.. recognizeCb" << endl;

    if (_as.isPreemptRequested() || !ros::ok()) {
      // std::cout << "preempt req or not ok" << std::endl;
      ROS_INFO("%s: Preempted", _action_name.c_str());
      // set the action state to preempted
      _as.setPreempted();
      // success = false;
      // break;
      // cout << "shutting down _image_sub" << endl;
      _image_sub.shutdown();
      return;
    }

    if (!_as.isActive()) {
      // std::cout << "not active" << std::endl;
      // cout << "shutting down _image_sub" << endl;
      _image_sub.shutdown();
      return;
    }

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("%s: cv_bridge exception: %s", _action_name.c_str(), e.what());
      return;
    }

    std::vector<cv::Rect> faces;
    // _fd.detectFaces(cv_ptr->image, faces, true);

    std::vector<cv::Mat> faceImgs = _fd.getFaceImgs(cv_ptr->image, faces, true);
    std::map<string, std::pair<string, double> > results;
    
    for( size_t i = 0; i < faceImgs.size(); i++ ) {
      cv::resize(faceImgs[i], faceImgs[i], cv::Size(100.0, 100.0));
      cv::cvtColor( faceImgs[i], faceImgs[i], CV_BGR2GRAY );
      cv::equalizeHist( faceImgs[i], faceImgs[i] );

      // rectangle( currentFrame, Point( faces[i].x, faces[i].y ), Point( faces[i].x + faces[i].width, faces[i].y + faces[i].height ), Scalar( 0, 255, 255 ) );
      results = _fr->recognize(faceImgs[i], true, true, true);
      
      /*
      std::cout << "Face " + boost::to_string(i) + ": " << std::endl;
      std::cout << "\tEigenfaces:" << std::endl;
      std::cout << "\t\tPredicted: " << results["eigenfaces"].first ;
      std::cout << ",  Confidence: " << boost::to_string(results["eigenfaces"].second) << std::endl ;
      std::cout << "\tFisherfaces:" << std::endl;
      std::cout << "\t\tPredicted: " << results["fisherfaces"].first ;
      std::cout << ",  Confidence: " << boost::to_string(results["fisherfaces"].second) << std::endl ;
      std::cout << "\tLBPH:" << std::endl;
      std::cout << "\t\tPredicted: " << results["lbph"].first ;
      std::cout << ",  Confidence: " << boost::to_string(results["lbph"].second) << std::endl ;
      */

      ROS_DEBUG("Face %lu:", i);
      ROS_DEBUG("\tEigenfaces: %s %f", results["eigenfaces"].first.c_str(), results["eigenfaces"].second);
      ROS_DEBUG("\tFisherfaces: %s %f", results["fisherfaces"].first.c_str(), results["fisherfaces"].second);
      ROS_DEBUG("\tLBPH: %s %f", results["lbph"].first.c_str(), results["lbph"].second);

    }


    // update GUI window
    // TODO check gui parameter
    cv::imshow("testing...", cv_ptr->image);
    cv::waitKey(3);

    // if faces were detected
    if (faceImgs.size() > 0) {
      // recognize only once
      if (_goal_id == 0) {
        // std::cout << "goal_id 0. setting succeeded." << std::endl;
        // cout << _recognitionAlgo << endl;
        _result.names.push_back(results[_recognitionAlgo].first);
        _result.confidence.push_back(results[_recognitionAlgo].second);
        _as.setSucceeded(_result);
      }
      // recognize continuously
      else {
        _feedback.names.push_back(results[_recognitionAlgo].first);
        _feedback.confidence.push_back(results[_recognitionAlgo].second);
        _as.publishFeedback(_feedback);
      }
    }
  
  } 

  // add training images for a person
  void addTrainingImagesCb(const sensor_msgs::ImageConstPtr& msg) {
    // cout << "addTrainingImagesCb" << endl;

    if (_as.isPreemptRequested() || !ros::ok()) {
      // std::cout << "preempt req or not ok" << std::endl;
      ROS_INFO("%s: Preempted", _action_name.c_str());
      // set the action state to preempted
      _as.setPreempted();
      // success = false;
      // break;
      // cout << "shutting down _image_sub" << endl;
      _image_sub.shutdown();
      return;
    }

    if (!_as.isActive()) {
      // std::cout << "not active" << std::endl;
      // cout << "shutting down _image_sub" << endl;
      _image_sub.shutdown();
      return;
    }

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("%s: cv_bridge exception: %s", _action_name.c_str(), e.what());
      return;
    }

    std::vector<cv::Rect> faces;
    // _fd.detectFaces(cv_ptr->image, faces, true);

    std::vector<cv::Mat> faceImgs = _fd.getFaceImgs(cv_ptr->image, faces, true);
    
    if (faceImgs.size() > 0)
      _fc->capture(faceImgs[0]);

    // call images capturing function
    _result.names.push_back(_goal_argument);
    // _result.confidence.push_back(2.0);
    
    
    if (_fc->getNoImgsClicked() >= 20) {
      // cv::imshow("testing...", cv::Mat::zeros(1, 1, CV_32F));   
      _as.setSucceeded(_result);
    }

    // update GUI window
    // check GUI parameter
    cv::imshow("testing...", cv_ptr->image);  
    cv::waitKey(3);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg) {
    ROS_INFO("imageCb...");
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // Update GUI Window
    cv::imshow("testing...", cv_ptr->image);
    cv::waitKey(3);
    
    // Output modified video stream
    // image_pub_.publish(cv_ptr->toImageMsg());
  }


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "corobot_face_recognition");

  FaceRecognition facerec(ros::this_node::getName());
  ros::spin();

  return 0;
}