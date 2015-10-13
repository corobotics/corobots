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

class FaceRecognition
{
protected:
  ros::NodeHandle _nh;
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
  FaceRec _fr;

public:

  FaceRecognition(std::string name) :
    _as(_nh, name, boost::bind(&FaceRecognition::executeCB, this, _1), false),
    _it(_nh),
    _action_name(name),
    _fr(true)
  {
    _as.start();

    // Subscrive to input video feed and publish output video feed
    // _image_sub = _it.subscribe("/camera/image_raw", 1, &FaceRecognition::imageCb, this);
    // _image_sub = _it.subscribe("/usb_cam/image_raw", 1, &FaceRecognition::imageCb, this);

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
    ROS_INFO("%s: Executing. order_id: %i, order_argument: %s ", _action_name.c_str(), goal->order_id, (goal->order_argument).c_str());

    switch (_goal_id) {
      // RECOGNIZE ONCE
      case 0:
        _image_sub = _it.subscribe("/usb_cam/image_raw", 1, &FaceRecognition::recognizeCb, this);
        break;
      
      // RECOGNIZE CONTINUOUSLY
      case 1:
        _image_sub = _it.subscribe("/usb_cam/image_raw", 1, &FaceRecognition::recognizeCb, this);
        
        while( _as.isActive() && !_as.isPreemptRequested() && !ros::isShuttingDown() )
          r.sleep();
        
        // break;
      
      // ADD TRAINING IMAGES
      case 2:
        _image_sub = _it.subscribe("/usb_cam/image_raw", 1, &FaceRecognition::addTrainingImagesCb, this);
        
        while( _as.isActive() && !_as.isPreemptRequested() && !ros::isShuttingDown() )
          r.sleep();

        // break;
      
      // TRAIN DATABASE
      case 3:
        // call training function and check if successful
        // bool trainingSuccessful = true;
        
        if (true) 
          _as.setSucceeded(_result);
        else 
          _as.setAborted(_result);
        break;
      
      // EXIT
      case 4:
        ROS_INFO("%s: Exit request.", _action_name.c_str());
        _as.setSucceeded(_result);
        r.sleep();
        ros::shutdown();
        break;
    }

    /*
    // helper variables
    ros::Rate r(1);
    bool success = true;

    // push_back the seeds for the fibonacci sequence
    _feedback.order_id = goal->order_id;
    _result.order_id = goal->order_id;
    _feedback.names.clear();
    _feedback.confidence.clear();

    // publish info to the console for the user
    ROS_INFO("%s: Executing with order_id: %i, order_argument: %s", _action_name.c_str(), goal->order_id, (goal->order_argument).c_str());

    // start executing the action
    for(int i=1; i<=goal->order_id; i++)
    {
      // check that preempt has not been requested by the client
      if (_as.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", _action_name.c_str());
        // set the action state to preempted
        _as.setPreempted();
        success = false;
        break;
      }
      _feedback.names.push_back("test" + boost::to_string(i));
      _feedback.confidence.push_back(1.1);
      // publish the feedback
      _as.publishFeedback(_feedback);
      // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
      r.sleep();
    }

    if(success)
    {
      _result.names = _feedback.names;
      _result.confidence = _feedback.confidence;
      ROS_INFO("%s: Succeeded", _action_name.c_str());
      // set the action state to succeeded
      _as.setSucceeded(_result);
    }
    */
  }

  // run face recognition on the recieved image 
  void recognizeCb(const sensor_msgs::ImageConstPtr& msg) {
    if (_as.isPreemptRequested() || !ros::ok()) {
      std::cout << "preempt req or not ok" << std::endl;

      ROS_INFO("%s: Preempted", _action_name.c_str());
      // set the action state to preempted
      _as.setPreempted();
      // success = false;
      // break;
      return;
    }

    if (!_as.isActive()) {
      std::cout << "not active" << std::endl;
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
      results = _fr.recognize(faceImgs[i], true, true, true);
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
    }


    // update GUI window
    // TODO check gui parameter
    cv::imshow("testing...", cv_ptr->image);
    cv::waitKey(3);


    // recognize only once
    if (_goal_id == 0) {
      _result.names.push_back("recognizeCb once");
      _result.confidence.push_back(0.0);
      _as.setSucceeded(_result);
    }
    else {
      _feedback.names.push_back("recognizeCb continuous");
      _feedback.confidence.push_back(1.0);
      _as.publishFeedback(_feedback);
    }
  } 

  // add training images for a person
  void addTrainingImagesCb(const sensor_msgs::ImageConstPtr& msg) {
    if (_as.isPreemptRequested() || !ros::ok()) {
      std::cout << "preempt req or not ok" << std::endl;

      ROS_INFO("%s: Preempted", _action_name.c_str());
      // set the action state to preempted
      _as.setPreempted();
      // success = false;
      // break;
      return;
    }

    if (!_as.isActive()) {
      std::cout << "not active" << std::endl;
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

    // call images capturing function
    _result.names.push_back("addTrainingImagesCb");
    _result.confidence.push_back(2.0);
    _as.setSucceeded(_result);

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
  std::cout << ros::this_node::getName() << std::endl;
  ros::spin();

  return 0;
}