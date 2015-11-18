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
  int _window_rows;
  int _window_cols;

  std::string _windowName;

  // PARAMS
  std::string _recognitionAlgo;
  std::string _preprocessing;
  int _noTrainingImgs;
  int _maxFaces;
  double _faceImgSize;
  bool _displayWindow;

public:

  FaceRecognition(std::string name) :
    _as(_nh, name, boost::bind(&FaceRecognition::executeCB, this, _1), false),
    _it(_nh),
    _ph("~"),
    _action_name(name),
    _fd(),
    _fr(NULL),
    _fc(NULL),
    _windowName("FaceRec"),
    _window_rows(0),
    _window_cols(0)
  {
    _as.start();

    // Subscrive to input video feed and publish output video feed
    // _image_sub = _it.subscribe("/camera/image_raw", 1, &FaceRecognition::imageCb, this);
    // _image_sub = _it.subscribe("/usb_cam/image_raw", 1, &FaceRecognition::imageCb, this);
    
    // Get params
    _ph.param<std::string>("algorithm", _recognitionAlgo , "lbph");
    _ph.param<int>("no_training_images", _noTrainingImgs , 20);
    _ph.param<int>("max_faces", _maxFaces , 2);
    _ph.param<bool>("display_window", _displayWindow, true);
    _ph.param<string>("preprocessing", _preprocessing, "tantriggs");

    try {
      _fr = new FaceRec(true, _preprocessing);
      ROS_INFO("Using %s for recognition.", _recognitionAlgo.c_str());
      ROS_INFO("Using %s for preprocessing.", _preprocessing.c_str());
    }
    catch( cv::Exception& e ) {
      const char* err_msg = e.what();
      ROS_INFO("%s", err_msg);
      ROS_INFO("Only ADD_IMAGES mode will work. Please add more subjects for recognition.");
    }

    cv::namedWindow(_windowName);
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
    ros::Rate r(60);
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
        // _fr = new FaceRec(true);
        _image_sub = _it.subscribe("/usb_cam/image_raw", 1, &FaceRecognition::recognizeCb, this);
        
        while( _as.isActive() && !_as.isPreemptRequested() && !ros::isShuttingDown() )
          r.sleep();

        break;
      
      // RECOGNIZE CONTINUOUSLY
      case 1:
        //cout << "case 1" << endl;
        // _fr = new FaceRec(true);
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
        _fr->train(_preprocessing);
        if (true) 
          _as.setSucceeded(_result);
        else 
          _as.setAborted(_result);
        break;
      
      // STOP
      case 4:
        if (_as.isActive()) {
          _as.setPreempted();
          // _image_sub.shutdown();
        }
        break;

      // EXIT
      case 5:
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

    _ph.getParam("algorithm", _recognitionAlgo);

    if (_as.isPreemptRequested() || !ros::ok()) {
      // std::cout << "preempt req or not ok" << std::endl;
      ROS_INFO("%s: Preempted", _action_name.c_str());
      // set the action state to preempted
      _as.setPreempted();
      // success = false;
      // break;
      // cout << "shutting down _image_sub" << endl;
      _image_sub.shutdown();
      Mat inactive(_window_rows, _window_cols, CV_8UC3, CV_RGB(20,20,20));
      appendStatusBar(inactive, "INACTIVE", "");
      cv::imshow(_windowName, inactive);
      cv::waitKey(3);
      return;
    }

    if (!_as.isActive()) {
      // std::cout << "not active" << std::endl;
      // cout << "shutting down _image_sub" << endl;
      _image_sub.shutdown();
      Mat inactive(_window_rows, _window_cols, CV_8UC3, CV_RGB(20,20,20));
      appendStatusBar(inactive, "INACTIVE", "");
      cv::imshow(_windowName, inactive);
      cv::waitKey(3);
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

    if (_window_rows == 0) {
      _window_rows = cv_ptr->image.rows;
      _window_cols = cv_ptr->image.cols;
    }

    // clear previous feedback
    _feedback.names.clear();
    _feedback.confidence.clear();
    // _result.names.clear();
    // _result.confidence.clear();

    std::vector<cv::Rect> faces;
    std::vector<cv::Mat> faceImgs = _fd.getFaceImgs(cv_ptr->image, faces, true);
    std::map<string, std::pair<string, double> > results;
    
    for( size_t i = 0; i < faceImgs.size(); i++ ) {
      cv::resize(faceImgs[i], faceImgs[i], cv::Size(100.0, 100.0));
      cv::cvtColor( faceImgs[i], faceImgs[i], CV_BGR2GRAY );
      cv::equalizeHist( faceImgs[i], faceImgs[i] );

      // perform recognition
      results = _fr->recognize(faceImgs[i],
                                ("eigenfaces" == _recognitionAlgo),
                                ("fisherfaces" == _recognitionAlgo),
                                ("lbph" == _recognitionAlgo)
                              );

      ROS_INFO("Face %lu:", i);
      if ("eigenfaces" == _recognitionAlgo)
        ROS_INFO("\tEigenfaces: %s %f", results["eigenfaces"].first.c_str(), results["eigenfaces"].second);
      if ("fisherfaces" == _recognitionAlgo)
        ROS_INFO("\tFisherfaces: %s %f", results["fisherfaces"].first.c_str(), results["fisherfaces"].second);
      if ("lbph" == _recognitionAlgo)
        ROS_INFO("\tLBPH: %s %f", results["lbph"].first.c_str(), results["lbph"].second);
    }


    // update GUI window
    // TODO check gui parameter
    appendStatusBar(cv_ptr->image, "RECOGNITION", "");
    cv::imshow(_windowName, cv_ptr->image);
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

    if (_window_rows == 0) {
      _window_rows = cv_ptr->image.rows;
      _window_cols = cv_ptr->image.cols;
    }

    std::vector<cv::Rect> faces;
    // _fd.detectFaces(cv_ptr->image, faces, true);

    std::vector<cv::Mat> faceImgs = _fd.getFaceImgs(cv_ptr->image, faces, true);
    
    if (faceImgs.size() > 0)
      _fc->capture(faceImgs[0]);

    // call images capturing function
    _result.names.push_back(_goal_argument);
    // _result.confidence.push_back(2.0);
    
    int no_images_to_click;
    _ph.getParam("no_training_images", no_images_to_click);
    
    if (_fc->getNoImgsClicked() >= no_images_to_click) {
      // Mat inactive = cv::Mat::zeros(cv_ptr->image.rows, cv_ptr->image.cols, CV_32F);
      Mat inactive(_window_rows, _window_cols, CV_8UC3, CV_RGB(20,20,20));
      appendStatusBar(inactive, "INACTIVE", "Images added. Please train.");
      cv::imshow(_windowName, inactive);  
      // cv::displayOverlay(_windowName, "Images added", 3000); 
      _as.setSucceeded(_result);
    } else {
      // update GUI window
      // check GUI parameter
      appendStatusBar(cv_ptr->image, "ADDING IMAGES.", "Images added");
      cv::imshow(_windowName, cv_ptr->image);  
    }

    cv::waitKey(3);
  }


  void appendStatusBar(Mat &img, std::string mode, std::string msg) {
    Mat row(20, img.cols, CV_8UC3, CV_RGB(56,66,72) );
    img.push_back(row);
    string text = "MODE: " + mode + "      " + msg;
    // putText(img, "STATUS", Point(10 ,img.cols + 10), FONT_HERSHEY_PLAIN, 5, Scalar(255,255,255));
    putText(img, text, Point2f(5, img.rows-5), FONT_HERSHEY_PLAIN, 1, Scalar(255,255,255), 0, CV_AA);
  }


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "corobot_face_recognition");

  FaceRecognition facerec(ros::this_node::getName());
  ros::spin();

  return 0;
}