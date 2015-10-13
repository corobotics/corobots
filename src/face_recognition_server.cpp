#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <corobot_face_recognition/FaceRecognitionAction.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

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

public:

  FaceRecognition(std::string name) :
    _as(_nh, name, boost::bind(&FaceRecognition::executeCB, this, _1), false),
    _it(_nh),
    _action_name(name)
  {
    _as.start();

    // Subscrive to input video feed and publish output video feed
    // _image_sub = _it.subscribe("/camera/image_raw", 1, &FaceRecognition::imageCb, this);
    _image_sub = _it.subscribe("/usb_cam/image_raw", 1, &FaceRecognition::imageCb, this);

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
      case 0:
        break;
      case 1:
        break;
      case 2:
        break;
      case 3:
        break;
      // EXIT
      case 4:
        ROS_INFO("%s: Exit request.", _action_name.c_str());
        _as.setSucceeded(_result);
        r.sleep();
        ros::shutdown();
        break;
    }

    // start executing the action
    /*for(int i=1; i<=goal->order; i++)
    {
      // check that preempt has not been requested by the client
      feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i-1]);
      // publish the feedback
      as_.publishFeedback(feedback_);
      // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
      r.sleep();
    }

    if(success)
    {
      result_.sequence = feedback_.sequence;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }*/

    // cv::waitKey(3);

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