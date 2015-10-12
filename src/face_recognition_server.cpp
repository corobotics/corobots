#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <corobot_face_recognition/FaceRecognitionAction.h>

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

public:

  FaceRecognition(std::string name) :
    _as(_nh, name, boost::bind(&FaceRecognition::executeCB, this, _1), false),
    _action_name(name)
  {
    _as.start();
  }

  ~FaceRecognition(void)
  {
  }

  void executeCB(const corobot_face_recognition::FaceRecognitionGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1);
    bool success = true;

    // push_back the seeds for the fibonacci sequence
    // feedback_.sequence.clear();
    // feedback_.sequence.push_back(0);
    // feedback_.sequence.push_back(1);

    // publish info to the console for the user
    ROS_INFO("%s: Executing..", _action_name.c_str());

    // start executing the action
    /*for(int i=1; i<=goal->order; i++)
    {
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        break;
      }
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
  }


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "corobot_face_recognition");

  FaceRecognition facerec(ros::this_node::getName());
  ros::spin();

  return 0;
}