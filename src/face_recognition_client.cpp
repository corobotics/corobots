/* Copyright 2012 Pouyan Ziafati, University of Luxembourg and Utrecht University
 * actionlib client example for the face_recognition simple actionlib server. The client subscribes to face_recognition::FRClientGoal messages. Each FRClientGoal message contains an order_id and an order_argument which specify a goal to be executed by the face_recognition server. After receiving a message, the client sends the corresponding goal to the server. By registering relevant call back functions, the client receives feedback and result information from the execution of goals in the server and prints such information on the terminal.
 * Provided by modifying the ROS wiki tutorial: "actionlib_tutorials/Tutorials/Writing a Callback Based Simple Action Client (last edited 2010-09-13 17:32:34 by VijayPradeep)"
 * License: Creative Commons Attribution 3.0. 
 */

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <corobot_face_recognition/FaceRecognitionClientGoal.h>
#include <corobot_face_recognition/FaceRecognitionAction.h>
#include <signal.h>

corobot_face_recognition::FaceRecognitionGoal goal; //Goal message
actionlib::SimpleActionClient<corobot_face_recognition::FaceRecognitionAction> * ac; //action lib client


// Called once when the goal completes
void doneCb(const actionlib::SimpleClientGoalState& state,
            const corobot_face_recognition::FaceRecognitionResultConstPtr& result)
{
  ROS_INFO("Goal [%i] Finished in state [%s]", result->order_id,state.toString().c_str());
  if(state.toString() != "SUCCEEDED") return;
  if( result->order_id==0)
    ROS_INFO("%s was recognized with confidence %f", result->names[0].c_str(),result->confidence[0]);          
  if( result->order_id==2)
    ROS_INFO("Pictures of %s were successfully added to the training images",result->names[0].c_str());
}

// Called once when the goal becomes active
void activeCb()
{
  ROS_INFO("Goal just went active");
}

// Called every time feedback is received for the goal
void feedbackCb(const corobot_face_recognition::FaceRecognitionFeedbackConstPtr& feedback)
{
  ROS_INFO("Received feedback from Goal [%d] ", feedback->order_id);
  if(feedback->order_id==1 )
    ROS_INFO("%s was recognized with confidence %f", feedback->names[0].c_str(),feedback->confidence[0]);          
  if( feedback->order_id==2)
    ROS_INFO("A picture of %s was successfully added to the training images",feedback->names[0].c_str());
}

//called for every FRClientGoal message received by the client. Client processes each message and sends the corresponding goal to the server and registers feedback and result and status call back functions.
void frclientCallback(const corobot_face_recognition::FaceRecognitionClientGoalConstPtr& msg)
  {
     
     ROS_INFO("request for sending goal [%i] is received", msg->order_id);
     goal.order_id = msg->order_id;
     goal.order_argument = msg->order_argument;
     ac->sendGoal(goal, &doneCb, &activeCb, &feedbackCb);        
  }
//shut down
void exit_handler(int s)
{
  delete(ac);
  ros::shutdown();
}


int main (int argc, char **argv)
{
  ros::init(argc, argv, "face_recognition_client");
  ros::NodeHandle n;
  ac = new actionlib::SimpleActionClient<corobot_face_recognition::FaceRecognitionAction>("corobot_face_recognition", true);
  //for proper shutdown exit_handler is used
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = exit_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);
  //wait for the server 
  ac->waitForServer();
  //subscribe to the topic of interest 
  ros::Subscriber sub = n.subscribe("face_rec_order", 1, frclientCallback);
  ros::spin();
  return 0;
}


/*
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <corobot_face_recognition/FaceRecognitionAction.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_corobot_face_recognition");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<corobot_face_recognition::FaceRecognitionAction> ac("corobot_face_recognition", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  corobot_face_recognition::FaceRecognitionGoal goal;
  goal.order_id = 2;
  goal.order_argument = "ronit";
  ac.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(300.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}
*/
