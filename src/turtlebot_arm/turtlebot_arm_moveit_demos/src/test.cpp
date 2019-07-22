#include <moveit/move_group_interface/move_group.h>
#include <ros/ros.h>
#include <serial/serial.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <control_msgs/FollowJointTrajectoryActionResult.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sensor_msgs/JointState.h>
#include <iostream>
#include <string.h>
#include <stdio.h>
#include <vector>
 
using namespace std;
 
class FollowJointTrajectoryAction
{
public:
 
 FollowJointTrajectoryAction(std::string name) :
   as_(nh_, name, false),
   action_name_(name)
 {
    as_.registerGoalCallback(boost::bind(&FollowJointTrajectoryAction::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&FollowJointTrajectoryAction::preemptCB, this));
    as_.start();
    Pub_jint = nh_.advertise<sensor_msgs::JointState>("/move_group/joint_states",10);
    js.name.resize(5);
    js.position.resize(5);
    js.name[0] = "arm_shoulder_pan_joint";
    js.name[1] = "arm_shoulder_lift_joint";
    js.name[2] = "arm_elbow_flex_joint";
    js.name[3] = "arm_wrist_flex_joint";
    js.name[4] = "gripper_joint";
    
    ROS_INFO("action start");
 }
   
 ~FollowJointTrajectoryAction(void)
  {
  }
 
 void goalCB()
 {
    ROS_INFO("goal is receive"); 
    double points_end[5];
    if(as_.isNewGoalAvailable())
     {
       js.position.clear();
       points_=&(as_.acceptNewGoal()->trajectory.points);
       Pos_length=points_->size(); 
       for(int co =0; co<5;co++)
       {
         points_end[co] = points_->at(Pos_length-1).positions[co];// points_[Pos_length-5+co];
         js.position.push_back(points_->at(Pos_length-1).positions[co]);
       }
       js.header.stamp = ros::Time::now();    
       Pub_jint.publish(js);
      }
     else
      {
        ROS_INFO("goal is not available"); 
      }
   control_msgs::FollowJointTrajectoryResult result;
   result.error_code = 0;
   as_.setSucceeded(result);
 } 
 
 void preemptCB()
 {
   ROS_INFO("%s: Preempted", action_name_.c_str());
   // set the action state to preempted
   as_.setPreempted();
 }
 
  protected: 
  sensor_msgs::JointState js;
  ros::NodeHandle nh_;
  ros::Publisher Pub_jint;
  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;
   
  //to the client
  control_msgs::FollowJointTrajectoryResult result_;
 
  //receive
  control_msgs::FollowJointTrajectoryGoal goal_;
};
 
int main(int argc, char** argv)
 {
   ros::init(argc, argv, "robothand_hard_driver");
   FollowJointTrajectoryAction followJointTrajectoryAction("f_arm_controller/follow_joint_trajectory");
   ros::spin();
   return 0;
}
