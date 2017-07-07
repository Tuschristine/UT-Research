#include <ros/ros.h>
#include <tf/tf.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "std_msgs/String.h"
#include <std_msgs/Int16.h>
#include <vector>
#include <iostream>

using namespace std;

int x_value;

void turnToPink(const std_msgs::Int16::ConstPtr& msg)
{
    x_value = msg->data;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_base_client");
  ros::NodeHandle n;
    
     while (ros::ok())
    {
    move_base_msgs::MoveBaseGoal goal;
   
    double x = 0;
    double yaw = 0;
 
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true);
   
    //set the header
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.header.frame_id = "/base_link";
   
    //set relative x, y, and angle
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = 0.0;
    goal.target_pose.pose.position.z = 0.0;
   
    if(x_value < 310)
    {
        yaw = -0.1;
    }
   
    else if(x_value > 330)
    {
		yaw = 0.1;
    }
   
    else
    {
        yaw = 0.0;
    }
    
    while(ros::ok)
    {
		ROS_INFO("%d", x_value);
	}
   
    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

    //send the goal
    ac.sendGoal(goal);
   
    //block until the action is completed
    ac.waitForResult();

  }
 
  ros::Subscriber sub = n.subscribe("x", 1000, turnToPink);
 
  return 0;

}

   
    else if(x_value > 330)
    {
		yaw = 0.1;
    }
   
    else
    {
        yaw = 0.0;
    }
    
    ROS_INFO("%d", x_value);
   
    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

    //send the goal
    ac.sendGoal(goal);
   
    //block until the action is completed
    ac.waitForResult();

  }
 
  ros::Subscriber sub = n.subscribe("x", 1000, turnToPink);
 
  return 0;

}
