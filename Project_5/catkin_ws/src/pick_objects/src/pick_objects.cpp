#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Int32.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects");

  ros::NodeHandle nd;
  ros::Publisher robot_position_pub = nd.advertise<std_msgs::Int32>("robot_is_at", 1000);

  std_msgs::Int32 position_msg;  // 1 = pickup; 2 = drop off; 0 = anywhere else
  position_msg.data = 0;  // Robot starts somewhere else
  robot_position_pub.publish(position_msg);   // Publishes first position information

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal_1;
  move_base_msgs::MoveBaseGoal goal_2;

  // set up the frame parameters
  goal_1.target_pose.header.frame_id = "map";	// base_link, base_footprint
  goal_1.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation of the first target
  goal_1.target_pose.pose.position.x = 0.17;	
  goal_1.target_pose.pose.position.y = 4.55;	
  goal_1.target_pose.pose.orientation.w = 1.57;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending first goal");
  ac.sendGoal(goal_1);

  // Wait an infinite time for the results
  ac.waitForResult();

 
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("The robot reached the first target pose.");
    position_msg.data = 1;
    robot_position_pub.publish(position_msg);
  }
  else
    ROS_INFO("Failed to reach the first target pose.");
  
  ROS_INFO("Waiting for 5 seconds...");
  ros::Duration(5.0).sleep();   // pause for 5 seconds

  ROS_INFO("Going to the second target pose.");

  // set up the frame parameters
  goal_2.target_pose.header.frame_id = "map";	// base_link, base_footprint
  goal_2.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation of the second target
  goal_2.target_pose.pose.position.x = -5.70;	
  goal_2.target_pose.pose.position.y = 4.55;	
  goal_2.target_pose.pose.orientation.w = 1.57;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending second goal");
  ac.sendGoal(goal_2);

  position_msg.data = 0;    // Robot is no longer at pickup position
  robot_position_pub.publish(position_msg);

  // Wait an infinite time for the results
  ac.waitForResult();
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("The robot reached the second target pose.");
    position_msg.data = 2;
    robot_position_pub.publish(position_msg);
  }
  else
    ROS_INFO("Failed to reach the second target pose.");
    

  ROS_INFO("Waiting for 5 seconds...");
  ros::Duration(5.0).sleep();   // pause for 5 seconds

  return 0;
}
