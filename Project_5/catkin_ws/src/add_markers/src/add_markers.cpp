#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Int32.h>

bool robot_is_at_pickup = false;
bool robot_is_at_dropoff = false;

void robotPositionCallback(const std_msgs::Int32 &msg)
{
  ROS_INFO("New position information received.");
  if (msg.data == 0)
  {
    robot_is_at_pickup = false;
    robot_is_at_dropoff = false;
  }
  else
    if (msg.data == 1)
    {
      robot_is_at_pickup = true;
      robot_is_at_dropoff = false;
    }
    else
      if (msg.data == 2)
      {
        robot_is_at_pickup = false;
        robot_is_at_dropoff = true;
      }
    
}


int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  //ros::Subscriber sub = n.subscribe("odom", 1000, odometryCallback);
  ros::Subscriber sub = n.subscribe("robot_is_at", 1000, robotPositionCallback);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  while (ros::ok())
  {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = 0.17;
    marker.pose.position.y = 4.55;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.8f;
    marker.color.g = 0.0f;
    marker.color.b = 0.8f;
    marker.color.a = 1.0;

        // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }

    marker_pub.publish(marker);
    ROS_INFO("Cube placed at pickup zone.");
    
  
/*    
    // From the first add_markers task
    
    ros::Duration(5.0).sleep();   // pause for 5 seconds
    marker.action = visualization_msgs::Marker::DELETE;   // Delete marker
    marker_pub.publish(marker);
    ROS_INFO("Cube removed from pickup zone.");
    ros::Duration(5.0).sleep();   // pause for 5 seconds
    marker.pose.position.x = -5.70;
    marker.action = visualization_msgs::Marker::ADD;   // Delete marker
    marker_pub.publish(marker);
    ROS_INFO("Cube placed at drop off zone.");
    ros::Duration(5.0).sleep();   // pause for 5 seconds
*/  

    // Add and Delete marker based on odometry
    while (!robot_is_at_pickup)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Waiting for the robot to reach pickup zone.");
      ros::spinOnce();
      ros::Duration(0.5).sleep();
    }    
    // Robot is now at pick up position
    marker.action = visualization_msgs::Marker::DELETE;   // Delete marker
    marker_pub.publish(marker);
    ROS_INFO("Cube removed from pickup zone.");
    ros::Duration(5.0).sleep();   // pause for 5 seconds

    while (!robot_is_at_dropoff)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Waiting for the robot to reach drop off zone.");
      ros::spinOnce();
      ros::Duration(0.5).sleep();
    }    
    // Robot is now at drop off position
    marker.pose.position.x = -5.70;
    marker.action = visualization_msgs::Marker::ADD;   // Delete marker
    marker_pub.publish(marker);
    ROS_INFO("Cube placed at drop off zone.");
    ros::Duration(5.0).sleep();   // pause for 5 seconds

  return 0;

  }
}
