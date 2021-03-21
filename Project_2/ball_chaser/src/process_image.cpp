#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot
void drive_robot(float lin_x, float ang_z)
{
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;
    
    // Call the service and pass the reference speeds
    if (!client.call(srv))
        ROS_ERROR("Failed to call service command_robot.");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    int white_pixel = 255;
    int image_rows = img.height;
    int image_columns = img.width*3; // 3 channels: RBG
    int image_center = image_columns/2;
    float speed_gain = 2.0/image_center;
    
    // Set reference speeds to zero, in case no ball is detected.
    float u_ref=0.0, w_ref=0.0; 
    
    for (int i=image_rows/3; i < 2*image_rows/3; i++)   // scan only the center of the image to increase speed.
        for (int j=0; j < image_columns; j=j+3)
            // check if all three consecutive values are euqal to 255 (RBG=255,255,255)
            if ((img.data[i*image_columns+j]==white_pixel)&&(img.data[i*image_columns+j+1]==white_pixel)&&(img.data[i*image_columns+j+2]==white_pixel))
            {
                // If white pixel is detected, move the robot forward and turn it
                // in the direction of the pixel, with proportional angular speed.
                u_ref = 0.5;
                w_ref = (image_center - j)*speed_gain;
                ROS_INFO("Ball found.");
                break;
            }
    
    drive_robot(u_ref, w_ref);
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;
    
    // Define a client service to request command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");
    
    // Subscribe to /camera/rgb/image_raw topic
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);
    
    // Handke ROS communication events
    ros::spin();
    
    return 0;
}
