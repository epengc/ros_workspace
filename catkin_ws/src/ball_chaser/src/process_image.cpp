#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can requst services
ros::ServiceClient client;

// This function calls the command_robot service 
// to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z){
    // TODO: Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    if(!client.call(srv)){
        ROS_ERROR("Failed to call service command_robot");
    }
}

// This callback function continiously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img){
    int white_pixel = 255;
    // TODO:: Loop through each pixle in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid or  right side of the image
    // Depending on the white ball position, call the drive_bot fucntion and pass velocities to it
    // Request a stop when there 's no white ball seen by the camera
    int rgb_threshold = 240;
    int section_size = ceil(img.step/9);
    bool white_detected = false;
    int pixel_column = 0;
    
    for(int i=0; i<img.height*img.step-4; i+=3){
        int red_channel = img.data[i];
        int green_channel = img.data[i+1];
        int blue_channel = img.data[i+2];
        if(red_channel>rgb_threshold && green_channel>rgb_threshold && blue_channel>rgb_threshold){
            // White ball detected
            pixel_column=(i%img.step)/3;
            white_detected = true;
            break;
        }
    }
    if(white_detected){
        if(pixel_column<section_size){
            drive_robot(0.0, 0.5);
        }
        if(pixel_column<section_size*2){
            drive_robot(0.5, 0.0);
        }
        else{
            drive_robot(0.0, -0.5);
        }
    }
    else{
        drive_robot(0.0, 0.0);
    }
}

int main(int argc, char** argv){
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_iamge_callback_function
    ros::Subscriber sub1 =  n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);
    ROS_INFO("Process image node initialized.");
    ROS_INFO("Move ball in front of robot now.");
    // Handle ROS communication events
    ros::spin();

    return 0;
}

