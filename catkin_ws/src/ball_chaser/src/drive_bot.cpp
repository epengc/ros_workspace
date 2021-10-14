// reference: https://github.com/phil-ludewig/Chase-Ball-Project/blob/master/catkin_ws_Project_GoChaseIt/src/ball_chaser/src/drive_bot.cpp
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <iostream>
// TODO: Include the ball_chaser "DriveToTarget" header file
#include "ball_chaser/DriveToTarget.h"
using namespace std;
// ROS::Publisher motor commands;
ros::Publisher motor_command_publisher;
// TODO: Create a handle_drive_request callback function that executes whenever a drive_bot service is requested.
// This function should publish the requested linear x and angular velocities to the robot wheel joints
// After publishing the requested velocities, a message feedback should be returned with the requested wheel velocities
bool handle_drive_request(ball_chaser::DriveToTarget::Request &req, ball_chaser::DriveToTarget::Response &res){
    ROS_INFO("GoToPositionRequest received linear_x:%1.2f, angular_x:%1.2f", (float)req.linear_x, (float)req.angular_z);
    // Publish the requested linear x and angluar velocities
    geometry_msgs::Twist motor_command;
    motor_command.linear.x = req.linear_x;
    motor_command.angular.z = req.angular_z;
    motor_command_publisher.publish(motor_command);

    res.msg_feedback =("Requested wheel velocities set - linear_x:"+std::to_string(req.linear_x)+", angluar_z:"+to_string(req.angular_z));
    return true;
}


int main(int argc, char** argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "drive_bot");

    // Create a ROS NodeHandle object
    ros::NodeHandle n;

    // Inform ROS master that we will be publishing a message of type 
    // geometry_msgs::Twist on the robot actuation topic with a publishing 
    // queue size of 10
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // TODO: Define a drive /ball_chaser/command_robot service with a 
    // handle_drive_request callback function

    // TODO: Delete the loop, move the code to the inside of the callback function and make the necessary changes to
    // publish the requested velocities and instead of constant values
    ros::ServiceServer drive_bot_service = n.advertiseService("/ball_chaser/command_robot", handle_drive_request);
    
    // Handle ROS communication events
    ROS_INFO("Drive bot service node initialized.");
    ros::spin();

    return 0;
}

