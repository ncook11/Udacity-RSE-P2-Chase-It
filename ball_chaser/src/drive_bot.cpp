#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <std_msgs/Float64.h>
#include "geometry_msgs/Twist.h"
#include <string>

ros::Publisher motor_command_publisher;


// TODO: Create a handle_drive_request callback function that executes whenever a drive_bot service is
// This function should publish the requested linear x and angular velocities to the robot wheel joints requested
// After publishing the requested velocities, a message feedback should be returned with the requested wheel velocities
bool handle_drive_request(ball_chaser::DriveToTarget::Request &req,
    ball_chaser::DriveToTarget::Response &res)
    {
        // Output a message indicating that the callback has started
        ROS_INFO("DriveToTarget request recieved - [lin-x %1.2f, ang-z %1.2f]", (float)req.linear_x, (float)req.angular_z);

        // Create request message variables 
        geometry_msgs::Twist motor_command;
        
        // Set the wheel velocities [lin x, ang z]
        motor_command.linear.x = req.linear_x;
        motor_command.angular.z = req.angular_z;

        // Publish the values
        motor_command_publisher.publish(motor_command);

        // output the response message 
        res.msg_feedback = "velocity set - [lin-x " + std::to_string(req.linear_x) + ", ang-z " + std::to_string(req.angular_z) + "]";
        ROS_INFO_STREAM(res.msg_feedback);

        return true;
    }


int main(int argc, char** argv)
{
    ros::init(argc, argv, "drive_bot");
    ros::NodeHandle n;

    // Inform ROS master that we will be publishing a message of type 
    // geometry_msgs::Twist on the robot actuation topic ("/cmd_vel") with a publishing queue size of 10
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // Define a drive /ball_chaser/command_robot service 
    // with a handle_drive_request callback function
    ros::ServiceServer drive = n.advertiseService("/ball_chaser/command_robot", handle_drive_request);
    ROS_INFO("Ready to send drive commands");


    // Handle ROS Communication events
    ros::spin();
}
