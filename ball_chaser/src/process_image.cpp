#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
#include <vector>
#include <string>

// Define global vector of joints last position, moving state of the arm, and the client that can request services
std::vector<double> joints_last_position {0,0};
bool moving {false};
ros::ServiceClient client;
float ang_vel = 0.4;
float lin_vel = 0.4;
float waitT = 0.4;

// Send requests to the DriveToTarget 
void drive_robot(float lin_x, float ang_z){
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    if (!client.call(srv)){
        ROS_INFO("The drive_robot move was unsuccessful: Unable to call the command_robot service.");
    }
}

// Find the center of the ball using the image data. 
// Once found send velocities to the drive_robot function
void determine_position_callback(const sensor_msgs::Image &img)
{
    bool l_found {false};
    bool r_found {false};
    int ball_color = 690;   // 230+230+230
    double ball_l {};
    double ball_r {};
    double ball_m {};

    // Loop through the height and step of the image to find the left and right sides of the ball.
    for (int i{0}; i < img.height; i++){
        for (int j{0}; j <img.step; j+= 3){
            int r = img.data[(i*img.step +j)];
            int g = img.data[(i*img.step +j+1)];
            int b = img.data[(i*img.step +j+2)];
            int color_sum = r + g + b;

            if (color_sum >= ball_color and l_found == false){
            l_found = true;
            ball_l = j;
            ROS_INFO("Left side of ball has been found at step %f", ball_l);
            }
            if (img.data[(i*img.step +j)] != ball_color and l_found == true and r_found == false){
            r_found = true;
            ball_r = j;
            ROS_INFO("Right side of ball has been found at step %f", ball_r);
            }
        }
    }

    if (l_found){
        // Determine the center of the ball
        if (ball_r != 0){
            ball_m = (ball_r + ball_l)/2;
            ROS_INFO("Center of ball is at %f", ball_m);
        }
        else{ // The ball is at the right edge of the screen and ball_r will = 0 (the first pixel on the next row of pixels)
            ball_m = ball_l;
            ROS_INFO("Center of ball is at %f", ball_m);
        }

        // ROS_INFO("img.step = %d", img.step);    // 2400
        // Send velocities to drive_robot function based on ball position
        if (ball_m <= 800){ 
            drive_robot(0,ang_vel); // rotate left
            ROS_INFO("Moving Robot Left");
            // Wait 0.5 seconds for robot to rotate;
            ros::Duration(waitT).sleep();
            }
        else if (ball_m > 1600){ 
            drive_robot(0,-ang_vel); // rotate right
            ROS_INFO("Moving Robot Right");
            // Wait 0.5 seconds for robot to rotate;
            ros::Duration(waitT).sleep();
            } 
        else { 
            drive_robot(lin_vel,0); // drive straight
            ROS_INFO("Moving Robot Straight");
            // Wait 0.5 seconds for robot to rotate;
            ros::Duration(waitT).sleep();
            }

    }

    // If the ball hasn't been found, rotate around to find it
    if (l_found == false){
        ROS_WARN("Ball has not been found. Searching...");
        drive_robot(0,ang_vel+.75); // rotate left
        // Wait 0.5 seconds for robot to rotate;
        // ros::Duration(waitT).sleep();
    }
}


int main(int argc, char** argv)
{
    // Initialize the look_away node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from /ball_chaser/command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");
    ROS_INFO("DriveToTarget client made.");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the determine_position_callback function
    ros::Subscriber img_sub = n.subscribe("/camera/rgb/image_raw", 10, determine_position_callback);

    // Handle ROS communication events "spin"
    ros::spin();

    return 0;
}