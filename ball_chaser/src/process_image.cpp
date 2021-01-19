#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // Request a service and pass the velocities to it to drive the robot
	ROS_INFO_STREAM("Moving the robot to the ball");

	ball_chaser::DriveToTarget srv;
	srv.request.linear_x = lin_x;
	srv.request.angular_z = ang_z;

	if (!client.call(srv))
		ROS_ERROR("Failed to call service command_robot");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;

    // Loop through each pixel in the image and check if there's a bright white one
	int white_location = -1;

	for (int i = 0; i < img.height * img.step; i += 3){
		if ((img.data[i] == white_pixel) && (img.data[i+1] == white_pixel) && (img.data[i+2] == white_pixel)){
			white_location = i;
			break;
		}
	}

    // Identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it

	// Consider leftmost 5/12 of the image as left and rightmost 5/12 of it as right
	// Center 1/6 of it considered as center
	float left_threshold = img.step / 3;
	float right_threshold = 2 * img.step / 3;
	
	// Find the location in the row
	int horizontal_location = white_location % img.step;
	
	// Check if white ball detected	
	if (white_location >= 0){
		if (horizontal_location < left_threshold)
			drive_robot(0.0, 0.1); // Turn left
		else if (horizontal_location > right_threshold)
			drive_robot(0.0, -0.1); // Turn right
		else
			drive_robot(0.1, 0.0); // Move forward
	}
    // Request a stop when there's no white ball seen by the camera
	else
		drive_robot(0.0, 0.0); // Stop
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
