#include "ros/ros.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Bool.h"

#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <iostream>
#include <cstdio>
#include <unistd.h>

using namespace std;

int angle, prevangle;

void get_angle(const std_msgs::UInt8::ConstPtr& ang)
{
	angle = ang->data;
}

void move_gripper()
{
	int usb_fd;
	char c_angle[4];

	//Check within limits of movement
	if (angle < 43 || angle > 100)
	{
		ROS_ERROR("Requested servo angle invalid.");
		return;
	}

	ROS_INFO("Moving gripper servo to %d degrees.", angle);

	//Open Teensy for communication. Requires Teensy udev rule. Baud irrelevant
	usb_fd = open("/dev/ttyACM0", B9600);

	if (usb_fd == -1)
	{
		ROS_ERROR("Failed to open /dev/ttyACM0");
		return;
	}

	//Convert integer to char array for communication
	sprintf(c_angle, "%d", angle);

	//Send angle to Teensy
	write(usb_fd, c_angle, strlen(c_angle));
	close(usb_fd);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Gripper");
	ros::NodeHandle node;
	ros::Publisher gripper_atpos_pub = node.advertise<std_msgs::Bool>("gripper_atpos", 1);
	ros::Subscriber angle_sub = node.subscribe("servo_angle",1,get_angle);
	const int looprate = 50;
	ros::Rate loop_rate(looprate);
	int transit_count;
	bool moving = false;

	ROS_INFO("Starting Gripper Control node");

	while(node.ok())
	{
		ros::spinOnce();
		std_msgs::Bool msg;

		//If moving to new position (new msg)
		if(angle != prevangle)
		{
			msg.data = false;
			gripper_atpos_pub.publish(msg);   //Publish gripper moving
			moving = true;
			transit_count = 0;

			//Move gripper to new angle
			move_gripper();
			prevangle = angle;
		}

		//Gripper was moved to new position
		else if (moving)
		{

			//0.75 second has passed since move to new position started
			if (transit_count >= 0.75*looprate)
			{
				//Set movement finished
				moving = false;
				msg.data = true;

				gripper_atpos_pub.publish(msg);  //Publish movement finished
			}
			else    //Less than 0.75 second has passed since move to new position
			{
				msg.data = false;
				gripper_atpos_pub.publish(msg);   //Publish gripper in transit
				transit_count++;
			}
		}
		else    //No new position to move to
		{
			msg.data = true;
			gripper_atpos_pub.publish(msg);  //Publish gripper stationary
		}
		loop_rate.sleep();
	}
	ROS_INFO("Closing Gripper Control node");
	return 0;
}
