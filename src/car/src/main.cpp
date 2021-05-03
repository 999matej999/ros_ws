#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PoseStamped.h>

#include <roboutils/utils.h>
#include <roboutils/I2C.h>

#include "PCA9685.h"
#include "PCA9685Servo.h"
#include "gimbal.h"

//using namespace RoboUtils;

sensor_msgs::Joy new_msg_joy = {};
geometry_msgs::PoseStamped new_msg_head = {};

class RosNode
{

	public:
		RosNode()
		{
			sub_joy = n.subscribe("/joy", 1, &RosNode::callback_joy, this);
			sub_head = n.subscribe("/head", 1, &RosNode::callback_head, this);
		}

		void callback_joy(const sensor_msgs::Joy& msg)
		{
			new_msg_joy = msg;
		}

		void callback_head(const geometry_msgs::PoseStamped& msg)
		{
			new_msg_head = msg;
		}

	private:
		ros::NodeHandle n;
		ros::Subscriber sub_joy;
		ros::Subscriber sub_head;

};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "car");

	auto i2c = new RoboUtils::I2C();
	PCA9685Servo servo1(i2c, 3);
	PCA9685Servo servo2(i2c, 6);

	bool turbo_button_last = false;
	bool turbo = false;

	Gimbal gimbal(i2c);
	gimbal.home();

	float speed = 0;

	RosNode node;
	ros::Rate loop_rate(100);

	while(ros::ok())
	{
		ros::spinOnce();
		
		if(new_msg_joy.header.seq > 0)
		{
			if(new_msg_joy.buttons[9] && !turbo_button_last) turbo = !turbo;

			if(turbo == 1) speed = new_msg_joy.axes[1];
			else speed = new_msg_joy.axes[1]/5;

			std::cout << "speed: " << speed << "\tdir: " << new_msg_joy.axes[3] << "\tturbo: " << turbo << std::endl;
			servo1.SetDirection(speed);
			servo2.SetDirection(new_msg_joy.axes[3]);
			
			turbo_button_last = new_msg_joy.buttons[9];
		}

		if(new_msg_head.header.seq > 0)
		{
			gimbal.set(new_msg_head.pose.orientation, new_msg_head.header.stamp);
			gimbal.run();
		}

		loop_rate.sleep();
	}

	delete i2c;
	
	return 0;
}
