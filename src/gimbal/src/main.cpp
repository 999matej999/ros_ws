#include <iostream>

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"

#include <roboutils/utils.h>
#include <roboutils/I2C.h>

#include "gimbal.h"

geometry_msgs::PoseStamped new_msg = {};

class RosNode
{
	public:
		RosNode()
		{
			sub = n.subscribe("/head", 1, &RosNode::callback, this);
		}

		void callback(const geometry_msgs::PoseStamped& msg)
		{
			new_msg = msg;
		}

	private:
		ros::NodeHandle n;
		ros::Subscriber sub;
};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "gimbal");

	auto i2c = new RoboUtils::I2C();

	Gimbal gimbal(i2c);

	gimbal.home();

	RosNode node;
	ros::Rate loop_rate(100);
	
	while(ros::ok())
	{
		ros::spinOnce();

		gimbal.set(new_msg.pose.orientation, new_msg.header.stamp);
		gimbal.run();

		loop_rate.sleep();
	}

	delete i2c;

	return 0;
}
