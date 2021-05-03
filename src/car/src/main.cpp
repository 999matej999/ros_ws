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
      std::cout << "header: " << std::endl;
      std::cout << "  seq: " << msg.header.seq << std::endl;
      std::cout << "  stamp: " << std::endl;
      std::cout << "    secs: " << msg.header.stamp.sec << std::endl;
      std::cout << "    nsecs: " << msg.header.stamp.nsec << std::endl;
      std::cout << "  frame_id: " << msg.header.frame_id << std::endl;

      std::cout << "axes: [";
      for(int i = 0; i < msg.axes.size(); ++i)
      {
        if(i != 0) std::cout << ", ";
        std::cout << msg.axes[i];
      }
      std::cout << "]" << std::endl;

      std::cout << "buttons: [";
      for(int i = 0; i < msg.buttons.size(); ++i)
      {
        if(i != 0) std::cout << ", ";
        std::cout << msg.buttons[i];
      }
      std::cout << "]" << std::endl;

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

  RosNode node;
  ros::Rate loop_rate(100);

  auto i2c = new RoboUtils::I2C();
  PCA9685Servo servo1(i2c, 3);
  PCA9685Servo servo2(i2c, 6);

  bool turbo_button_last = false;
  bool turbo = false;

  float rychlost = 0;

  while(ros::ok())
  {
    ros::spinOnce();
    
    if(new_msg_joy.header.seq > 0)
    {
      // here you can place some code
      if(new_msg_joy.buttons[9] && !turbo_button_last)
      {
          turbo = !turbo;
      }

      if(turbo == 1) rychlost = new_msg_joy.axes[1];
      if(turbo == 0) rychlost = new_msg_joy.axes[1]/5;

      std::cout << "----- x: " << new_msg_joy.axes[1] << "----- y: " << new_msg_joy.axes[3] << "Turbo: " << new_msg_joy.buttons[9] << "Turbo: " << turbo << "Rychlost: " << rychlost << std::endl;
      servo1.SetDirection(rychlost);
      servo2.SetDirection(new_msg_joy.axes[3]);
      
      turbo_button_last = new_msg_joy.buttons[9];
      loop_rate.sleep();
    }
  }

  delete i2c;
  
  return 0;
}
