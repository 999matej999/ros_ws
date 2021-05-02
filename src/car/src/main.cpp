#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include <roboutils/utils.h>
#include <roboutils/I2C.h>

#include "PCA9685.h"
#include "PCA9685Servo.h"
using namespace RoboUtils;

sensor_msgs::Joy new_msg = {};

class RosNode
{

  public:
    RosNode()
    {
      sub = n.subscribe("/joy", 1, &RosNode::callback, this);
    }

    void callback(const sensor_msgs::Joy& msg)
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

      new_msg = msg;
    }

  private:
    ros::NodeHandle n;
    ros::Subscriber sub;

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "car");

  RosNode node;

  auto i2c = new I2C();
  PCA9685Servo servo1(i2c);
  PCA9685Servo servo2(i2c);

  bool turbo_button_last = false;
  bool turbo = false;

  float rychlost=0;

  while(ros::ok())
  {
    ros::spinOnce();
    
    if(new_msg.header.seq > 0)
    {
      // here you can place some code
      std::cout << "----- x: " << new_msg.axes[1] << "----- y: " << new_msg.axes[3] << "Turbo: " << new_msg.buttons[9] << "Turbo: " << turbo << "Rychlost: " << rychlost << std::endl;
      servo1.SetDirection(3,rychlost);
      servo2.SetDirection(6,new_msg.axes[3]);

      if(new_msg.buttons[9] && !turbo_button_last)
      {
          turbo = !turbo;
      }
      turbo_button_last = new_msg.buttons[9];

      if(turbo==1) rychlost = new_msg.axes[1];
      if(turbo==0) rychlost = new_msg.axes[1]/5;

    }
  }
  
  return 0;
}
