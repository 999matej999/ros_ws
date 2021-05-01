#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include <roboutils/utils.h>

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

  while(ros::ok())
  {
    ros::spinOnce();
    
    if(new_msg.header.seq > 0)
    {
      // here you can place some code
      std::cout << "----- x: " << new_msg.buttons[0] << std::endl;
    }
  }
  
  return 0;
}
