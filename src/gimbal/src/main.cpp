#include <iostream>

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"

#include <roboutils/utils.h>

class RosNode
{

  public:
    RosNode()
    {
      sub = n.subscribe("/head", 1, &RosNode::callback, this);
    }

    void callback(const geometry_msgs::PoseStamped& msg)
    {
      std::cout << "header: " << std::endl;
      std::cout << "  seq: " << msg.header.seq << std::endl;
      std::cout << "  stamp: " << std::endl;
      std::cout << "    secs: " << msg.header.stamp.sec << std::endl;
      std::cout << "    nsecs: " << msg.header.stamp.nsec << std::endl;
      std::cout << "  frame_id: " << msg.header.frame_id << std::endl;

      std::cout << "pose: " << std::endl;
      std::cout << "  position: " << std::endl;
      std::cout << "    x: " << msg.pose.position.x << std::endl;
      std::cout << "    y: " << msg.pose.position.y << std::endl;
      std::cout << "    z: " << msg.pose.position.z << std::endl;
      std::cout << "  orientation: " << std::endl;
      std::cout << "    x: " << msg.pose.orientation.x << std::endl;
      std::cout << "    y: " << msg.pose.orientation.y << std::endl;
      std::cout << "    z: " << msg.pose.orientation.z << std::endl;
      std::cout << "    w: " << msg.pose.orientation.w << std::endl;
    }

  private:
    ros::NodeHandle n;
    ros::Subscriber sub;

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gimbal");

  RosNode node;

  ros::spin();

  return 0;
}
