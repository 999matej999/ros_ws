#include <iostream>

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const geometry_msgs::PoseStamped& msg)
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

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "gimbal");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber sub = n.subscribe("/head", 1, chatterCallback);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
