#include <iostream>

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"

#include <roboutils/utils.h>

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
  Quaternion quat;
  EulerAngles ang_req;

  ros::init(argc, argv, "gimbal");

  RosNode node;
  ros::Rate loop_rate(100);

  Gimbal gimbal;

  gimbal.init();
  //gimbal.home();

  while (ros::ok())
  {
    ros::spinOnce();

    quat.w = new_msg.pose.orientation.w;
    quat.x = new_msg.pose.orientation.x;
    quat.y = new_msg.pose.orientation.y;
    quat.z = new_msg.pose.orientation.z;

    ang_req = toEulerAngles(quat);

    //ang_req = ToDegrees(ang_req); // only for second variant of calculations angles

    //gimbal.EndstopsControl();
    gimbal.setAngle(ang_req.pitch, MOTOR_PITCH, new_msg.header.stamp.sec, new_msg.header.stamp.nsec);
    gimbal.setAngle(ang_req.yaw, MOTOR_YAW, new_msg.header.stamp.sec, new_msg.header.stamp.nsec);
    gimbal.run();

    std::cout.precision(2);
    std::cout << "YAW =\t" << std::fixed << ang_req.yaw << "\t\t";
    std::cout << "PITCH =\t" << std::fixed << ang_req.pitch << "\t\t";
    std::cout << "ROLL =\t" << std::fixed << ang_req.roll << std::endl;

    loop_rate.sleep();
  }

  return 0;
}
