#include <iostream>

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"

#include <roboutils/utils.h>

#include "gimbal.h"

#include <unistd.h>
#include <cstdlib>
#include <signal.h>

geometry_msgs::PoseStamped new_msg = {};

// Define the function to be called when ctrl+c (SIGINT) is sent to process
void signal_callback_handler(int signum) {
   std::cout << "Caught signal " << signum << std::endl;
   // Terminate program
   exit(signum);
}

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

  // Register signal and signal handler
  signal(SIGINT, signal_callback_handler);

  fd = i2c_init(1); // i2c init
  GimbalInit();
  //GimbalHome();

  while (ros::ok())
  {
    ros::spinOnce();

    quat.w = new_msg.pose.orientation.w;
    quat.x = new_msg.pose.orientation.x;
    quat.y = new_msg.pose.orientation.y;
    quat.z = new_msg.pose.orientation.z;

    ang_req = ToEulerAngles(quat);

    //ang_req = ToDegrees(ang_req); // only for second variant of calculations angles

    EndstopsControl();
    SetAngle(ang_req.pitch, MOTOR_PITCH, new_msg.header.stamp.sec, new_msg.header.stamp.nsec);
    SetAngle(ang_req.yaw, MOTOR_YAW, new_msg.header.stamp.sec, new_msg.header.stamp.nsec);
    km2_drive(fd, 0x71, speed_yaw, speed_pitch);

    std::cout.precision(2);
    std::cout << "YAW =\t" << std::fixed << ang_req.yaw << "\t\t";
    std::cout << "PITCH =\t" << std::fixed << ang_req.pitch << "\t\t";
    std::cout << "ROLL =\t" << std::fixed << ang_req.roll << std::endl;

    loop_rate.sleep();
  }
  i2c_close(fd); // i2c close

  return 0;
}
