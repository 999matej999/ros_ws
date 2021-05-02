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
      #if 0
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
      #endif
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

  // Register signal and signal handler
  signal(SIGINT, signal_callback_handler);

  wiringPiSetup();
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

    #if 1
        std::cout.precision(2);
        std::cout << "YAW =\t" << std::fixed << ang_req.yaw << "\t\t";
        std::cout << "PITCH =\t" << std::fixed << ang_req.pitch << "\t\t";
        std::cout << "ROLL =\t" << std::fixed << ang_req.roll << std::endl;
    #endif

    #if 0  
        std::cout.precision(1);
        std::cout << std::fixed << ang_req.pitch << "\t" << std::fixed << angle_pitch << "\t" << elapsed_time << "\t" << speed_pitch << std::endl;
    #endif
  }
  i2c_close(fd); // i2c close

  return 0;
}
