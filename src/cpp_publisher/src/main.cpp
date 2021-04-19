#include <sstream>

#include "ros/ros.h"
#include "std_msgs/Header.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "cpp_publisher");
    ros::NodeHandle n;

    ros::Publisher publisher = n.advertise<std_msgs::Header>("/my_topic", 1);
    ros::Rate loop_rate(10);

    int count = 0;
    while (ros::ok()) {

        std_msgs::Header msg;
        msg.stamp = ros::Time::now();
        msg.seq = count++;
        msg.frame_id = "origin";

        publisher.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

