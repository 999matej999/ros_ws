#include <sstream>

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "head_tracking");
    ros::NodeHandle n;

    ros::Publisher publisher = n.advertise<geometry_msgs::PoseStamped>("/head", 1);
    ros::Rate loop_rate(10);

    int count = 0;
    while (ros::ok()) {

        geometry_msgs::PoseStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.header.seq = count++;
        msg.header.frame_id = "origin";

        msg.pose.position;
        msg.pose.position.x = 0.0;
        msg.pose.position.y = 0.0;
        msg.pose.position.z = 0.0;
        msg.pose.orientation.x = 4.0;
        msg.pose.orientation.y = 5.0;
        msg.pose.orientation.z = 6.0;
        msg.pose.orientation.w = 7.0;

        publisher.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

