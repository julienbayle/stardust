#include "ros/ros.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cmd_vel_selector");
    ROS_INFO("Starting cmd_vel_selector node");

    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    ros::spin();
}
