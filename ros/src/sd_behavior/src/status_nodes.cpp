#include "sd_behavior/status_nodes.h"
#include <cstdlib>
#include <iostream>

std::string base_link_frame_id_;
std::string odom_frame_id_;
std::string map_frame_id_;

MoveBaseClient *ac_;

tf::TransformListener *listener_;

double max_load_average_; 

void StatusNodes::registerNodes(BT::BehaviorTreeFactory& factory, ros::NodeHandle& nh)
{
	ros::NodeHandle nh_priv("~");
    nh_priv.param("base_link_frame_id", base_link_frame_id_, std::string("base_link"));
    nh_priv.param("odom_frame_id", odom_frame_id_, std::string("odom"));
    nh_priv.param("map_frame_id", map_frame_id_, std::string("map"));

    listener_ = new tf::TransformListener(nh);

    std::string bt_move_base_topic;
    nh_priv.param("move_base_topic", bt_move_base_topic, std::string("move_base"));
    ac_ = new MoveBaseClient(nh, bt_move_base_topic, true);

    factory.registerSimpleCondition("RobotPret", std::bind(StatusNodes::IsReady));

    nh_priv.param("max_load_average", max_load_average_, 3.0);
    factory.registerSimpleCondition("RobotEnBonneSante", std::bind(StatusNodes::IsHealthy));
}

BT::NodeStatus StatusNodes::IsReady()
{
    tf::StampedTransform transform;
    try {
        listener_->lookupTransform(odom_frame_id_, map_frame_id_, ros::Time(0), transform);
        listener_->lookupTransform(base_link_frame_id_, map_frame_id_, ros::Time(0), transform);
        return ac_->waitForServer(ros::Duration(0.05)) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
        
    }
    catch(tf::TransformException ex)
    {
        ROS_INFO("Robot is not ready");
    }
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus StatusNodes::IsHealthy()
{
    double averages[3];

    if (getloadavg(averages, 3) != 3)
        return BT::NodeStatus::FAILURE;

    if (averages[0] > max_load_average_)
    {
        ROS_WARN_THROTTLE(1, "CPU usage limit is exceeded");
        return BT::NodeStatus::FAILURE;
    }
        

    return BT::NodeStatus::SUCCESS;
}