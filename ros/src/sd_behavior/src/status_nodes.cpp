#include "sd_behavior/status_nodes.h"

std::string base_link_frame_id_;
std::string odom_frame_id_;
std::string map_frame_id_;

tf::TransformListener *listener_;

void StatusNodes::registerNodes(BT::BehaviorTreeFactory& factory, ros::NodeHandle& nh)
{
	nh.param("base_link_frame_id", base_link_frame_id_, std::string("base_link"));
    nh.param("odom_frame_id", odom_frame_id_, std::string("odom"));
    nh.param("map_frame_id", map_frame_id_, std::string("map"));

    factory.registerSimpleCondition("RobotPret", std::bind(StatusNodes::IsReady));

    listener_ = new tf::TransformListener(nh);
}

BT::NodeStatus StatusNodes::IsReady()
{
    tf::StampedTransform transform;
    try {
        listener_->lookupTransform(odom_frame_id_, map_frame_id_, ros::Time(0), transform);
        listener_->lookupTransform(base_link_frame_id_, map_frame_id_, ros::Time(0), transform);

        return BT::NodeStatus::SUCCESS;
    }
    catch(tf::TransformException ex)
    {
        ROS_INFO("Robot is not ready");
    }
    return BT::NodeStatus::FAILURE;
}