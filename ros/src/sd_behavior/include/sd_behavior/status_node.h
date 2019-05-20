#include "behaviortree_cpp/bt_factory.h"
#include "tf/transform_listener.h"
#include <ros/ros.h>

namespace StatusNodes
{
	void registerNodes(BT::BehaviorTreeFactory& factory, ros::NodeHandle& nh);

    /**
     * Robot is ready when odom and map tf are availables
     */
    BT::NodeStatus IsReady();

    tf::TransformListener listener_;
};