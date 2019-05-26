#include "behaviortree_cpp/bt_factory.h"
#include "tf/transform_listener.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

namespace StatusNodes
{
	void registerNodes(BT::BehaviorTreeFactory& factory, ros::NodeHandle& nh);

    /**
     * Robot is ready when odom and map tf are availables
     */
    BT::NodeStatus IsReady();

    /**
     * Robot is healthy (load average, ...)
     */
    BT::NodeStatus IsHealthy();
};