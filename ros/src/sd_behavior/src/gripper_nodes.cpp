#include "sd_behavior/gripper_nodes.h"
#include <std_msgs/UInt16.h>

namespace GripperNodes
{
	void registerNodes(BT::BehaviorTreeFactory& factory) 
	{
		factory.registerNodeType<GripperNodes::GripperNode>("Pince");
	}

    GripperNode::GripperNode(
        const std::string& name, 
        const BT::NodeConfiguration& config) 
			: SyncActionNode(name, config)
    { 
        ros::NodeHandle nh;
        angle_base_pub_ = nh.advertise<std_msgs::UInt16>("servo/F", 1);
        angle_pince_pub_ = nh.advertise<std_msgs::UInt16>("servo/E", 1);
    }

	BT::NodeStatus GripperNode::tick()
	{	
        unsigned angle_base, angle_pince;
	    if( !getInput("angle_base", angle_base) )
	        throw BT::RuntimeError("angle_base is missing");
	    if( !getInput("angle_pince", angle_pince) )
	        throw BT::RuntimeError("linear_y is missing");
	 
        ROS_DEBUG_STREAM_NAMED("GripperNode", 
			"Publish pince - " 
			<< "angle base: " << angle_base
			<< ", angle pince: " << angle_pince);

        std_msgs::UInt16 angle_base_msg;
        angle_base_msg.data = angle_base;
        angle_base_pub_.publish(angle_base_msg);

        std_msgs::UInt16 angle_pince_msg;
        angle_pince_msg.data = angle_pince;
        angle_pince_pub_.publish(angle_pince_msg);

	    return BT::NodeStatus::SUCCESS;
	}
}