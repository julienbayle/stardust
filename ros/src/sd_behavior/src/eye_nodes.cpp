#include "sd_behavior/eye_nodes.h"
#include "sd_led_matrix/Eye.h"

namespace EyeNodes
{
	void registerNodes(BT::BehaviorTreeFactory& factory, ros::NodeHandle& nh) 
	{
		factory.registerNodeType<EyeNodes::EyeNode>("AfficheYeux");
		nh_ = &nh;
	}

	EyeNode::EyeNode(
		const std::string& name, 
		const BT::NodeConfiguration& config):
	    SyncActionNode(name,config)
	{
		eye_pub = nh_->advertise<sd_led_matrix::Eye>("/r2/eye", 1);
		default_eye_pub = nh_->advertise<sd_led_matrix::Eye>("/r2/default_eye", 1);
	}

	BT::NodeStatus EyeNode::tick()
	{
	    bool is_default;
		if( !getInput("is_default", is_default) )
	        throw BT::RuntimeError("is_default is missing");

		std::string message;
	    if( !getInput("message", message) )
	        throw BT::RuntimeError("message is missing");

		sd_led_matrix::Eye msg;
		msg.text = message;
		
		if (is_default)
			eye_pub.publish(msg);
		else
			default_eye_pub.publish(msg);
			
		return BT::NodeStatus::SUCCESS;
	}
	
}