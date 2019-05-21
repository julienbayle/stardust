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
	    std::string message;
	    if( !getInput("message", message) )
	        throw BT::RuntimeError("message is missing");

	    bool is_default = false;
		getInput("defaut", is_default);
		
		int repeat = 1;
		getInput("repetition", repeat);

		int fps = 4;
		getInput("fps", fps);


		sd_led_matrix::Eye msg;
		msg.text = message.c_str();
		msg.repeat = repeat;
		msg.fps = fps;

		if (is_default)
			default_eye_pub.publish(msg);
		else
			eye_pub.publish(msg);
			
		return BT::NodeStatus::SUCCESS;
	}
	
}