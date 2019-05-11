#include "sd_behavior/move_nodes.h"
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>

namespace MoveNodes
{
	void registerNodes(BT::BehaviorTreeFactory& factory) 
	{
		factory.registerNodeType<MoveNodes::ConstantVelocityNode>("VitesseConstante");
		factory.registerNodeType<MoveNodes::GotoNode>("Aller");
	}

	ConstantVelocityNode::ConstantVelocityNode(
		const std::string& name, 
		const BT::NodeConfiguration& config) :
	    SyncActionNode(name, config) 
	{
		ros::NodeHandle nh;
		std::string bt_cmd_vel_topic;
		nh.param("bt_cmd_vel_topic", bt_cmd_vel_topic, std::string("/cmd_vel"));
		cmd_pub = nh.advertise<geometry_msgs::Twist>(bt_cmd_vel_topic, 10);
	}

	BT::NodeStatus ConstantVelocityNode::tick()
	{
	    geometry_msgs::Twist twist;
	    if( !getInput("linear_x", twist.linear.x) )
	        throw BT::RuntimeError("linear_x is missing");
	    if( !getInput("linear_y", twist.linear.y) )
	        throw BT::RuntimeError("linear_y is missing");
	    if( !getInput("angular_z", twist.angular.z) )
	        throw BT::RuntimeError("angular_z is mission");

		cmd_pub.publish(twist);

		ROS_DEBUG_STREAM_NAMED("ConstantVelocityNode", 
			"Vitesse constante : " 
			<< "linear_x: " << twist.linear.x
			<< "- linear_y: " << twist.linear.y
			<< "- angular_z: " << twist.angular.z);

	    return BT::NodeStatus::SUCCESS;
	}

	GotoNode::GotoNode(
		const std::string& name, 
		const BT::NodeConfiguration& config):
	    	CoroActionNode(name,config),
	    	ac("move_base", true) { }

	BT::NodeStatus GotoNode::tick()
	{
		halted_.store(false);
		
		if (!ac.waitForServer(ros::Duration(1.0)))
		{
			ROS_ERROR("Unable to communicate with move base node");
			return BT::NodeStatus::FAILURE;
		}

	  	move_base_msgs::MoveBaseGoal goal;
		goal.target_pose.header.stamp = ros::Time::now();

		if( !getInput("x", goal.target_pose.pose.position.x) )
			throw BT::RuntimeError("x is missing");
		if( !getInput("y", goal.target_pose.pose.position.y) )
			throw BT::RuntimeError("y is missing");

		double theta;
		if( !getInput("theta", theta) )
			throw BT::RuntimeError("theta is missing");

		// generate quaternion
        tf::Quaternion quaternion;
		geometry_msgs::Quaternion qMsg;
        quaternion = tf::createQuaternionFromYaw(theta * (M_PI/180));
        tf::quaternionTFToMsg(quaternion, qMsg);

       	goal.target_pose.pose.orientation = qMsg;
	  	

		goal.target_pose.header.frame_id = "map_link";
		ROS_DEBUG_STREAM_NAMED("GotoNode", 
			"Aller en : " 
			<< "x: " << goal.target_pose.pose.position.x
			<< "y: " << goal.target_pose.pose.position.y
			<< "theta: " << theta);
			
		ac.sendGoal(goal);
		
		actionlib::SimpleClientGoalState ac_state = actionlib::SimpleClientGoalState::ACTIVE;
		while(ac_state == actionlib::SimpleClientGoalState::ACTIVE && !halted_)
		{
			ac_mutex_.lock();
			ac_state = ac.getState();
			ac_mutex_.unlock();
			setStatusRunningAndYield();
		}

		if(halted_)
			return BT::NodeStatus::FAILURE;
		
  		if(ac_state == actionlib::SimpleClientGoalState::SUCCEEDED)
  			return BT::NodeStatus::SUCCESS;
  		else
  			return BT::NodeStatus::FAILURE;
	}

	void GotoNode::halt()
	{
		halted_.store(true);
		ac_mutex_.lock();
		ac.cancelGoal();
		ac_mutex_.unlock();
	} 
}