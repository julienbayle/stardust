#include "sd_behavior/move_nodes.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>

namespace MoveNodes
{
	void registerNodes(BT::BehaviorTreeFactory& factory, ros::NodeHandle& nh) 
	{
		factory.registerNodeType<MoveNodes::SetPositionNode>("DefinirPosition");
        factory.registerNodeType<MoveNodes::ConstantVelocityNode>("VitesseConstante");
		factory.registerNodeType<MoveNodes::GotoNode>("Aller");

		nh_ = &nh;
	}

    SetPositionNode::SetPositionNode(
		const std::string& name, 
		const BT::NodeConfiguration& config) :
	    SyncActionNode(name, config) 
	{
		std::string bt_initialpose_topic;
		nh_->param("bt_initialpose_topic", bt_initialpose_topic, std::string("initialpose"));
		initialpose_pub = nh_->advertise<geometry_msgs::PoseWithCovarianceStamped>(bt_initialpose_topic, 1);
	}

	BT::NodeStatus SetPositionNode::tick()
	{
	    geometry_msgs::PoseWithCovarianceStamped initialpose;
	    if( !getInput("x", initialpose.pose.pose.position.x) )
	        throw BT::RuntimeError("x is missing");
	    if( !getInput("y", initialpose.pose.pose.position.y) )
	        throw BT::RuntimeError("y is missing");
	    double theta;
		if( !getInput("theta", theta) )
			throw BT::RuntimeError("theta is missing");

		ROS_DEBUG_STREAM_NAMED("SetPositionNode", 
			"Set position : " 
			<< "x: " << initialpose.pose.pose.position.x
			<< "- y: " << initialpose.pose.pose.position.y
			<< "- theta: " << theta);

        initialpose.header.frame_id = "map";
        initialpose.header.stamp = ros::Time::now();
        
        // generate quaternion
        tf::Quaternion quaternion;
        quaternion = tf::createQuaternionFromYaw(theta * (M_PI/180));
    
        initialpose.pose.pose.orientation.x = quaternion[0];
        initialpose.pose.pose.orientation.y = quaternion[1];
        initialpose.pose.pose.orientation.z = quaternion[2];
        initialpose.pose.pose.orientation.w = quaternion[3];
        initialpose.pose.covariance[0] = 0.001;
        initialpose.pose.covariance[7] = 0.001;
        initialpose.pose.covariance[21] = 0.001;

        initialpose_pub.publish(initialpose);

	    return BT::NodeStatus::SUCCESS;
	}

	ConstantVelocityNode::ConstantVelocityNode(
		const std::string& name, 
		const BT::NodeConfiguration& config) :
	    SyncActionNode(name, config) 
	{
		std::string bt_cmd_vel_topic;
		nh_->param("bt_cmd_vel_topic", bt_cmd_vel_topic, std::string("/cmd_vel"));
		cmd_pub = nh_->advertise<geometry_msgs::Twist>(bt_cmd_vel_topic, 10);
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
		const BT::NodeConfiguration& config)
		: CoroActionNode(name,config)
	{
		std::string bt_move_base_topic;
		nh_->param("bt_move_base_topic", bt_move_base_topic, std::string("move_base"));

		ROS_DEBUG_STREAM_NAMED("GotoNode",  
			"Init a goto node with : " << bt_move_base_topic);

		ac = new MoveBaseClient(*nh_, bt_move_base_topic, true);
	}

	BT::NodeStatus GotoNode::tick()
	{
		halted_.store(false);
		
		if (!ac->waitForServer(ros::Duration(2.0)))
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

		ROS_DEBUG_STREAM_NAMED("GotoNode", 
			"Aller en : " 
			<< "x: " << goal.target_pose.pose.position.x
			<< "y: " << goal.target_pose.pose.position.y
			<< "theta: " << theta);

		// generate quaternion
        tf::Quaternion quaternion;
		geometry_msgs::Quaternion qMsg;
        quaternion = tf::createQuaternionFromYaw(theta * (M_PI/180));
        tf::quaternionTFToMsg(quaternion, qMsg);

	goal.target_pose.pose.orientation = qMsg;
		goal.target_pose.header.frame_id = "map";

        // Cancel all goals
		ac->cancelAllGoals();
        ac->sendGoal(goal);

		actionlib::SimpleClientGoalState ac_state = actionlib::SimpleClientGoalState::PENDING;
		while(!ac_state.isDone() && !halted_)
		{
			ac_mutex_.lock();
			ac_state = ac->getState();

			ROS_DEBUG_STREAM_NAMED("GotoNode", 
				"Aller en : " 
				<< "x: " << goal.target_pose.pose.position.x
				<< "y: " << goal.target_pose.pose.position.y
				<< "theta: " << theta
				<< "statut: " << ac_state.toString());
			ac_mutex_.unlock();
			setStatusRunningAndYield();
		}

		if(ac_state == actionlib::SimpleClientGoalState::SUCCEEDED)
			return BT::NodeStatus::SUCCESS;

        return BT::NodeStatus::FAILURE;
	}

	void GotoNode::halt()
	{
		halted_.store(true);
		ac_mutex_.lock();
		ac->cancelGoal();
		ac_mutex_.unlock();
	} 
}