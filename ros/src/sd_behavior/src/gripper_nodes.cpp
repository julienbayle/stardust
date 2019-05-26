#include "sd_behavior/gripper_nodes.h"
#include "std_msgs/Int16.h"
#include "std_msgs/UInt16.h"

namespace GripperNodes
{
	
	void registerNodes(BT::BehaviorTreeFactory& factory, ros::NodeHandle& nh) 
	{
		factory.registerNodeType<GripperNodes::ArmNode>("Bras");
		factory.registerNodeType<GripperNodes::GripperNode>("Pince");
        nh_ = &nh;
	}

    ArmNode::ArmNode(
        const std::string& name, 
        const BT::NodeConfiguration& config) 
			: CoroActionNode(name, config)
    { 
        angle_base_pub_ = nh_->advertise<std_msgs::UInt16>("/r1/servo/E", 1);
        angle_pince_pub_ = nh_->advertise<std_msgs::UInt16>("/r1/servo/F", 1);
		duration_ = std::chrono::milliseconds(0);
		halted_.store(false);
    }

	BT::NodeStatus ArmNode::tick()
	{	
        unsigned angle_base, angle_pince;
	    if( !getInput("angle_base", angle_base) )
	        throw BT::RuntimeError("angle_base is missing");
	    if( !getInput("angle_pince", angle_pince) )
	        throw BT::RuntimeError("linear_y is missing");
	 
        ROS_DEBUG_STREAM_NAMED("ArmNode", 
			"Publish bras - " 
			<< "angle base: " << angle_base
			<< ", angle pince: " << angle_pince);

        std_msgs::UInt16 angle_base_msg;
        angle_base_msg.data = angle_base;
        angle_base_pub_.publish(angle_base_msg);

        ROS_DEBUG_STREAM_NAMED("ArmNode", "Publish pince F - " << angle_base_msg.data);

        std_msgs::UInt16 angle_pince_msg;
        angle_pince_msg.data = angle_pince;

        angle_pince_pub_.publish(angle_pince_msg);

        ROS_DEBUG_STREAM_NAMED("ArmNode", "Publish base E  - " << angle_pince_msg.data);

		duration_ = std::chrono::milliseconds(angle_pince*ROTATION_DELAY_BY_DEGREE);
		start_ = std::chrono::high_resolution_clock::now();	
		t_chrono elapsed_time = std::chrono::milliseconds(0);
		
			while (elapsed_time < duration_ && !halted_)
			{	
				elapsed_time = std::chrono::high_resolution_clock::now() - start_;
				ROS_DEBUG_STREAM_NAMED("GripperNode",
					"Attente en cours depuis " 
					<< std::fixed << std::setprecision(0) 
					<< 1000 * elapsed_time.count() << " ms");
				setStatusRunningAndYield();
			}
			
			if(! halted_)
			{
				ROS_DEBUG_STREAM_NAMED("GripperNode","Fin de l'attente");
			}

	return BT::NodeStatus::SUCCESS;
	}
	
	void ArmNode::halt() 
	{
		t_chrono elapsed_time = std::chrono::high_resolution_clock::now() - start_;
        ROS_DEBUG_STREAM_NAMED("ArmNode",
			"Arret de l'attente avant la fin, il restait " 
			<< std::fixed << std::setprecision(0) 
			<< 1000 * (duration_.count() - elapsed_time.count()) 
			<< " ms");
		halted_.store(true);
	}

    GripperNode::GripperNode(
        const std::string& name, 
        const BT::NodeConfiguration& config) 
			: CoroActionNode(name, config)
    { 
        activation_right_valve_pub_ = nh_->advertise<std_msgs::Int16>("/r1/pwm/vanne1", 1);
        activation_middle_valve_pub_ = nh_->advertise<std_msgs::Int16>("/r1/pwm/vanne3", 1);
	    activation_left_valve_pub_ = nh_->advertise<std_msgs::Int16>("/r1/pwm/vanne2", 1);
        activation_pump_pub_ = nh_->advertise<std_msgs::Int16>("/r1/pwm/pompe", 1);
		duration_ = std::chrono::milliseconds(10);
		halted_.store(false);
    }

	BT::NodeStatus GripperNode::tick()
	{
		
		
	bool left_valve, middle_valve, right_valve;
	    if( !getInput("left_valve", left_valve) )
	        throw BT::RuntimeError("left_valve is missing");
	    if( !getInput("middle_valve", middle_valve) )
	        throw BT::RuntimeError("middle_valve is missing");
	    if( !getInput("right_valve", right_valve) )
	        throw BT::RuntimeError("right_valve is missing");

        ROS_DEBUG_STREAM_NAMED("GripperNode", 
			"Publish pince - " 
			<< "left valve: " << left_valve
			<< "middle valve: " << middle_valve
			<< "right valve: " << right_valve);

	if(left_valve)
	{
		std_msgs::Int16 msg;
		msg.data = 4096;
		activation_pump_pub_.publish(msg);

		start_ = std::chrono::high_resolution_clock::now();	
		t_chrono elapsed_time = std::chrono::milliseconds(0);
		
		while (elapsed_time < duration_ && !halted_)
		{	
			elapsed_time = std::chrono::high_resolution_clock::now() - start_;
			ROS_DEBUG_STREAM_NAMED("GripperNode",
				"Attente en cours depuis " 
				<< std::fixed << std::setprecision(0) 
				<< 1000 * elapsed_time.count() << " ms");
			setStatusRunningAndYield();
		}
		
		 activation_left_valve_pub_.publish(msg);
		
		if(! halted_)
		{
			ROS_DEBUG_STREAM_NAMED("GripperNode","Fin de l'attente");
		}

	}
	else
	{
                std_msgs::Int16 msg;
                msg.data = 0;
                activation_left_valve_pub_.publish(msg);
	}

	if(right_valve)
        {
                std_msgs::Int16 msg;
                msg.data = 4096;
                activation_pump_pub_.publish(msg);

		start_ = std::chrono::high_resolution_clock::now();	
		t_chrono elapsed_time = std::chrono::milliseconds(0);
		
			while (elapsed_time < duration_ && !halted_)
			{	
				elapsed_time = std::chrono::high_resolution_clock::now() - start_;
				ROS_DEBUG_STREAM_NAMED("GripperNode",
					"Attente en cours depuis " 
					<< std::fixed << std::setprecision(0) 
					<< 1000 * elapsed_time.count() << " ms");
				setStatusRunningAndYield();
			}
			
					activation_right_valve_pub_.publish(msg);
			
			if(! halted_)
			{
				ROS_DEBUG_STREAM_NAMED("GripperNode","Fin de l'attente");
			}

        }
        else
        {
                std_msgs::Int16 msg;
                msg.data = 0;
                activation_right_valve_pub_.publish(msg);
        }

        if(middle_valve)
        {
                std_msgs::Int16 msg;
                msg.data = 4096;
                activation_pump_pub_.publish(msg);

		start_ = std::chrono::high_resolution_clock::now();	
		t_chrono elapsed_time = std::chrono::milliseconds(0);
		
			while (elapsed_time < duration_ && !halted_)
			{	
				elapsed_time = std::chrono::high_resolution_clock::now() - start_;
				ROS_DEBUG_STREAM_NAMED("GripperNode",
					"Attente en cours depuis " 
					<< std::fixed << std::setprecision(0) 
					<< 1000 * elapsed_time.count() << " ms");
				setStatusRunningAndYield();
			}
			
                activation_middle_valve_pub_.publish(msg);
			
			if(! halted_)
			{
				ROS_DEBUG_STREAM_NAMED("GripperNode","Fin de l'attente");
			}

        }
        else
        {
                std_msgs::Int16 msg;
                msg.data = 0;
                activation_middle_valve_pub_.publish(msg);
        }
	    if(!left_valve && !middle_valve && !right_valve)
	    {
		std_msgs::Int16 msg;
		msg.data = 0;
		activation_pump_pub_.publish(msg);

		start_ = std::chrono::high_resolution_clock::now();	
		t_chrono elapsed_time = std::chrono::milliseconds(0);
		ROS_DEBUG_STREAM_NAMED("GripperNode","Extinction de la pompe...");
	    }

	    return BT::NodeStatus::SUCCESS;
	}
	
		
	void GripperNode::halt() 
	{
		t_chrono elapsed_time = std::chrono::high_resolution_clock::now() - start_;
        ROS_DEBUG_STREAM_NAMED("GripperNode",
			"Arret de l'attente avant la fin, il restait " 
			<< std::fixed << std::setprecision(0) 
			<< 1000 * (duration_.count() - elapsed_time.count()) 
			<< " ms");
		halted_.store(true);
	}
}
