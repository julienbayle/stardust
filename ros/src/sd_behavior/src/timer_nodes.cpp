#include "sd_behavior/timer_nodes.h"
#include <iomanip>
#include <ros/ros.h>

namespace TimerNodes
{
	// General timer
	static t_clock timer_start_;
	static t_chrono timer_duration_;
	static std::atomic<bool> timer_started_;

	static BT::PortsList timerPorts()
	{
		return { BT::InputPort<std::string>("seconds") };
	}

	void registerNodes(BT::BehaviorTreeFactory& factory)
	{
		factory.registerNodeType<WaitNode>("Attendre");

		factory.registerSimpleCondition(
			"MinuterieNonEcoulee", 
			std::bind(TimerNodes::IsNotTimeOut));

        factory.registerSimpleCondition(
			"MinuterieDemarree", 
			std::bind(TimerNodes::IsStarted));

		factory.registerSimpleAction(
			"LancerLaMinuterie", 
			StartTimer, 
			timerPorts());

        factory.registerSimpleAction(
			"ArreterLaMinuterie", 
			StopTimer);

		timer_started_.store(false);
		timer_duration_ = std::chrono::seconds(88);
	}

	WaitNode::WaitNode(
		const std::string& name, 
		const BT::NodeConfiguration& config)
		: BT::CoroActionNode(name, config)
	{ 
		duration_ = std::chrono::milliseconds(0);
		halted_.store(false);
	}

	BT::NodeStatus WaitNode::tick() 
	{ 
		unsigned duration;
		if (!getInput<unsigned>("ms", duration))
			throw BT::RuntimeError( "missing required input [ms]" );
		duration_ = std::chrono::milliseconds(duration);
		
		ROS_DEBUG_STREAM_NAMED("WaitNode", 
			"Demarrage attente pour " 
			<< std::fixed << std::setprecision(0) 
			<< duration << " ms");
		
		start_ = std::chrono::high_resolution_clock::now();	
		t_chrono elapsed_time = std::chrono::milliseconds(0);
		while (elapsed_time < duration_ && !halted_)
		{	
			elapsed_time = std::chrono::high_resolution_clock::now() - start_;
			ROS_DEBUG_STREAM_NAMED("WaitNode",
				"Attente en cours depuis " 
				<< std::fixed << std::setprecision(0) 
				<< 1000 * elapsed_time.count() << " ms");
			setStatusRunningAndYield();
		}
		
		if(! halted_)
		{
			ROS_DEBUG_STREAM_NAMED("WaitNode","Fin de l'attente");
		}
		return BT::NodeStatus::SUCCESS;
	}

	void WaitNode::halt() 
	{
		t_chrono elapsed_time = std::chrono::high_resolution_clock::now() - start_;
        ROS_DEBUG_STREAM_NAMED("WaitNode",
			"Arret de l'attente avant la fin, il restait " 
			<< std::fixed << std::setprecision(0) 
			<< 1000 * (duration_.count() - elapsed_time.count()) 
			<< " ms");
		halted_.store(true);
	}

	BT::NodeStatus IsNotTimeOut()
	{
		if( ! timer_started_)
		{
			ROS_DEBUG_STREAM_NAMED("GlobalTimer", "Minuterie non demarree");
			return BT::NodeStatus::SUCCESS;
		}
			
		t_chrono elapsed_time = std::chrono::high_resolution_clock::now() - timer_start_;
		t_chrono remaining_time = timer_duration_ - elapsed_time;
		if(remaining_time < std::chrono::milliseconds(0))
		{
			ROS_DEBUG_STREAM_NAMED("GlobalTimer", "Minuterie terminee");

			return BT::NodeStatus::FAILURE;
		}
		else
		{
			ROS_DEBUG_STREAM_NAMED("GlobalTimer",
				"Minuterie en cours (temps restant "  
				<< std::fixed << std::setprecision(1) 
				<< remaining_time.count() << " sec. )");
			return BT::NodeStatus::SUCCESS;
		}		
	}

    BT::NodeStatus IsStarted()
	{
		return timer_started_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
	}

	BT::NodeStatus StartTimer(BT::TreeNode& self)
	{
		auto msg = self.getInput<int>("seconds");
		if (!msg)
			throw BT::RuntimeError( "missing required input [seconds]: ", msg.error() );

		timer_start_ = std::chrono::high_resolution_clock::now();
		timer_duration_ = std::chrono::seconds(msg.value());
		timer_started_.store(true);
		ROS_DEBUG_STREAM_NAMED("GlobalTimer",
			"Lancement de la minuterie pour " 
			<< std::fixed << std::setprecision(0) 
			<< timer_duration_.count() 
			<< " seconde(s)");
			
		return BT::NodeStatus::SUCCESS;
	}

    BT::NodeStatus StopTimer()
	{
		timer_started_.store(false);
		ROS_DEBUG_STREAM_NAMED("GlobalTimer", "Arrêt de la minuterie");
			
		return BT::NodeStatus::SUCCESS;
	}
}
