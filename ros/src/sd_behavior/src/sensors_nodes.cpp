#include "sd_behavior/sensors_nodes.h"
#include <ros/ros.h>
#include <bitset>

std::atomic<bool> 		is_tirette_;
std::atomic<bool>		is_camp_violet_;
std::atomic<unsigned>  	ui32_switches_;
ros::Subscriber 		sensors_sub_;

void SensorsNodes::registerNodes(BT::BehaviorTreeFactory& factory)
{
	factory.registerSimpleCondition("IsTirettePresente", std::bind(SensorsNodes::IsTirettePresente));
	factory.registerSimpleCondition("IsCampViolet", std::bind(SensorsNodes::IsCampViolet));
	factory.registerSimpleCondition("IsArretUrgence", std::bind(SensorsNodes::IsArretUrgence));
	factory.registerSimpleCondition("IsPaletDroit", std::bind(SensorsNodes::IsPaletDroit));
	factory.registerSimpleCondition("IsPaletCentre", std::bind(SensorsNodes::IsPaletCentre));
	factory.registerSimpleCondition("IsPaletGauche", std::bind(SensorsNodes::IsPaletGauche));
	factory.registerSimpleCondition("IsVentouseDroite", std::bind(SensorsNodes::IsVentouseDroite));
	factory.registerSimpleCondition("IsVentouseCentre", std::bind(SensorsNodes::IsVentouseCentre));
	factory.registerSimpleCondition("IsVentouseGauche", std::bind(SensorsNodes::IsVentouseGauche));

	ros::NodeHandle nh;
	sensors_sub_ = nh.subscribe("/r1/pilo/switches", 1, SensorsNodes::rosUpdate);
	is_tirette_.store(false);
	is_camp_violet_.store(true);
	ui32_switches_.store(0);
}

BT::NodeStatus SensorsNodes::IsTirettePresente()
{
    ROS_DEBUG_STREAM_NAMED("RobotSensorsCondition", "IsTirettePresente : " << is_tirette_);
    return is_tirette_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus SensorsNodes::IsCampViolet()
{
	ROS_DEBUG_STREAM_NAMED("RobotSensorsCondition", "IsCampViolet : " << is_camp_violet_);
	return is_camp_violet_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus SensorsNodes::IsArretUrgence()
{
	bool bit=(ui32_switches_&(1<<ARRET_URGENCE)>0);
	ROS_DEBUG_STREAM_NAMED("RobotSensorsCondition", "IsArretUrgence : " << bit);
	return bit ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus SensorsNodes::IsPaletDroit()
{
	bool bit=(ui32_switches_&(1<<PALET_DROIT)>0);
	ROS_DEBUG_STREAM_NAMED("RobotSensorsCondition", "IsPaletDroit : " << bit);
	return bit ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus SensorsNodes::IsPaletCentre()
{
	bool bit=(ui32_switches_&(1<<PALET_CENTRE)>0);
	ROS_DEBUG_STREAM_NAMED("RobotSensorsCondition", "IsPaletCentre : " << bit);
	return bit ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
BT::NodeStatus SensorsNodes::IsPaletGauche()
{
	bool bit=(ui32_switches_&(1<<PALET_GAUCHE)>0);
	ROS_DEBUG_STREAM_NAMED("RobotSensorsCondition", "IsPaletGauche : " << bit);
	return bit ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus SensorsNodes::IsVentouseDroite()
{
	bool bit=(ui32_switches_&(1<<VENTOUSE_DROITE)>0);
	ROS_DEBUG_STREAM_NAMED("RobotSensorsCondition", "IsVentouseDroite : " << bit);
	return bit ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
BT::NodeStatus SensorsNodes::IsVentouseCentre()
{
	bool bit=(ui32_switches_&(1<<VENTOUSE_CENTRE)>0);
	ROS_DEBUG_STREAM_NAMED("RobotSensorsCondition", "IsVentouseCentre : " << bit);
	return bit ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
BT::NodeStatus SensorsNodes::IsVentouseGauche()
{
	bool bit=(ui32_switches_&(1<<VENTOUSE_GAUCHE)>0);
	ROS_DEBUG_STREAM_NAMED("RobotSensorsCondition", "IsVentouseGauche : " << bit);
	return bit ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

void SensorsNodes::rosUpdate(const std_msgs::UInt32 &switches)
{
 	ROS_DEBUG_STREAM_NAMED("RobotSensorsCondition", 
	 	"Update sensor values with " 
		 << std::bitset<10>(switches.data));
	ui32_switches_.store(switches.data);
	is_tirette_.store(ui32_switches_&(1<<TIRRETTE_DEMARAGE)>0);
	if(is_tirette_)
	{
		if((ui32_switches_&(1<<PALET_GAUCHE)>0))
			is_camp_violet_.store(true);

		if((ui32_switches_&(1<<PALET_DROIT)>0))
			is_camp_violet_.store(false);
	}
}