#include "sd_behavior/sensors_nodes.h"
#include <bitset>

std::atomic<bool> 		is_tirette_;
std::atomic<bool>		is_camp_violet_;
std::atomic<bool>		is_robot_en_mouvement_;
std::atomic<unsigned>  		ui32_switches_;
geometry_msgs::PoseWithCovarianceStamped		position_robot_;
ros::Subscriber 		sensors_sub_;
ros::Subscriber 		position_sub_;
ros::Subscriber 		speed_sub_;

void SensorsNodes::registerNodes(BT::BehaviorTreeFactory& factory, ros::NodeHandle& nh)
{
	factory.registerSimpleCondition("IsTirettePresente", std::bind(SensorsNodes::IsTirettePresente));
	factory.registerSimpleCondition("IsCampViolet", std::bind(SensorsNodes::IsCampViolet));
	factory.registerSimpleCondition("IsArretUrgence", std::bind(SensorsNodes::IsArretUrgence));
	factory.registerSimpleCondition("IsRobotEnMouvement", std::bind(SensorsNodes::IsRobotEnMouvement));
	factory.registerSimpleCondition("IsPaletDroit", std::bind(SensorsNodes::IsPaletDroit));
	factory.registerSimpleCondition("IsPaletCentre", std::bind(SensorsNodes::IsPaletCentre));
	factory.registerSimpleCondition("IsPaletGauche", std::bind(SensorsNodes::IsPaletGauche));
	factory.registerSimpleCondition("IsVentouseDroite", std::bind(SensorsNodes::IsVentouseDroite));
	factory.registerSimpleCondition("IsVentouseCentre", std::bind(SensorsNodes::IsVentouseCentre));
	factory.registerSimpleCondition("IsVentouseGauche", std::bind(SensorsNodes::IsVentouseGauche));

	sensors_sub_ = nh.subscribe("/r1/pilo/switches", 1, SensorsNodes::rosUpdate);
	sensors_sub_ = nh.subscribe("/r1/amcl_pose", 1, SensorsNodes::rosUpdatePosition);
	speed_sub_ = nh.subscribe("/r1/auto_cmd_vel", 1, SensorsNodes::rosUpdateMovement);

	is_tirette_.store(false);
	is_camp_violet_.store(true);
	is_robot_en_mouvement_.store(false);
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

BT::NodeStatus SensorsNodes::IsRobotEnMouvement()
{
	ROS_DEBUG_STREAM_NAMED("RobotSensorsCondition", "IsRobotEnMouvement : " << is_robot_en_mouvement_);
	return is_robot_en_mouvement_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus SensorsNodes::IsArretUrgence()
{
	bool bit=(ui32_switches_&(1<<ARRET_URGENCE))!=0;
	ROS_DEBUG_STREAM_NAMED("RobotSensorsCondition", "IsArretUrgence : " << bit);
	return bit ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus SensorsNodes::IsPaletDroit()
{
	bool bit=(ui32_switches_&(1<<PALET_DROIT))==0;
	ROS_DEBUG_STREAM_NAMED("RobotSensorsCondition", "IsPaletDroit : " << bit);
	return bit ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus SensorsNodes::IsPaletCentre()
{
	bool bit=(ui32_switches_&(1<<PALET_CENTRE))==0;
	ROS_DEBUG_STREAM_NAMED("RobotSensorsCondition", "IsPaletCentre : " << bit);
	return bit ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
BT::NodeStatus SensorsNodes::IsPaletGauche()
{
	bool bit=(ui32_switches_&(1<<PALET_GAUCHE))==0;
	ROS_DEBUG_STREAM_NAMED("RobotSensorsCondition", "IsPaletGauche : " << bit);
	return bit ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus SensorsNodes::IsVentouseDroite()
{
	bool bit=(ui32_switches_&(1<<VENTOUSE_DROITE))!=0;
	ROS_DEBUG_STREAM_NAMED("RobotSensorsCondition", "IsVentouseDroite : " << bit);
	return bit ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
BT::NodeStatus SensorsNodes::IsVentouseCentre()
{
	bool bit=(ui32_switches_&(1<<VENTOUSE_CENTRE))!=0;
	ROS_DEBUG_STREAM_NAMED("RobotSensorsCondition", "IsVentouseCentre : " << bit);
	return bit ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
BT::NodeStatus SensorsNodes::IsVentouseGauche()
{
	bool bit=(ui32_switches_&(1<<VENTOUSE_GAUCHE))!=0;
	ROS_DEBUG_STREAM_NAMED("RobotSensorsCondition", "IsVentouseGauche : " << bit);
	return bit ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

void SensorsNodes::rosUpdateMovement(const geometry_msgs::Twist &speed)
{
        ROS_DEBUG_STREAM_NAMED("RobotSensorsCondition",
                "Update sensor values with "
                 << speed);
		is_robot_en_mouvement_ = false;
}

void SensorsNodes::rosUpdatePosition(const geometry_msgs::PoseWithCovarianceStamped &position)
{
        ROS_DEBUG_STREAM_NAMED("RobotSensorsCondition",
                "Update sensor values with "
                 << position.pose.pose.position.x << "," << position.pose.pose.position.y);
				 
				 position_robot_ = position;
}

void SensorsNodes::rosUpdate(const std_msgs::UInt32 &switches)
{
 	ROS_DEBUG_STREAM_NAMED("RobotSensorsCondition", 
	 	"Update sensor values with " 
		 << std::bitset<10>(switches.data));
	ui32_switches_.store(switches.data);
	is_tirette_.store((ui32_switches_&(1<<TIRRETTE_DEMARAGE))!=0);
	if(is_tirette_)
	{
		bool is_palet_gauche=(ui32_switches_&(1<<PALET_GAUCHE))==0;
		bool is_palet_droit=(ui32_switches_&(1<<PALET_DROIT))==0;
		if(is_palet_gauche)
			is_camp_violet_.store(true);

		if(is_palet_droit)
			is_camp_violet_.store(false);
	}
}
