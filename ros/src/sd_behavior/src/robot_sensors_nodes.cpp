#include "robot_sensors_nodes.h"

static bool _is_tirette = true;
static bool _is_time_out = true;
static bool _is_voie_libre = true;
static bool _is_camp_violet = true;

BT::NodeStatus RobotSensors::IsTirettePresente()
{
    std::cout << "IsTirettePresente : " << _is_tirette << std::endl;
    return _is_tirette ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus RobotSensors::IsTimeOut()
{
	std::cout << "isTimeOut : " << _is_time_out << std::endl;
	return _is_time_out ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus RobotSensors::IsCampViolet()
{
	std::cout << "isCampViolet : " << _is_camp_violet << std::endl;
	return _is_camp_violet ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus RobotSensors::IsVoieLibre()
{
        std::cout << "isVoieLibre : " << _is_voie_libre << std::endl;
        return _is_voie_libre ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

