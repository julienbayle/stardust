#include "robot_sensors_nodes.h"

static bool _is_tirette = true;
static bool _is_camp_violet = true;

BT::NodeStatus RobotSensors::IsTirettePresente()
{
    std::cout << "IsTirettePresente : " << _is_tirette << std::endl;
    return _is_tirette ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus RobotSensors::IsCampViolet()
{
	std::cout << "isCampViolet : " << _is_camp_violet << std::endl;
	return _is_camp_violet ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

void RobotSensors::rosUpdateTirettePresent(const std_msgs::BoolConstPtr &tirette)
{
	_is_tirette = (bool) tirette->data;
}

void RobotSensors::rosUpdateCampViolet(const std_msgs::BoolConstPtr &campviolet)
{
        _is_camp_violet = (bool) campviolet->data;
}

