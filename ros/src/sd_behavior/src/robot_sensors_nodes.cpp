#include "robot_sensors_nodes.h"

static bool _is_tirette = true;

BT::NodeStatus RobotSensors::IsTirettePresent()
{
    std::cout << "IsTirettePresent : " << _is_tirette << std::endl;
    return _is_tirette ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}