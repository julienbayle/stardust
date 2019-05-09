#include "robot_sensors_nodes.h"


static bool _is_camp_violet = true;
static unsigned int _ui32_switches = 0;
static bool _is_tirette = false;

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
/*
void RobotSensors::rosUpdateTirettePresent(const std_msgs::BoolConstPtr &tirette)
{
	_is_tirette = (bool) tirette->data;
}
/*
void RobotSensors::rosUpdateCampViolet(const std_msgs::BoolConstPtr &campviolet)
{
        _is_camp_violet = (bool) campviolet->data;
}*/

BT::NodeStatus RobotSensors::IsPaletDroit()
{
bool _is_bool=(_ui32_switches&(1<<PALET_GAUCHE)==0);
	std::cout << "IsPaletDroit : " << _is_bool << std::endl;
	return _is_camp_violet ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
BT::NodeStatus RobotSensors::IsPaletCentre()
{
bool _is_bool=(_ui32_switches&(1<<PALET_GAUCHE)==0);
	std::cout << "IsPaletCentre : " << _is_bool << std::endl;
	return _is_bool ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
BT::NodeStatus RobotSensors::IsPaletGauche()
{
bool _is_bool=(_ui32_switches&(1<<PALET_GAUCHE)==0);
	std::cout << "IsPaletGauche : " << _is_bool << std::endl;
	return _is_bool ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus RobotSensors::IsVentouseDroit()
{
bool _is_bool=(_ui32_switches&(1<<PALET_GAUCHE)==0);
	std::cout << "IsVentouseDroit : " << _is_bool << std::endl;
	return _is_camp_violet ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
BT::NodeStatus RobotSensors::IsVentouseCentre()
{
bool _is_bool=(_ui32_switches&(1<<PALET_GAUCHE)==0);
	std::cout << "IsVentouseCentre : " << _is_bool << std::endl;
	return _is_bool ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
BT::NodeStatus RobotSensors::IsVentouseGauche()
{
bool _is_bool=(_ui32_switches&(1<<PALET_GAUCHE)==0);
	std::cout << "IsVentouseGauche : " << _is_bool << std::endl;
	return _is_bool ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
void RobotSensors::rosUpdateSwitch(const std_msgs::UInt32 &switches)
{
  _ui32_switches = (bool) switches.data;
 _is_tirette = (_ui32_switches&(1<<TIRRETTE_DEMARAGE)==0);
	if(_is_tirette) // pas demarer
	{
		if((_ui32_switches&(1<<PALET_GAUCHE)==0))
			_is_camp_violet=true;
		if((_ui32_switches&(1<<PALET_DROIT)==0))
			_is_camp_violet=false;
	}
}

