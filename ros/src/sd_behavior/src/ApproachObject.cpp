// Exemple de base de behaviorTree.cpp pour test
#include "ApproachObject.h"

namespace RobotNodes
{

ApproachObject::ApproachObject(const std::string& name) :
	BT::SyncActionNode(name, {})
{
}

BT::NodeStatus ApproachObject::tick() 
{
	std::cout << "ApproachObject : tick" << std::endl;
	return BT::NodeStatus::SUCCESS;
}
}
