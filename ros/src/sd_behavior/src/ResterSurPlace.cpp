// Exemple de base de behaviorTree.cpp pour test
#include "ResterSurPlace.h"

namespace RobotNodes
{

	ResterSurPlace::ResterSurPlace(const std::string& name)
      : BT::SyncActionNode(name, {})
	    { }

	BT::NodeStatus ResterSurPlace::tick()
	{
        	// use the method value() to extract the valid message.
	        std::cout << "On reste sur place..." << std::endl;
        	return BT::NodeStatus::SUCCESS;
	}
}
