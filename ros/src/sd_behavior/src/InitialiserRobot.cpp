// Exemple de base de behaviorTree.cpp pour test
#include "InitialiserRobot.h"

namespace RobotNodes
{

InitialiserRobot::InitialiserRobot(const std::string& name, const BT::NodeConfiguration& config)
      : BT::SyncActionNode(name, config)
    { }

BT::NodeStatus InitialiserRobot::tick() 
{
	BT::Optional<std::string> msg = getInput<std::string>("location");
	// Check if optional is valid. If not, throw its error
        if (!msg)
        {
            throw BT::RuntimeError("missing required input [location]: ", 
                                   msg.error() );
        }

        // use the method value() to extract the valid message.
        std::cout << "InitialiserRobot a initialisé le robot ! " << std::endl;
        return BT::NodeStatus::SUCCESS;
}
}
