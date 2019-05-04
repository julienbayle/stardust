// Exemple de base de behaviorTree.cpp pour test
#include "ApproachObject.h"

namespace RobotNodes
{

ApproachObject::ApproachObject(const std::string& name, const BT::NodeConfiguration& config)
      : BT::SyncActionNode(name, config)
    { }

BT::NodeStatus ApproachObject::tick() 
{
	BT::Optional<std::string> msg = getInput<std::string>("location");
	// Check if optional is valid. If not, throw its error
        if (!msg)
        {
            throw BT::RuntimeError("missing required input [location]: ", 
                                   msg.error() );
        }

        // use the method value() to extract the valid message.
        std::cout << "ApproachObject arrivé à la destination: " << msg.value() << std::endl;
        return BT::NodeStatus::SUCCESS;
}
}
