// Exemple de base de behaviorTree.cpp pour test
#include "AfficherLeScore.h"

namespace RobotNodes
{

AfficherLeScore::AfficherLeScore(const std::string& name, const BT::NodeConfiguration& config)
      : BT::SyncActionNode(name, config)
    { }

BT::NodeStatus AfficherLeScore::tick() 
{
	BT::Optional<std::string> msg = getInput<std::string>("score");
	// Check if optional is valid. If not, throw its error
        if (!msg)
        {
            throw BT::RuntimeError("missing required input [score]: ", 
                                   msg.error() );
        }

        // use the method value() to extract the valid message.
        std::cout << "Le score est de : " << msg.value() << std::endl;
        return BT::NodeStatus::SUCCESS;
}
}
