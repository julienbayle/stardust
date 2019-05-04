// Méthode pour l'affichage du score
#include "behaviortree_cpp/action_node.h"

namespace RobotNodes
{
class AfficherLeScore : public BT::SyncActionNode
{
	public:
		AfficherLeScore(const std::string& name, const BT::NodeConfiguration& config);

		BT::NodeStatus tick() override;

    // It is mandatory to define this static method
    static BT::PortsList providedPorts()
    {
      // This action has a single input port called "score"
      // Any port must have a name. The type is optional.
      return { BT::InputPort<std::string>("score") };
    }

};
};
