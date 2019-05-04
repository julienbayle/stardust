// Exemple de base de behaviorTree.h pour test
#include "behaviortree_cpp/action_node.h"

namespace RobotNodes
{
class ApproachObject : public BT::SyncActionNode
{
	public:
		ApproachObject(const std::string& name, const BT::NodeConfiguration& config);

		BT::NodeStatus tick() override;

    // It is mandatory to define this static method
    static BT::PortsList providedPorts()
    {
      // This action has a single input port called "locaion"
      // Any port must have a name. The type is optional.
      return { BT::InputPort<std::string>("location") };
    }

};
};
