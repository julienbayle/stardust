// Exemple de base de behaviorTree.h pour test
#include "behaviortree_cpp/action_node.h"

namespace RobotNodes
{
class ApproachObject : public BT::SyncActionNode
{
	public:
		ApproachObject(const std::string& name);

		BT::NodeStatus tick() override;
};
};
