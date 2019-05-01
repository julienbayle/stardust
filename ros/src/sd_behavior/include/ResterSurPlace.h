#include "behaviortree_cpp/action_node.h"

namespace RobotNodes
{
class ResterSurPlace : public BT::SyncActionNode
{
	public:
		ResterSurPlace(const std::string& name);

		BT::NodeStatus tick() override;

};
};
