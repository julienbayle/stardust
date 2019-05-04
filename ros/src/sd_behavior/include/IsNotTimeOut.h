#include "behaviortree_cpp/action_node.h"

namespace RobotNodes
{
class IsNotTimeOut : public BT::SyncActionNode
{
	public:
		IsNotTimeOut(const std::string& name);

		BT::NodeStatus tick() override;
	private:	
		 unsigned long debut;
		 unsigned long duree;
};
};
