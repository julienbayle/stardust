#include "behaviortree_cpp/bt_factory.h"
#include <chrono>

typedef std::chrono::time_point<std::chrono::high_resolution_clock> t_clock;
typedef std::chrono::duration<double> t_chrono;

namespace TimerNodes
{
	void registerNodes(BT::BehaviorTreeFactory& factory);
	
	class WaitNode : public BT::CoroActionNode
	{
		public:
			WaitNode(const std::string& name, const BT::NodeConfiguration& config);

			BT::NodeStatus tick() override;

			virtual void halt() override;

			static BT::PortsList providedPorts()
			{
				return { BT::InputPort<std::string>("ms") };
			}

		private:

		t_clock start_;
		t_chrono duration_;
		std::atomic_bool halted_;
	};

	BT::NodeStatus StartTimer(BT::TreeNode& self);

	BT::NodeStatus IsNotTimeOut();
};