#include "behaviortree_cpp/bt_factory.h"
#include <chrono>

typedef std::chrono::time_point<std::chrono::high_resolution_clock> t_clock;
typedef std::chrono::duration<double> t_chrono;

/**
  * ------------
  * GLOBAL TIMER
  * ------------
  * 
  * Non démarrée <--
  *      |         |
  *  *StartTimer*  |
  *      |         |
  *      v         |
  *  En cours      |
  *      |         |
  *      -----*StopTimer* 
  *      |
  *      v
  *   Terminé (temps dépassé)
  * 
  *
  *   				IsStarted	IsNotTimeOut
  * Non démarrée	0			1
  * En cours		1			1
  * Terminé			1			0
  */
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

  BT::NodeStatus StopTimer(BT::TreeNode& self);

  BT::NodeStatus IsStarted();

	BT::NodeStatus IsNotTimeOut();
};