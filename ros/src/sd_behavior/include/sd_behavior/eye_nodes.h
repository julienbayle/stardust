#include "behaviortree_cpp/bt_factory.h"
#include <ros/ros.h>

namespace EyeNodes
{
	void registerNodes(BT::BehaviorTreeFactory& factory, ros::NodeHandle& nh);

	// Async spinner node handle
	static ros::NodeHandle* nh_;
	
	class EyeNode : public BT::SyncActionNode
	{
		public:
			EyeNode(const std::string& name, const BT::NodeConfiguration& config);

			BT::NodeStatus tick() override;

		    static BT::PortsList providedPorts()
		    {
			    return { 
			    	BT::InputPort<double>("text"), 
					BT::InputPort<bool>("is_default"),
				};
			}
		
		private:
			ros::Publisher eye_pub;
			ros::Publisher default_eye_pub;
	};
}