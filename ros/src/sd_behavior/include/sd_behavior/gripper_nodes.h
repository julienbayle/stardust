#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/basic_types.h"
#include <ros/ros.h>

namespace GripperNodes
{
	void registerNodes(BT::BehaviorTreeFactory& factory);

    class GripperNode : public BT::SyncActionNode
	{
		public:
			GripperNode(const std::string& name, const BT::NodeConfiguration& config);

			BT::NodeStatus tick() override;

		    static BT::PortsList providedPorts()
		    {
			    return { 
			    	BT::InputPort<double>("angle_base"), 
					BT::InputPort<double>("angle_pince"),
				};
			}
		private:
			ros::Publisher angle_base_pub_;
        	ros::Publisher angle_pince_pub_;
        	
	};
}