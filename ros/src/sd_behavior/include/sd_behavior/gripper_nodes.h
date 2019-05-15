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
				    	BT::InputPort<bool>("left_valve"), 
				    	BT::InputPort<bool>("middle_valve"), 
				    	BT::InputPort<bool>("right_valve"), 
				    	BT::InputPort<bool>("pump") 
				};
			}
		private:
			ros::Publisher activation_left_valve_pub_;
			ros::Publisher activation_middle_valve_pub_;
			ros::Publisher activation_right_valve_pub_;
			ros::Publisher activation_pump_pub_;
	};	

    class ArmNode : public BT::SyncActionNode
	{
		public:
			ArmNode(const std::string& name, const BT::NodeConfiguration& config);

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
			const int rotationDelayPerDegree = 30;       	
	};
}
