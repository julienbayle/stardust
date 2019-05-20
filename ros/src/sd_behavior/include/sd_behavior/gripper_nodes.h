#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/basic_types.h"
#include <ros/ros.h>
#include <chrono>

typedef std::chrono::time_point<std::chrono::high_resolution_clock> t_clock;
typedef std::chrono::duration<double> t_chrono;

namespace GripperNodes
{
   	void registerNodes(BT::BehaviorTreeFactory& factory, ros::NodeHandle& nh);

	// Async spinner node handle
	static ros::NodeHandle* nh_;
	
    class GripperNode : public BT::CoroActionNode
	{
		public:
			GripperNode(const std::string& name, const BT::NodeConfiguration& config);

			BT::NodeStatus tick() override;

			virtual void halt() override;
			
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
			t_clock start_;
			t_chrono duration_;
			std::atomic_bool halted_;
	};	

    class ArmNode : public BT::CoroActionNode
	{
		public:
			ArmNode(const std::string& name, const BT::NodeConfiguration& config);

			BT::NodeStatus tick() override;
			
			virtual void halt() override;

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
			t_clock start_;
			t_chrono duration_;
			std::atomic_bool halted_;
			const unsigned ROTATION_DELAY_BY_DEGREE = 30.0;       	
	};
}
