#include <string>
#include <ros/ros.h>
#include "behaviortree_cpp/bt_factory.h"

#include "sd_behavior/gripper_nodes.h"
#include "sd_behavior/move_nodes.h"
#include "sd_behavior/score_nodes.h"
#include "sd_behavior/sensors_nodes.h"
#include "sd_behavior/timer_nodes.h"

using namespace BT;

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "robot_behavior");
	ros::NodeHandle nh;

	// Init behavior tree
	BehaviorTreeFactory factory;

	GripperNodes::registerNodes(factory);
	MoveNodes::registerNodes(factory);
	ScoreNodes::registerNodes(factory);
	SensorsNodes::registerNodes(factory);
	TimerNodes::registerNodes(factory);

	std::string fn = argv[1];
	auto tree = factory.createTreeFromFile(fn);

	// Run behavior tree
	BT::NodeStatus status = NodeStatus::RUNNING;

	ros::Rate rate(50);
	while(ros::ok() && status != NodeStatus::FAILURE) {
		ros::spinOnce();
		status = tree.root_node->executeTick();
		rate.sleep();
	}

	// Wait for ROS threads to terminate
	ros::waitForShutdown();

	return 0;
}
