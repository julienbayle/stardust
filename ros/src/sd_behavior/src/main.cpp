#include <string>
#include <ros/ros.h>
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/bt_file_logger.h"
#include "behaviortree_cpp/loggers/bt_zmq_publisher.h"

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
  	ros::CallbackQueue queue;
  	nh.setCallbackQueue(&queue);

  	// Run the ROS loop in a separate thread as external calls such
  	// as service callbacks to load controllers can block the (main) control loop
  	ros::AsyncSpinner spinner(1, &queue);
  	spinner.start();

	// Init behavior tree
	BehaviorTreeFactory factory;

	GripperNodes::registerNodes(factory, nh);
	MoveNodes::registerNodes(factory, nh);
	ScoreNodes::registerNodes(factory, nh);
	SensorsNodes::registerNodes(factory, nh);
	TimerNodes::registerNodes(factory);

	std::string fn = argv[1];
	auto tree = factory.createTreeFromFile(fn);

    // Real time monitoring with Groot
    BT::PublisherZMQ publisher_zmq(tree);

	// This logger saves state changes on file
    BT::FileLogger logger_file(tree, "bt_trace.fbl", 20);

	// Run behavior tree
	BT::NodeStatus status = NodeStatus::RUNNING;

	while(ros::ok() && status != NodeStatus::FAILURE) {
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		status = tree.root_node->executeTick();
	}

	// Wait for ROS threads to terminate
	ros::waitForShutdown();

	// Release AsyncSpinner object
  	spinner.stop();

	return 0;
}
