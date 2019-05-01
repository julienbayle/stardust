#include <string>
#include "behaviortree_cpp/bt_factory.h"

#include "ApproachObject.h"
#include "robot_sensors_nodes.h"

using namespace BT;

// WORK :
// catkin_make then
// rosrun sd_behavior maintree src/sd_behavior/config/bt_demo.xml

int main(int argc, char* argv[])
{
	BehaviorTreeFactory factory;

	factory.registerNodeType<RobotNodes::ApproachObject>("ApproachObject");
	factory.registerSimpleCondition("IsTirettePresent", std::bind(RobotSensors::IsTirettePresent));

	//	GripperInterface gripper;
	//	factory.registerSimpleAction("OpenGripper", std:bind(&GripperInteface::open, &gripper));
	//	factory.registerSimpleAction("CloseGripper", std:bind(&GripperInteface::close, &gripper));
	
	std::string fn = argv[1];
	auto tree = factory.createTreeFromFile(fn);
	tree.root_node->executeTick();

	return 0;
}
