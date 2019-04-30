#include "behaviortree_cpp/bt_factory.h"

#include "ApproachObject.h"

using namespace BT;

int main()
{
	BehaviorTreeFactory factory;

	using namespace RobotNodes;
	factory.registerNodeType<ApproachObject>("ApproachObject");

//	factory.registerSimpleCondition("CheckBattery", std::bind(CheckBattery));
//
//	GripperInterface gripper;

//	factory.registerSimpleAction("OpenGripper", std:bind(&GripperInteface::open, &gripper));

//	factory.registerSimpleAction("CloseGripper", std:bind(&GripperInteface::close, &gripper));

	auto tree = factory.createTreeFromFile("./my_tree.xml");

	tree.root_node->executeTick();

	return 0;
}
