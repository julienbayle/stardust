#include <string>
#include "behaviortree_cpp/bt_factory.h"

#include "ApproachObject.h"
#include "IsNotTimeOut.h"
#include "AfficherLeScore.h"
#include "robot_sensors_nodes.h"

using namespace BT;

// WORK :
// catkin_make then
// rosrun sd_behavior maintree src/sd_behavior/config/bt_demo.xml

int main(int argc, char* argv[])
{
	BehaviorTreeFactory factory;

	factory.registerNodeType<RobotNodes::ApproachObject>("ApproachObject");
	factory.registerNodeType<RobotNodes::IsNotTimeOut>("IsNotTimeOut");
	factory.registerNodeType<RobotNodes::AfficherLeScore>("AfficherLeScore");
	factory.registerSimpleCondition("IsTirettePresente", std::bind(RobotSensors::IsTirettePresente));
	factory.registerSimpleCondition("IsTimeOut", std::bind(RobotSensors::IsTimeOut));
	factory.registerSimpleCondition("IsCampViolet", std::bind(RobotSensors::IsCampViolet));
	factory.registerSimpleCondition("IsVoieLibre", std::bind(RobotSensors::IsVoieLibre));

	//	GripperInterface gripper;
	//	factory.registerSimpleAction("OpenGripper", std:bind(&GripperInteface::open, &gripper));
	//	factory.registerSimpleAction("CloseGripper", std:bind(&GripperInteface::close, &gripper));
	
	std::string fn = argv[1];
	auto tree = factory.createTreeFromFile(fn);

	while( tree.root_node->executeTick() != NodeStatus::FAILURE)
	{
        	std::this_thread::sleep_for( std::chrono::milliseconds(10) );
	}

	return 0;
}
