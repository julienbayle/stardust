#include <string>
#include <ros/ros.h>
#include "behaviortree_cpp/bt_factory.h"

#include "ApproachObject.h"
#include "IsNotTimeOut.h"
#include "AfficherLeScore.h"
#include "robot_sensors_nodes.h"

using namespace BT;

// WORK :
// catkin_make then
// roscore &
// rosrun sd_behavior maintree src/sd_behavior/config/bt_demo.xml

ros::Subscriber test_subscriber_;
ros::Subscriber subscriber_camp;

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "robot_behavior");
  	ros::NodeHandle nh;

  	BehaviorTreeFactory factory;
	factory.registerNodeType<RobotNodes::ApproachObject>("ApproachObject");
	factory.registerNodeType<RobotNodes::IsNotTimeOut>("IsNotTimeOut");
	factory.registerNodeType<RobotNodes::AfficherLeScore>("AfficherLeScore");
	factory.registerSimpleCondition("IsTirettePresente", std::bind(RobotSensors::IsTirettePresente));
	factory.registerSimpleCondition("IsCampViolet", std::bind(RobotSensors::IsCampViolet));
	
	//RobotSensors::init(factory, nh);
	test_subscriber_ = nh.subscribe("/test_topic", 1, RobotSensors::rosUpdateTirettePresent);
	subscriber_camp = nh.subscribe("/test_topic2", 1, RobotSensors::rosUpdateCampViolet);

	//	GripperInterface gripper;
	//	factory.registerSimpleAction("OpenGripper", std:bind(&GripperInteface::open, &gripper));
	//	factory.registerSimpleAction("CloseGripper", std:bind(&GripperInteface::close, &gripper));
	
	std::string fn = argv[1];
	auto tree = factory.createTreeFromFile(fn);

	BT::NodeStatus status = NodeStatus::RUNNING;
	
	ros::Rate rate(100);
    while(ros::ok() && status != NodeStatus::FAILURE) {
      ros::spinOnce();
      status = tree.root_node->executeTick();
      rate.sleep();
    }

    // Wait for ROS threads to terminate
  	ros::waitForShutdown();

	return 0;
}
