#include "behaviortree_cpp/bt_factory.h"
#include <std_msgs/Bool.h>

namespace RobotSensors
{
	BT::NodeStatus IsTirettePresente();
	BT::NodeStatus IsCampViolet();
	void rosUpdateTirettePresent(const std_msgs::BoolConstPtr &tirette);
	void rosUpdateCampViolet(const std_msgs::BoolConstPtr &campviolet);
	
}
