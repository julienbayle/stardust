#include "behaviortree_cpp/bt_factory.h"
#include <sd_sensor_msgs/LaserPatternDetector.h>
#include <ros/ros.h>

namespace LaserNodes
{
	void registerNodes(BT::BehaviorTreeFactory& factory, ros::NodeHandle& nh);

	BT::NodeStatus IsFrontFreeFromObstacle();
	BT::NodeStatus IsLeftFreeFromObstacle();
	BT::NodeStatus IsRightFreeFromObstacle();
	BT::NodeStatus IsBackFreeFromObstacle();

	BT::NodeStatus IsLaserTirettePresent();

	void rosUpdate(const sd_sensor_msgs::LaserPatternDetector &detector);
}