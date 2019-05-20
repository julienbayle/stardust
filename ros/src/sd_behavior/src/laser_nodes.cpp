#include "sd_behavior/laser_nodes.h"

std::atomic<sd_sensor_msgs::LaserPatternDetector> last_detector_;
ros::Subscriber laser_detector_sub_;

void LaserNodes::registerNodes(BT::BehaviorTreeFactory& factory, ros::NodeHandle& nh)
{

    factory.registerSimpleCondition("LaVoieEstLibre-Devant",   std::bind(LaserNodes::IsFrontFreeFromObstacle));
	factory.registerSimpleCondition("LaVoieEstLibre-Gauche",   std::bind(LaserNodes::IsLeftFreeFromObstacle));
	factory.registerSimpleCondition("LaVoieEstLibre-Droite",   std::bind(LaserNodes::IsRightFreeFromObstacle));
	factory.registerSimpleCondition("LaVoieEstLibre-Derriere", std::bind(LaserNodes::IsBackFreeFromObstacle));

    factory.registerSimpleCondition("TiretteLaserPresente", std::bind(LaserNodes::IsLaserTirettePresent));

	laser_detector_sub_ = nh.subscribe("/r1/laser_pattern_detector", 1, LaserNodes::rosUpdate);
    sd_sensor_msgs::LaserPatternDetector initial_detector;
    initial_detector.IsTirette = false;
    last_detector_.store(initial_detector)
}

BT::NodeStatus LaserNodes::IsFrontFreeFromObstacle()
{
    bool d=last_detector_.IsFrontFreeFromObstacle;
    ROS_DEBUG_STREAM_NAMED("LaserNodes", "IsFrontFreeFromObstacle : " << d);
    return d ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus LaserNodes::IsLeftFreeFromObstacle()
{
    bool d=last_detector_.IsLeftFreeFromObstacle;
    ROS_DEBUG_STREAM_NAMED("LaserNodes", "IsLeftFreeFromObstacle : " << d);
    return d ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus LaserNodes::IsRightFreeFromObstacle()
{
    bool d=last_detector_.IsRightFreeFromObstacle;
    ROS_DEBUG_STREAM_NAMED("LaserNodes", "IsRightFreeFromObstacle : " << d);
    return d ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus LaserNodes::IsBackFreeFromObstacle()
{
    bool d=last_detector_.IsBackFreeFromObstacle;
    ROS_DEBUG_STREAM_NAMED("LaserNodes", "IsBackFreeFromObstacle : " << d);
    return d ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus LaserNodes::IsLaserTirettePresent()
{
    bool d=last_detector_.IsLaserTirettePresent;
    ROS_DEBUG_STREAM_NAMED("LaserNodes", "IsLaserTirettePresent : " << d);
    return d ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

void LaserNodes::rosUpdate(const sd_sensor_msgs::LaserPatternDetector &detector)
{
	ROS_DEBUG_STREAM_NAMED("LaserNodes", "Update laser detector");
	sd_sensor_msgs::LaserPatternDetector val = detector;
    last_detector_.store(val);
}
