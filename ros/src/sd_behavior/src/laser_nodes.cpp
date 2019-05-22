#include "sd_behavior/laser_nodes.h"

sd_sensor_msgs::LaserPatternDetector last_detector_;
ros::Subscriber laser_detector_sub_;

void LaserNodes::registerNodes(BT::BehaviorTreeFactory& factory, ros::NodeHandle& nh)
{

    factory.registerSimpleCondition("LaVoieEstLibre-Devant",   std::bind(LaserNodes::IsFrontFreeFromObstacle));
	factory.registerSimpleCondition("LaVoieEstLibre-Gauche",   std::bind(LaserNodes::IsLeftFreeFromObstacle));
	factory.registerSimpleCondition("LaVoieEstLibre-Droite",   std::bind(LaserNodes::IsRightFreeFromObstacle));
	factory.registerSimpleCondition("LaVoieEstLibre-Derriere", std::bind(LaserNodes::IsBackFreeFromObstacle));

    factory.registerSimpleCondition("TiretteLaserPresente", std::bind(LaserNodes::IsLaserTirettePresent));

    std::string bt_laser_detector_topic;
    nh.param("bt_laser_detector_topic", bt_laser_detector_topic, std::string("laser_pattern_detector"));
	laser_detector_sub_ = nh.subscribe(bt_laser_detector_topic, 1, LaserNodes::rosUpdate);
    sd_sensor_msgs::LaserPatternDetector initial_detector;
    initial_detector.isTirette = false;
    last_detector_ = initial_detector;
}

BT::NodeStatus LaserNodes::IsFrontFreeFromObstacle()
{
    bool d=last_detector_.isFrontFree;
    ROS_DEBUG_STREAM_NAMED("LaserNodes", "IsFrontFreeFromObstacle : " << d);
    return d ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus LaserNodes::IsLeftFreeFromObstacle()
{
    bool d=last_detector_.isLeftFree;
    ROS_DEBUG_STREAM_NAMED("LaserNodes", "IsLeftFreeFromObstacle : " << d);
    return d ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus LaserNodes::IsRightFreeFromObstacle()
{
    bool d=last_detector_.isRightFree;
    ROS_DEBUG_STREAM_NAMED("LaserNodes", "IsRightFreeFromObstacle : " << d);
    return d ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus LaserNodes::IsBackFreeFromObstacle()
{
    bool d=last_detector_.isBackFree;
    ROS_DEBUG_STREAM_NAMED("LaserNodes", "IsBackFreeFromObstacle : " << d);
    return d ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus LaserNodes::IsLaserTirettePresent()
{
    bool d=last_detector_.isTirette;
    ROS_DEBUG_STREAM_NAMED("LaserNodes", "IsLaserTirettePresent : " << d);
    return d ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

void LaserNodes::rosUpdate(const sd_sensor_msgs::LaserPatternDetector &detector)
{
	ROS_DEBUG_STREAM_NAMED("LaserNodes", "Update laser detector");
	sd_sensor_msgs::LaserPatternDetector val = detector;
    last_detector_ = val;
}
