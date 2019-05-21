#include "sd_behavior/score_nodes.h"
#include <std_msgs/String.h>
#include <sstream>

namespace ScoreNodes
{
  static std::atomic_int score_;

  static BT::PortsList scorePorts()
  {
    return { BT::InputPort<std::string>("score") };
  }

  static BT::PortsList outputScorePorts()
  {
    return { BT::OutputPort<std::string>("score") };
  }

  void registerNodes(BT::BehaviorTreeFactory& factory, ros::NodeHandle& nh) 
	{    
    factory.registerNodeType<DisplayScoreNode>("AfficherLeScore");
    
    factory.registerSimpleAction(
      "DefinirLeScore", 
      SetScore,
      scorePorts());

    factory.registerSimpleAction(
      "RecupererLeScore", 
      ExportScore,
      outputScorePorts());

    factory.registerSimpleAction(
      "AjouterAuScore", 
      AddScore,
      scorePorts());

    factory.registerSimpleCondition(
      "ScoreInferieurA", 
      IsScoreBelowThan,
      scorePorts());

    factory.registerSimpleCondition(
      "ScoreSuperieurA", 
      IsScoreGreaterThan,
      scorePorts());

    // Initial score
    score_.store(0);

    nh_ = &nh;
	}

  DisplayScoreNode::DisplayScoreNode(
    const std::string& name, 
    const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config),
          last_score_(-1)
  { 
    ros::NodeHandle nh;
		score_pub_ = nh_->advertise<std_msgs::String>("/r1/lcd/line1", 1);
  }

  BT::NodeStatus DisplayScoreNode::tick() 
  { 
    if(last_score_ != score_)
    {
      last_score_.store(score_);
      
      std::stringstream ss;
      std::cout.precision(3);
      ss << "Score : " << score_ << "     ";
      
      std_msgs::String msg;
      msg.data = ss.str();
		  score_pub_.publish(msg);

      ROS_DEBUG_STREAM_NAMED("ScoreNodes","Publish score : " << msg.data);
    }

	  return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus SetScore(BT::TreeNode& self)
  {
    auto msg = self.getInput<int>("score");
    if (!msg)
        throw BT::RuntimeError( "missing required input [score]: ", msg.error() );

    ROS_DEBUG_STREAM_NAMED("ScoreNodes", "Set score : " << msg.value());

    score_.store(msg.value());
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus ExportScore(BT::TreeNode& self)
  {
    std::stringstream ss;
    std::cout.precision(3);
    ss << score_ << " pts";
    self.setOutput("score", ss.str());
    ROS_DEBUG_STREAM_NAMED("ScoreNodes", "Recuperer le score");

    return BT::NodeStatus::SUCCESS;
  }
  
  BT::NodeStatus AddScore(BT::TreeNode& self)
  {
    auto msg = self.getInput<int>("score");
    if (!msg)
        throw BT::RuntimeError( "missing required input [score]: ", msg.error() );

    ROS_DEBUG_STREAM_NAMED("ScoreNodes","Add to score : " << msg.value());
    score_.fetch_add(msg.value());
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus IsScoreBelowThan(BT::TreeNode& self)
  {
    auto msg = self.getInput<int>("score");
    if (!msg)
        throw BT::RuntimeError( "missing required input [score]: ", msg.error() );

    return score_ < msg.value() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus IsScoreGreaterThan(BT::TreeNode& self)
  {
    auto msg = self.getInput<int>("score");
    if (!msg)
        throw BT::RuntimeError( "missing required input [score]: ", msg.error() );

    return score_ > msg.value() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
}