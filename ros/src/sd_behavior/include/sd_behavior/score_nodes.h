#include "behaviortree_cpp/bt_factory.h"
#include <ros/ros.h>

namespace ScoreNodes
{

  void registerNodes(BT::BehaviorTreeFactory& factory);

  class DisplayScoreNode : public BT::SyncActionNode
  {
    public:
      DisplayScoreNode(const std::string& name, const BT::NodeConfiguration& config);

      BT::NodeStatus tick() override;

      static BT::PortsList providedPorts()
      {
        return { };
      }

    private:
      ros::Publisher score_pub_;
      std::atomic_int last_score_;
  };

  BT::NodeStatus SetScore(BT::TreeNode& self);
  
  BT::NodeStatus AddScore(BT::TreeNode& self);

  BT::NodeStatus IsScoreBelowThan(BT::TreeNode& self);

  BT::NodeStatus IsScoreGreaterThan(BT::TreeNode& self);

}
