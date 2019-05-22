#include "sd_behavior/control_nodes.h"

namespace ControlNodes
{
    void registerNodes(BT::BehaviorTreeFactory& factory) 
    {
        factory.registerNodeType<ControlNodes::RealParallelNode>("RealParallel");
    }


    RealParallelNode::RealParallelNode(const std::string &name, const BT::NodeConfiguration& config)
        : BT::ControlNode::ControlNode(name, config) {}

    BT::NodeStatus RealParallelNode::tick()
    {
        size_t success_childred_num = 0;
        size_t failure_childred_num = 0;
        size_t running_childred_num = 0;
        
        const size_t children_count = children_nodes_.size();

        // Routing the tree according to the sequence node's logic:
        for (unsigned int i = 0; i < children_count; i++)
        {
            BT::TreeNode* child_node = children_nodes_[i];

            BT::NodeStatus child_status = child_node->executeTick();

            switch (child_status)
            {
                case BT::NodeStatus::SUCCESS:
                {
                    success_childred_num++;
                } break;

                case BT::NodeStatus::FAILURE:
                {
                    failure_childred_num++;
                } break;

                case BT::NodeStatus::RUNNING:
                {
                    running_childred_num++;
                }  break;

                default:
                {
                    throw BT::LogicError("A child node must never return IDLE");
                }
            }
        }

        if (running_childred_num > 0)
            return BT::NodeStatus::RUNNING;

        if (failure_childred_num > 0)
            return BT::NodeStatus::FAILURE;

        return BT::NodeStatus::SUCCESS;
    }

    void RealParallelNode::halt()
    {
        BT::ControlNode::halt();
    }
}