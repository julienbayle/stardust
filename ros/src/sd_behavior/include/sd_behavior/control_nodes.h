#ifndef SD_CONTROL_NODES_H
#define SD_CONTROL_NODES_H

#include "behaviortree_cpp/control_node.h"
#include "behaviortree_cpp/bt_factory.h"

namespace ControlNodes
{

    void registerNodes(BT::BehaviorTreeFactory& factory);

    class RealParallelNode : public BT::ControlNode
    {
    public:

        RealParallelNode(const std::string& name, const BT::NodeConfiguration& config);

        static BT::PortsList providedPorts() { return {}; }

        ~RealParallelNode() = default;

        virtual void halt() override;

    private:
    
        virtual BT::NodeStatus tick() override;
    };

}
#endif   // SD_CONTROL_NODES_H
