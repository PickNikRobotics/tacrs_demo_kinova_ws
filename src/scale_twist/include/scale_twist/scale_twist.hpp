#pragma once

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <behaviortree_cpp/action_node.h>

#include <moveit_studio_behavior_interface/shared_resources_node.hpp>


namespace scale_twist
{
/**
 * @brief TODO(...)
 */
class ScaleTwist : public moveit_studio::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:

  ScaleTwist(const std::string& name, const BT::NodeConfiguration& config, const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();

  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;

};
}  // namespace scale_twist
