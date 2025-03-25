#include <behaviortree_cpp/bt_factory.h>
#include <moveit_studio_behavior_interface/behavior_context.hpp>
#include <moveit_studio_behavior_interface/shared_resources_node_loader.hpp>

#include <scale_twist/scale_twist.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace scale_twist
{
class ScaleTwistBehaviorsLoader : public moveit_studio::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
    [[maybe_unused]] const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources) override
  {
    moveit_studio::behaviors::registerBehavior<ScaleTwist>(factory, "ScaleTwist", shared_resources);

  }
};
}  // namespace scale_twist

PLUGINLIB_EXPORT_CLASS(scale_twist::ScaleTwistBehaviorsLoader,
                       moveit_studio::behaviors::SharedResourcesNodeLoaderBase);
