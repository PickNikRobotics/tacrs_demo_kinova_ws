#include <behaviortree_cpp/bt_factory.h>
#include <moveit_studio_behavior_interface/behavior_context.hpp>
#include <moveit_studio_behavior_interface/shared_resources_node_loader.hpp>

#include <calibrate_camera_pose/calibrate_camera_pose.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace calibrate_camera_pose
{
class CalibrateCameraPoseBehaviorsLoader : public moveit_studio::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
    [[maybe_unused]] const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources) override
  {
    moveit_studio::behaviors::registerBehavior<CalibrateCameraPose>(factory, "CalibrateCameraPose", shared_resources);
  }
};
}  // namespace calibrate_camera_pose

PLUGINLIB_EXPORT_CLASS(calibrate_camera_pose::CalibrateCameraPoseBehaviorsLoader,
                       moveit_studio::behaviors::SharedResourcesNodeLoaderBase);
