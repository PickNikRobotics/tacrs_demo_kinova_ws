#include "calibration_behaviors/save_calibration_pose_yaml.hpp"

#include <vector>
#include <fstream>

#include "fmt/format.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "yaml-cpp/yaml.h"

#include "moveit_studio_behavior_interface/get_required_ports.hpp"
#include "moveit_studio_common/utils/filesystem_utils.hpp"

namespace
{
  constexpr auto kPortIDCalibrationPoseStamped = "calibration_pose_stamped";
  constexpr auto kPortIDPackageName = "package_name";
  constexpr auto kPortIDFilePath = "file_path";
}

namespace calibration_behaviors
{
  SaveCalibrationPoseYaml::SaveCalibrationPoseYaml(const std::string &name, const BT::NodeConfiguration &config,
                                           const std::shared_ptr<moveit_studio::behaviors::BehaviorContext> &
                                           shared_resources)
    : moveit_studio::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}


BT::PortsList SaveCalibrationPoseYaml::providedPorts()
{
  return BT::PortsList({
    BT::InputPort<geometry_msgs::msg::PoseStamped>(kPortIDCalibrationPoseStamped, "{average_pose_stamped}",
                                                    "Computed average of all the PoseStamped objects."),
    BT::InputPort<std::string>(kPortIDPackageName, "",
                               "Optional package to save the file relative to. If empty, and provided a relative file_path, will assume the file is to be saved in the objectives directory."),
    BT::InputPort<std::string>(kPortIDFilePath, "calibration_pose.yaml", "Path at which to save the file.")
  });
}

BT::KeyValueVector SaveCalibrationPoseYaml::metadata()
{
  return { {"description", "Calibrate scene camera extrinsics"} };
}

BT::NodeStatus SaveCalibrationPoseYaml::tick()
{
  const auto ports = moveit_studio::behaviors::getRequiredInputs(
    getInput<geometry_msgs::msg::PoseStamped>(kPortIDCalibrationPoseStamped), getInput<std::string>(kPortIDFilePath));

  if (!ports.has_value()) {
    shared_resources_->logger->publishFailureMessage(name(), fmt::format("Missing input port: {}", ports.error()));
    return BT::NodeStatus::FAILURE;
  }

  const auto& [pose_stamped, file_path] = ports.value();
  const auto& package_name_maybe = getInput<std::string>(kPortIDPackageName);

  // Convert from quaternion to RPY.
  tf2::Quaternion quat(pose_stamped.pose.orientation.x, pose_stamped.pose.orientation.y,
                       pose_stamped.pose.orientation.z, pose_stamped.pose.orientation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  // Create the YAML to save.
  YAML::Node node;
  std::string calibration_data_str = "calibration_data";
  node[calibration_data_str]["x"] = pose_stamped.pose.position.x;
  node[calibration_data_str]["y"] = pose_stamped.pose.position.y;
  node[calibration_data_str]["z"] = pose_stamped.pose.position.z;
  node[calibration_data_str]["roll"] = roll;
  node[calibration_data_str]["pitch"] = pitch;
  node[calibration_data_str]["yaw"] = yaw;

  // Attempt to save the file.
  const std::vector<std::string> objective_library_directories = shared_resources_->node->get_parameter("objective_library_directories").as_string_array();
  auto filepath_maybe = moveit_studio::common::filesystem_utils::getFilePath(
    file_path, objective_library_directories[0],
    /*must_exist=*/false, package_name_maybe.value());
  if (!filepath_maybe)
  {
    shared_resources_->logger->publishFailureMessage(
      name(), fmt::format("Filepath '{}' could not be resolved. Error: '{}'", file_path, filepath_maybe.error()));
    return BT::NodeStatus::FAILURE;
  }

  shared_resources_->logger->publishInfoMessage(fmt::format("Writing calibration file to '{}'", filepath_maybe.value().string()));
  std::ofstream file_out(filepath_maybe.value());
  file_out << node;
  file_out.close();

  return BT::NodeStatus::SUCCESS;
}

}  // namespace calibrate_camera_pose
