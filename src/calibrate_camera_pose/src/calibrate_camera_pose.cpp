#include <calibrate_camera_pose/calibrate_camera_pose.hpp>

#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <moveit_studio_behavior_interface/get_required_ports.hpp>

namespace
{
  constexpr auto kPortIDDetectedPoses = "detected_poses";
  constexpr auto kPortIDCalibrationToolFrame = "calibration_tool_frame";
  constexpr auto kPortIDBaseFrame = "base_link";
  constexpr auto kPortIDComputedPose = "computed_pose";
}

namespace calibrate_camera_pose
{
  CalibrateCameraPose::CalibrateCameraPose(const std::string &name, const BT::NodeConfiguration &config,
                                           const std::shared_ptr<moveit_studio::behaviors::BehaviorContext> &
                                           shared_resources)
    : moveit_studio::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}


BT::PortsList CalibrateCameraPose::providedPorts()
{
  return BT::PortsList({
    BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>(kPortIDDetectedPoses, "{detected_poses}",
                                                                 "Vector of the detected poses."),
    BT::InputPort<std::string>(kPortIDCalibrationToolFrame, "calibration_tool_link",
                               "Frame of the calibration tool. Must be a part of the robot kinematic chain."),
    BT::InputPort<std::string>(kPortIDBaseFrame, "base_link",
                               "Frame the output computed pose should be represented in. Must be a part of the robot kinematic chain."),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>(kPortIDComputedPose, "{computed_pose}",
                                                    "Computed pose of the camera.")
  });
}

BT::KeyValueVector CalibrateCameraPose::metadata()
{
  return { {"description", "Calibrate scene camera extrinsics"} };
}

BT::NodeStatus CalibrateCameraPose::tick()
{
  const auto ports = moveit_studio::behaviors::getRequiredInputs(
    getInput<std::vector<geometry_msgs::msg::PoseStamped> >(kPortIDDetectedPoses),
    getInput<std::string>(kPortIDCalibrationToolFrame), getInput<std::string>(kPortIDBaseFrame));

  if (!ports.has_value()) {
    shared_resources_->logger->publishFailureMessage(name(), fmt::format("Missing input port: {}", ports.error()));
    return BT::NodeStatus::FAILURE;
  }

  const auto& [detected_poses, calibration_tool_frame, base_frame] = ports.value();

  if (detected_poses.size() != 3) {
    shared_resources_->logger->publishFailureMessage(
      name(), fmt::format("`detected_poses` input port should have size of 3, but has size of {}",
                          detected_poses.size()));
    return BT::NodeStatus::FAILURE;
  }

  // Convert points to Eigen for easier computation.
  Eigen::Vector3d point1(detected_poses[0].pose.position.x, detected_poses[0].pose.position.y,
                         detected_poses[0].pose.position.z);
  Eigen::Vector3d point2(detected_poses[1].pose.position.x, detected_poses[1].pose.position.y,
                         detected_poses[1].pose.position.z);
  Eigen::Vector3d point3(detected_poses[2].pose.position.x, detected_poses[2].pose.position.y,
                         detected_poses[2].pose.position.z);

  // Compute center point of the 3 poses.
  Eigen::Vector3d center_point = (point1 + point2 + point3) / 3.0;

  // Compute the vectors of the triangle sides.
  Eigen::Vector3d v1 = point1 - point2;
  Eigen::Vector3d v2 = point3 - point1;

  // Compute y axis as the vector between point1 & point2.
  Eigen::Vector3d y_axis(v1.normalized());

  // Compute z axis as the triangle normal.
  Eigen::Vector3d z_axis(v1.cross(v2).normalized());

  // Compute the x axis as the cross product between the y & z axes.
  Eigen::Vector3d x_axis(y_axis.cross(z_axis).normalized());

  // Convert the pose into an Eigen::Isometry3d for computations & to get the quaternion orientation.
  Eigen::Isometry3d detected_pose = Eigen::Isometry3d::Identity();
  detected_pose.linear().col(0) = x_axis;
  detected_pose.linear().col(1) = y_axis;
  detected_pose.linear().col(2) = z_axis;
  detected_pose.translation() = center_point;

  if (auto determinant = detected_pose.rotation().determinant();
      abs(determinant - 1) > std::numeric_limits<double>::epsilon())
  {
    shared_resources_->logger->publishFailureMessage(fmt::format(
      "Rotation matrix of computed pose is not normalized. Determinant={}",
      determinant));
    return BT::NodeStatus::FAILURE;
  }

  Eigen::Quaterniond orientation(detected_pose.rotation());

  // TODO: need to grab the base frame & calibration tool frame to transform the frame to the camera pose in the base frame.

  // Create the PoseStamped object to return.
  geometry_msgs::msg::PoseStamped computed_pose;
  computed_pose.header.frame_id = base_frame;
  computed_pose.pose.position.x = center_point(0);
  computed_pose.pose.position.y = center_point(1);
  computed_pose.pose.position.z = center_point(2);
  computed_pose.pose.orientation.w = orientation.w();
  computed_pose.pose.orientation.x = orientation.x();
  computed_pose.pose.orientation.y = orientation.y();
  computed_pose.pose.orientation.z = orientation.z();

  setOutput(kPortIDComputedPose, computed_pose);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace calibrate_camera_pose
