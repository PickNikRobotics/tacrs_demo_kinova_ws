#include <scale_twist/scale_twist.hpp>

#include "moveit_studio_behavior_interface/get_required_ports.hpp"
#include "moveit_studio_behavior_interface/metadata_fields.hpp"

namespace scale_twist {

constexpr auto kPortIdInputOdometry = "input_odometry";
constexpr auto kPortIdVelocityScale = "velocity_scale";
constexpr auto kPortIdScaledVelocity = "scaled_velocity";

ScaleTwist::ScaleTwist(
    const std::string &name, const BT::NodeConfiguration &config,
    const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>
        &shared_resources)
    : moveit_studio::behaviors::SharedResourcesNode<BT::SyncActionNode>(
          name, config, shared_resources) {}

BT::PortsList ScaleTwist::providedPorts() {
  return {BT::InputPort<nav_msgs::msg::Odometry>(
              kPortIdInputOdometry, "input_odometry",
              "Odometry message containing the velocity."),
          BT::InputPort<double>(kPortIdVelocityScale, "0.8",
                                "Velocity scalar multiplier."),
          BT::OutputPort<geometry_msgs::msg::TwistStamped>(
            kPortIdScaledVelocity, "{scaled_velocity}",
              "Odometry velocity multiplied by the scalar.")};
}

BT::KeyValueVector ScaleTwist::metadata() {
  // TODO(...)
  return {{"description", "Return an odometry twist multiplied by a scalar."}};
}

BT::NodeStatus ScaleTwist::tick() {

  // Extract the input ports.
  const auto ports = moveit_studio::behaviors::getRequiredInputs(
      getInput<nav_msgs::msg::Odometry>(kPortIdInputOdometry),
      getInput<double>(kPortIdVelocityScale) );

  // If a port was set incorrectly, log an error message return FAILURE
  if (!ports.has_value()) {
    shared_resources_->logger->publishFailureMessage(
        name(),
        fmt::format("Failed to get required value from input data port: {}",
                    ports.error()));
    return BT::NodeStatus::FAILURE;
  }

  const auto &[odometry_input, velocity_scale] =
      ports.value();

  // Scale the velocity.
  geometry_msgs::msg::TwistStamped scaled_velocity;
  scaled_velocity.header = odometry_input.header;
  scaled_velocity.twist.linear.x = odometry_input.twist.twist.linear.x * velocity_scale;
  scaled_velocity.twist.linear.y = odometry_input.twist.twist.linear.y * velocity_scale;
  scaled_velocity.twist.linear.z = odometry_input.twist.twist.linear.z * velocity_scale;
  scaled_velocity.twist.angular.x = odometry_input.twist.twist.angular.x * velocity_scale;
  scaled_velocity.twist.angular.y = odometry_input.twist.twist.angular.y * velocity_scale;
  scaled_velocity.twist.angular.z = odometry_input.twist.twist.angular.z * velocity_scale;

  setOutput(kPortIdScaledVelocity, scaled_velocity);


  return BT::NodeStatus::SUCCESS;
}

} // namespace scale_twist
