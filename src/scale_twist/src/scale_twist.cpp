#include <scale_twist/scale_twist.hpp>

#include "moveit_studio_behavior_interface/get_required_ports.hpp"
#include "moveit_studio_behavior_interface/metadata_fields.hpp"

namespace scale_twist {

constexpr auto kPortIdInputVelocity = "input_velocity";
constexpr auto kPortIdVelocityScale = "velocity_scale";
constexpr auto kPortIdScaledVelocity = "scaled_velocity";

ScaleTwist::ScaleTwist(
    const std::string &name, const BT::NodeConfiguration &config,
    const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>
        &shared_resources)
    : moveit_studio::behaviors::SharedResourcesNode<BT::SyncActionNode>(
          name, config, shared_resources) {}

BT::PortsList ScaleTwist::providedPorts() {
  return {BT::InputPort<geometry_msgs::msg::TwistStamped>(
    kPortIdInputVelocity, "input_velocity",
              "Input velocity."),
          BT::InputPort<double>(kPortIdVelocityScale, "0.8",
                                "Velocity scalar multiplier."),
          BT::OutputPort<geometry_msgs::msg::TwistStamped>(
            kPortIdScaledVelocity, "{scaled_velocity}",
              "Input velocity multiplied by the scalar.")};
}

BT::KeyValueVector ScaleTwist::metadata() {
  // TODO(...)
  return {{"description", "Return a velocity multiplied by a scalar."}};
}

BT::NodeStatus ScaleTwist::tick() {

  // Extract the input ports.
  const auto ports = moveit_studio::behaviors::getRequiredInputs(
      getInput<geometry_msgs::msg::TwistStamped>(kPortIdInputVelocity),
      getInput<double>(kPortIdVelocityScale) );

  // If a port was set incorrectly, log an error message return FAILURE
  if (!ports.has_value()) {
    shared_resources_->logger->publishFailureMessage(
        name(),
        fmt::format("Failed to get required value from input data port: {}",
                    ports.error()));
    return BT::NodeStatus::FAILURE;
  }

  const auto &[velocity_input, velocity_scale] =
      ports.value();

  // Scale the velocity.
  geometry_msgs::msg::TwistStamped scaled_velocity = velocity_input;
  scaled_velocity.twist.linear.x = velocity_input.twist.linear.x * velocity_scale;
  scaled_velocity.twist.linear.y = velocity_input.twist.linear.y * velocity_scale;
  scaled_velocity.twist.linear.z = velocity_input.twist.linear.z * velocity_scale;
  scaled_velocity.twist.angular.x = velocity_input.twist.angular.x * velocity_scale;
  scaled_velocity.twist.angular.y = velocity_input.twist.angular.y * velocity_scale;
  scaled_velocity.twist.angular.z = velocity_input.twist.angular.z * velocity_scale;

  setOutput(kPortIdScaledVelocity, scaled_velocity);


  return BT::NodeStatus::SUCCESS;
}

} // namespace scale_twist
