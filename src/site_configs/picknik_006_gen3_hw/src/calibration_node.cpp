#include "rclcpp/node.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/logging.hpp>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>

class TransformListenerNode : public rclcpp::Node {
public:
  TransformListenerNode() : Node("calibration_node") {
    this->declare_parameter<std::string>("world_frame", "world");
    this->declare_parameter<std::string>("target_frame", "target_frame");
    this->declare_parameter<std::vector<std::string>>(
        "fiducial_frames", std::vector<std::string>());
    this->declare_parameter<double>("min_contribution", 0.001);

    this->get_parameter("world_frame", world_frame_);
    this->get_parameter("target_frame", target_frame_);
    this->get_parameter("fiducial_frames", fiducial_frames_);
    this->get_parameter("min_contribution", min_contribution_);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&TransformListenerNode::timer_callback, this));
    num_samples_ = 0;

    if (min_contribution_ < 0. || min_contribution_ > 0.01) {
      throw std::runtime_error(
          "min_contribution value must be in the range [0, 0.01]");
    }

    if (fiducial_frames_.empty() || world_frame_.empty() ||
        target_frame_.empty()) {
      throw std::runtime_error(
          "Invalid configuration for calibration node. Fiducial frames, world "
          "frame, and target frame must be non-empty");
    }
    origin_fiducial_frame_ = fiducial_frames_[0];
    for (auto const &frame : fiducial_frames_) {
      avg_pose_diff_.emplace_back();
    }
    running_avg_pose_ = identity_pose();
  }

private:
  geometry_msgs::msg::Pose identity_pose() {
    geometry_msgs::msg::Pose to_return;
    to_return.orientation.w = 1;
    to_return.orientation.x = 0;
    to_return.orientation.y = 0;
    to_return.orientation.z = 0;
    to_return.position.x = 0;
    to_return.position.y = 0;
    to_return.position.z = 0;
    return to_return;
  }
  geometry_msgs::msg::Pose
  applyTransform(const geometry_msgs::msg::TransformStamped &transform,
                 const geometry_msgs::msg::Pose &pose) {
    tf2::Transform tf_transform;
    tf2::fromMsg(transform.transform, tf_transform);

    tf2::Transform tf_pose;
    tf_pose.setOrigin(
        tf2::Vector3(pose.position.x, pose.position.y, pose.position.z));

    tf2::Quaternion q;
    tf2::fromMsg(pose.orientation, q);
    tf_pose.setRotation(q);

    tf2::Transform tf_result = tf_transform * tf_pose;

    geometry_msgs::msg::Pose transformed_pose;
    transformed_pose.position.x = tf_result.getOrigin().x();
    transformed_pose.position.y = tf_result.getOrigin().y();
    transformed_pose.position.z = tf_result.getOrigin().z();
    transformed_pose.orientation.w = tf_result.getRotation().w();
    transformed_pose.orientation.x = tf_result.getRotation().x();
    transformed_pose.orientation.y = tf_result.getRotation().y();
    transformed_pose.orientation.z = tf_result.getRotation().z();

    return transformed_pose;
  }
  Eigen::Quaterniond
  toEigenQuaternion(const geometry_msgs::msg::Quaternion &q) {
    return Eigen::Quaterniond(q.w, q.x, q.y, q.z);
  }

  geometry_msgs::msg::Quaternion toROSQuaternion(const Eigen::Quaterniond &q) {
    geometry_msgs::msg::Quaternion ros_q;
    ros_q.w = q.w();
    ros_q.x = q.x();
    ros_q.y = q.y();
    ros_q.z = q.z();
    return ros_q;
  }
  geometry_msgs::msg::Quaternion
  weightedAverageQuaternion(const geometry_msgs::msg::Quaternion &q1,
                            const geometry_msgs::msg::Quaternion &q2,
                            double weight) {
    Eigen::Quaterniond eig_q1 = toEigenQuaternion(q1);
    Eigen::Quaterniond eig_q2 = toEigenQuaternion(q2);

    weight = std::clamp(weight, 0.0, 1.0);

    Eigen::Quaterniond result = eig_q1.slerp(weight, eig_q2);
    result.normalize();

    return toROSQuaternion(result);
  }
  geometry_msgs::msg::Quaternion
  quaternionDifference(const geometry_msgs::msg::Quaternion &q1,
                       const geometry_msgs::msg::Quaternion &q2) {
    Eigen::Quaterniond eig_q1 = toEigenQuaternion(q1);
    Eigen::Quaterniond eig_q2 = toEigenQuaternion(q2);

    Eigen::Quaterniond q_diff = eig_q1.inverse() * eig_q2;

    return toROSQuaternion(q_diff);
  }
  void print_tf(geometry_msgs::msg::TransformStamped const &tf) {
    RCLCPP_INFO(this->get_logger(),
                "\n\nTransform from %s to %s:", tf.header.frame_id.c_str(),
                tf.child_frame_id.c_str());
    RCLCPP_INFO(this->get_logger(), "Translation -> x: %.6f, y: %.6f, z: %.6f",
                tf.transform.translation.x, tf.transform.translation.y,
                tf.transform.translation.z);
    double y, p, r;
    tf2::getEulerYPR(tf.transform.rotation, y, p, r);
    RCLCPP_INFO(this->get_logger(),
                "Rotation -> yaw: %.6f, pitch: %.6f, roll: %.6f", y, p, r);
  }
  void print_pose(geometry_msgs::msg::Pose const &pose,
                  std::string const &title) {
    RCLCPP_INFO(this->get_logger(), "\n\n%s:", title.c_str());
    RCLCPP_INFO(this->get_logger(), "Translation -> x: %.6f, y: %.6f, z: %.6f",
                pose.position.x, pose.position.y, pose.position.z);
    double y, p, r;
    tf2::getEulerYPR(pose.orientation, y, p, r);
    RCLCPP_INFO(this->get_logger(),
                "Rotation -> yaw: %.6f, pitch: %.6f, roll: %.6f", y, p, r);
  }
  void timer_callback() {
    RCLCPP_INFO_ONCE(this->get_logger(), "Starting calibration capture...");
    std::stringstream s;
    s << "Fiducials of interest: ";
    for (auto const &fiducial_frame : fiducial_frames_) {
      s << fiducial_frame << ", ";
    }
    RCLCPP_INFO_ONCE(this->get_logger(), "%s", s.str().c_str());
    RCLCPP_INFO_ONCE(this->get_logger(), "Parent frame: %s",
                     world_frame_.c_str());
    RCLCPP_INFO_ONCE(this->get_logger(), "Target frame: %s",
                     target_frame_.c_str());
    auto const num_fiducials = fiducial_frames_.size();

    RCLCPP_INFO(this->get_logger(), "\n\n\n\n\n");

    for (std::size_t fiducial_index = 0;
         fiducial_index < fiducial_frames_.size(); ++fiducial_index) {
      auto const &fiducial_frame = fiducial_frames_[fiducial_index];
      try {
        auto const fiducial_target_frame = target_frame_ + "_" + fiducial_frame;
        geometry_msgs::msg::TransformStamped target_to_fiducial =
            tf_buffer_->lookupTransform(fiducial_frame, fiducial_target_frame,
                                        tf2::TimePointZero);

        // measurement of target_frame's position in fiducial's frame
        auto pose_measurement = identity_pose();
        pose_measurement.position.x =
            target_to_fiducial.transform.translation.x;
        pose_measurement.position.y =
            target_to_fiducial.transform.translation.y;
        pose_measurement.position.z =
            target_to_fiducial.transform.translation.z;
        pose_measurement.orientation.w =
            target_to_fiducial.transform.rotation.w;
        pose_measurement.orientation.x =
            target_to_fiducial.transform.rotation.x;
        pose_measurement.orientation.y =
            target_to_fiducial.transform.rotation.y;
        pose_measurement.orientation.z =
            target_to_fiducial.transform.rotation.z;

        geometry_msgs::msg::TransformStamped fiducial_to_origin =
            tf_buffer_->lookupTransform(origin_fiducial_frame_, fiducial_frame,
                                        tf2::TimePointZero);

        auto pose_measurement_origin =
            applyTransform(fiducial_to_origin, pose_measurement);
        // print_pose(pose_measurement, "Pose measurement");
        // print_pose(pose_measurement_origin, "Pose measurement origin");

        // calculate the difference
        auto current_diff = identity_pose();
        current_diff.position.x =
            pose_measurement_origin.position.x - running_avg_pose_.position.x;
        current_diff.position.y =
            pose_measurement_origin.position.y - running_avg_pose_.position.y;
        current_diff.position.z =
            pose_measurement_origin.position.z - running_avg_pose_.position.z;
        current_diff.orientation = quaternionDifference(
            pose_measurement_origin.orientation, running_avg_pose_.orientation);

        // the fraction to update the averages by
        auto const fraction = std::max(
            1. / (static_cast<double>(++num_samples_)), min_contribution_);
        auto const complementary_fraction = (1. - fraction);

        // update the average difference
        auto &avg_pose_diff = avg_pose_diff_[fiducial_index];
        avg_pose_diff.position.x =
            complementary_fraction * avg_pose_diff.position.x +
            fraction * current_diff.position.x;
        avg_pose_diff.position.y =
            complementary_fraction * avg_pose_diff.position.y +
            fraction * current_diff.position.y;
        avg_pose_diff.position.z =
            complementary_fraction * avg_pose_diff.position.z +
            fraction * current_diff.position.z;
        avg_pose_diff.orientation = weightedAverageQuaternion(
            avg_pose_diff.orientation, current_diff.orientation, fraction);

        // update the running average
        // don't let the running average get stuck, each measurement contributes
        // minimum 0.1%
        running_avg_pose_.position.x =
            complementary_fraction * running_avg_pose_.position.x +
            fraction * pose_measurement_origin.position.x;
        running_avg_pose_.position.y =
            complementary_fraction * running_avg_pose_.position.y +
            fraction * pose_measurement_origin.position.y;
        running_avg_pose_.position.z =
            complementary_fraction * running_avg_pose_.position.z +
            fraction * pose_measurement_origin.position.z;
        running_avg_pose_.orientation = weightedAverageQuaternion(
            running_avg_pose_.orientation, pose_measurement_origin.orientation,
            fraction);

            std::stringstream diff_s;
            diff_s << "Current difference from " << fiducial_frame
                   << " target frame to average";
            print_pose(current_diff, diff_s.str());

            std::stringstream avg_diff_s;
            avg_diff_s << "Current difference from " << fiducial_frame
                   << " target frame to average";
            print_pose(avg_pose_diff, avg_diff_s.str());

      } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(),
                    "Calibration node could not get transform: %s", ex.what());
        continue;
      }
    }
    std::stringstream avg_s;
    avg_s << "Running average pose in " << origin_fiducial_frame_ << " frame";
    print_pose(running_avg_pose_, avg_s.str());
  }

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string world_frame_;
  std::string target_frame_;
  std::vector<std::string> fiducial_frames_;
  double min_contribution_;
  std::string origin_fiducial_frame_;
  std::size_t num_samples_;
  // the running avg pose of the target frame in the origin_fiducial_frame_
  geometry_msgs::msg::Pose running_avg_pose_;

  // the average pose difference from running_avg_pose_ of the nth fiducial in
  // the origin_fiducial_frame_
  std::vector<geometry_msgs::msg::Pose> avg_pose_diff_;
};

int main(int argc, char **argv) {
  // Initialize the ROS 2 system
  rclcpp::init(argc, argv);

  // Create and spin the transform listener node
  rclcpp::spin(std::make_shared<TransformListenerNode>());

  // Shutdown ROS 2 system
  rclcpp::shutdown();
  return 0;
}