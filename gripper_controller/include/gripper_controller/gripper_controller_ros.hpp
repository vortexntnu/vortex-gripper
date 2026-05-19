#ifndef GRIPPER_CONTROLLER__GRIPPER_CONTROLLER_ROS_HPP_
#define GRIPPER_CONTROLLER__GRIPPER_CONTROLLER_ROS_HPP_

#include <chrono>
#include <functional>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <vortex_msgs/msg/gripper_reference_filter.hpp>
#include <vortex_msgs/msg/gripper_state.hpp>
#include <vortex_msgs/msg/gripper_state_velocity_command.hpp>
#include <vortex/utils/ros/qos_profiles.hpp>
#include "gripper_controller/gripper_controller.hpp"
#include "gripper_controller/gripper_controller_translator.hpp"
#include "gripper_controller/gripper_controller_typedefs.hpp"

// ---------------------------------------------------------------------------
// Responsibility: ROS wiring only — subscriptions, publications, parameter
// loading, and timer setup. All control mathematics live in GripperController.
// All message translation lives in gripper_controller::translator.
// ---------------------------------------------------------------------------

namespace vortex::controller {

class GripperControllerNode : public rclcpp::Node {
public:
  explicit GripperControllerNode(const rclcpp::NodeOptions & options);

private:
  void reference_callback(
    const vortex_msgs::msg::GripperReferenceFilter::SharedPtr reference_msg);

  void state_callback(
    const vortex_msgs::msg::GripperState::SharedPtr state_msg);

  void publish_control();

  void set_controller_params();

  void set_subscribers_and_publisher();

  GripperController controller_;

  rclcpp::Subscription<vortex_msgs::msg::GripperReferenceFilter>::SharedPtr
    reference_sub_;
  rclcpp::Subscription<vortex_msgs::msg::GripperState>::SharedPtr
    state_sub_;
  rclcpp::Publisher<vortex_msgs::msg::GripperStateVelocityCommand>::SharedPtr
    control_pub_;

  rclcpp::TimerBase::SharedPtr control_timer_;
  std::chrono::milliseconds time_step_ms_;

  std::mutex state_mutex_;

  double roll_ref_ = 0.0;
  double pinch_ref_ = 0.0;
  double roll_measured_ = 0.0;
  double pinch_measured_ = 0.0;
};

} // namespace vortex::controller

#endif // GRIPPER_CONTROLLER__GRIPPER_CONTROLLER_ROS_HPP_
