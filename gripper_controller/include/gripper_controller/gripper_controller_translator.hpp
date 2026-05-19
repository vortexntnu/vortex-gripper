#ifndef GRIPPER_CONTROLLER__GRIPPER_CONTROLLER_TRANSLATOR_HPP_
#define GRIPPER_CONTROLLER__GRIPPER_CONTROLLER_TRANSLATOR_HPP_

#include <vortex_msgs/msg/gripper_reference_filter.hpp>
#include <vortex_msgs/msg/gripper_state.hpp>
#include <vortex_msgs/msg/gripper_state_velocity_command.hpp>
#include "gripper_controller/gripper_controller_typedefs.hpp"

// ---------------------------------------------------------------------------
// Responsibility: translate between domain structs and ROS message types.
// This is intentionally a namespace of stateless free functions rather than
// a class — there is no invariant to maintain, no state to encapsulate.
// Keeping translation logic here (instead of inside the node) satisfies SRP:
// the node only orchestrates, the translator only converts.
// ---------------------------------------------------------------------------

namespace gripper_controller::translator {

/// @brief Extract position-only GripperState from a smoothed reference message.
inline types::GripperState reference_filter_msg_to_gripper_state(
  const vortex_msgs::msg::GripperReferenceFilter& reference_filter_msg) {
  types::GripperState gripper_state;
  gripper_state.roll = reference_filter_msg.roll;
  gripper_state.pinch = reference_filter_msg.pinch;
  return gripper_state;
}

/// @brief Convert a raw GripperState ROS message to the domain GripperState struct.
inline types::GripperState gripper_state_msg_to_gripper_state(
  const vortex_msgs::msg::GripperState& gripper_state_msg) {
  types::GripperState gripper_state;
  gripper_state.roll = gripper_state_msg.roll;
  gripper_state.pinch = gripper_state_msg.pinch;
  return gripper_state;
}

/// @brief Pack a 2D velocity command vector into a GripperStateVelocityCommand message.
inline vortex_msgs::msg::GripperStateVelocityCommand
velocity_command_to_gripper_velocity_command_msg(
  const types::Vector2d& velocity_command) {
  vortex_msgs::msg::GripperStateVelocityCommand velocity_command_msg;
  velocity_command_msg.roll_velocity = velocity_command(0);
  velocity_command_msg.pinch_velocity = velocity_command(1);
  return velocity_command_msg;
}

} // namespace gripper_controller::translator

#endif // GRIPPER_CONTROLLER__GRIPPER_CONTROLLER_TRANSLATOR_HPP_
