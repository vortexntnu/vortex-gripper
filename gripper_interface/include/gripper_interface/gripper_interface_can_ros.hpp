#ifndef GRIPPER_INTERFACE__GRIPPER_INTERFACE_ROS_HPP_
#define GRIPPER_INTERFACE__GRIPPER_INTERFACE_ROS_HPP_

#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <vortex_msgs/msg/gripper_state.hpp>
#include <vortex_msgs/msg/gripper_state_velocity_command.hpp>
#include <vortex/utils/ros/qos_profiles.hpp>

#include "gripper_interface/gripper_can_driver.hpp"
#include "gripper_interface/gripper_interface_translator.hpp"
#include "gripper_interface/gripper_interface_typedefs.hpp"

// ---------------------------------------------------------------------------
// Responsibility: ROS wiring only.
//   - Polls CAN for encoder frames on a timer → publishes GripperState
//   - Subscribes to GripperStateVelocityCommand → converts → send_pwm over CAN
//
// The node is the glue between:
//   GripperReferenceFilterNode  ──(GripperReferenceFilter)──►
//   GripperControllerNode       ──(GripperStateVelocityCommand)──►
//   GripperInterfaceNode        ──(CAN SET_PWM)──► MCU
//
//   MCU ──(CAN SEND_ANGLES)──► GripperInterfaceNode ──(GripperState)──►
//   GripperReferenceFilterNode + GripperControllerNode
// ---------------------------------------------------------------------------

namespace vortex::interface {

class GripperInterfaceNode : public rclcpp::Node {
public:
    explicit GripperInterfaceNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~GripperInterfaceNode() override;

private:
    // Setup
    void declare_and_load_parameters();
    void setup_pub_sub();
    void init_can_driver();

    // Timer: polls CAN socket for encoder frames, publishes GripperState.
    // Rate is independent from the controller rate — MCU sends at its own TC0
    // timer rate (~1 Hz based on TC0 period of 46874 @ DIV1024 / 48 MHz).
    // The ROS poll timer runs faster to catch frames promptly.
    void encoder_poll_callback();

    // Velocity command from GripperControllerNode → CAN SET_PWM.
    void velocity_command_callback(
        const vortex_msgs::msg::GripperStateVelocityCommand::SharedPtr msg);

    // ---------------------------------------------------------------------------
    // ROS interfaces
    // ---------------------------------------------------------------------------
    rclcpp::Publisher<vortex_msgs::msg::GripperState>::SharedPtr state_pub_;

    rclcpp::Subscription<vortex_msgs::msg::GripperStateVelocityCommand>::SharedPtr
        velocity_cmd_sub_;

    rclcpp::TimerBase::SharedPtr encoder_poll_timer_;

    // ---------------------------------------------------------------------------
    // Hardware
    // ---------------------------------------------------------------------------
    std::unique_ptr<GripperCanDriver> can_driver_;

    // ---------------------------------------------------------------------------
    // Parameters (loaded once at construction)
    // ---------------------------------------------------------------------------
    std::string can_interface_;
    std::chrono::milliseconds poll_period_;
};

}  // namespace vortex::interface

#endif  // GRIPPER_INTERFACE__GRIPPER_INTERFACE_ROS_HPP_
