#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <vortex_msgs/msg/gripper_state.hpp>
#include <vortex_msgs/msg/gripper_state_velocity_command.hpp>
#include <vortex/utils/ros/qos_profiles.hpp>
#include "gripper_interface/can_interface.hpp"
#include "gripper_interface/gripper_interface_translator.hpp"

#include <mutex>
#include <string>

namespace vortex::interface {

class GripperInterfaceNode : public rclcpp::Node {
public:
    explicit GripperInterfaceNode(const rclcpp::NodeOptions& options);
    ~GripperInterfaceNode() override;

private:
    void declare_and_load_parameters();
    void init_can();
    void setup_pub_sub();

    void on_can_frame(const struct canfd_frame& frame, can_status status);
    void velocity_command_callback(
        const vortex_msgs::msg::GripperStateVelocityCommand::SharedPtr msg);

    can_interface can_;
    std::string can_interface_name_;
    std::mutex pub_mutex_;

    rclcpp::Publisher<vortex_msgs::msg::GripperState>::SharedPtr state_pub_;
    rclcpp::Subscription<vortex_msgs::msg::GripperStateVelocityCommand>::SharedPtr velocity_cmd_sub_;
};

}  // namespace vortex::interface
