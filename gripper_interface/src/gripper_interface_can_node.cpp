#include <rclcpp/rclcpp.hpp>
#include "gripper_interface/gripper_interface_ros.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<vortex::interface::GripperInterfaceNode>(
        rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}
