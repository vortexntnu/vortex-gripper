#include "gripper_interface/gripper_interface_ros.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Started gripper_interface_node");
    rclcpp::spin(std::make_shared<GripperInterfaceNode>());
    rclcpp::shutdown();
    return 0;
}