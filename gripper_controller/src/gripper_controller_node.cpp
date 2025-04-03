
#include "gripper_controller/gripper_controller_ros.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Started gripper_controller_node");
    rclcpp::spin(std::make_shared<GripperControllerNode>());
    rclcpp::shutdown();
    return 0;
}