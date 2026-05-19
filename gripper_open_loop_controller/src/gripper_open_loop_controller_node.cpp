#include <rclcpp/rclcpp.hpp>
#include "gripper_open_loop_controller/gripper_open_loop_controller_ros.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<vortex::controller::GripperOpenLoopControllerNode>(
        rclcpp::NodeOptions());

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
