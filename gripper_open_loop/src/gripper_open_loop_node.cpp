#include <rclcpp/rclcpp.hpp>
#include "gripper_open_loop/gripper_open_loop_ros.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<vortex::open_loop::GripperOpenLoopNode>(
        rclcpp::NodeOptions());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
