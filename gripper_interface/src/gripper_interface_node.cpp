#include <rclcpp/rclcpp.hpp>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include "gripper_interface/gripper_interface_ros.hpp"

int main(int argc, char* argv[]) {
    // Force spdlog to stdout at debug level before anything else runs
    spdlog::set_default_logger(spdlog::stdout_color_mt("gripper"));
    spdlog::set_level(spdlog::level::debug);
    spdlog::info("main: starting gripper_interface_node");

    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    auto node = std::make_shared<vortex::interface::GripperInterfaceNode>(options);

    spdlog::info("main: node constructed, spinning");
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
