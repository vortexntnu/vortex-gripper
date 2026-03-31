#include <rclcpp/rclcpp.hpp>
#include "gripper_controller/gripper_controller_ros.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto gripper_controller_node =
    std::make_shared<vortex::controller::GripperControllerNode>(
      rclcpp::NodeOptions());

  rclcpp::spin(gripper_controller_node);
  rclcpp::shutdown();
  return 0;
}
