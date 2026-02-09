#include "vortex-gripper/gripper_controller_ros.hpp" 

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv); 
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Started Gripper Controller Node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
