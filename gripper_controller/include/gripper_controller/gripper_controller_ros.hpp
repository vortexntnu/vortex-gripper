#ifndef GRIPPER_CONTROLLER_ROS_HPP
#define GRIPPER_CONTROLLER_ROS_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "gripper_controller/gripper_controller_driver.hpp"

class GripperControllerNode : public rclcpp::Node {
public:
    GripperControllerNode();

private:
    void extract_parameters();
    void ref_callback(const std_msgs::msg::Float64MultiArray::SharedPtr ref);
    void pos_callback(const std_msgs::msg::Float64MultiArray::SharedPtr pos);
    void control_loop();
    
    std_msgs::msg::Float64MultiArray vec_to_msg(std::vector<double> vec);
    std::vector<double> msg_to_vec(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    
    std::string ref_topic_;
    std::string pos_topic_;
    std::string uin_topic_;
    
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr ref_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr pos_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr uin_pub_;
    
    std::unique_ptr<GripperControllerDriver> gripper_controller_;
};

#endif // GRIPPER_CONTROLLER_ROS_HPP