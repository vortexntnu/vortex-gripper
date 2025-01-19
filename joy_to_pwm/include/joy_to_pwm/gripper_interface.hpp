#ifndef GRIPPER_INTERFACE_HPP
#define GRIPPER_INTERFACE_HPP

#include "joy_to_pwm/gripper_interface_driver.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

class GripperInterface : public rclcpp::Node
{
    public:
        GripperInterface();

    private:
        void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

        short i2c_bus_;
        short i2c_address_;

        std::unique_ptr<GripperInterfaceDriver> gripper_driver_;

        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
};

#endif // GRIPPER_INTERFACE_HPP