#include "joy_to_pwm/gripper_interface.hpp"

GripperInterface::GripperInterface()
    : Node("gripper_interface") {
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&GripperInterface::joy_callback, this, std::placeholders::_1)
        );
        i2c_bus_ = 1;
        i2c_address_ = 0x21;
        gripper_driver_ = std::make_unique<GripperInterfaceDriver>();
        gripper_driver_->joy_gain_ = 200;
    }

void GripperInterface::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    double shoulder_value = msg->axes[1];
    double wrist_value = msg->axes[0];
    double grip_value = msg->axes[3];

    std::uint16_t shoulder_pwm = gripper_driver_->joy_to_pwm(shoulder_value);
    std::uint16_t wrist_pwm = gripper_driver_->joy_to_pwm(wrist_value);
    std::uint16_t grip_pwm = gripper_driver_->joy_to_pwm(grip_value);

    RCLCPP_INFO(this->get_logger(), "Shoulder: %d, Wrist: %d, Grip: %d", shoulder_pwm, wrist_pwm, grip_pwm);
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GripperInterface>());
    rclcpp::shutdown();
    return 0;
}
