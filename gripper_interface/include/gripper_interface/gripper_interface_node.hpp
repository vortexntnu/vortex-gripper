#ifndef GRIPPER_INTERFACE_HPP
#define GRIPPER_INTERFACE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include "gripper_interface/gripper_interface_driver.hpp"
#include <chrono>

class GripperInterface : public rclcpp::Node {
   public:
    GripperInterface();

   private:
    /**
     * @brief Extract parameters from the config file.
     */
    void extract_parameters();
    
    void set_publisher_and_subsribers();

    /**
     * @brief Callback function for the joystick message.
     * @param msg The joystick message.
     */
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

    void encoder_angles_callback();

    /**
     * @brief Convert a vector of PWM values to a ROS message.
     * @param vec The vector of PWM values.
     * @return The ROS message.
     */
    std_msgs::msg::Int16MultiArray vec_to_msg(std::vector<std::uint16_t> vec);

    std::string joy_topic_;
    std::string pwm_topic_;
    int i2c_bus_;
    int i2c_address_;
    int pwm_gain_;
    int pwm_idle_;

    std::unique_ptr<GripperInterfaceDriver> gripper_driver_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr angle_pub_;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr pwm_pub_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr encoder_angles_pub_;
    rclcpp::Time last_msg_time_;
};

#endif  // GRIPPER_INTERFACE_HPP
