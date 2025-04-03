#ifndef GRIPPER_INTERFACE_ROS_HPP
#define GRIPPER_INTERFACE_ROS_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "gripper_interface/gripper_interface_driver.hpp"

class GripperInterfaceNode : public rclcpp::Node {
   public:
    GripperInterfaceNode();

   private:
    /**
     * @brief Extract parameters from the config file.
     */
    void extract_parameters();

    /**
     * @brief Callback function for the joystick message.
     * @param msg The joystick message.
     */
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

    /**
     * @brief Callback function for the joystick message.
     * @param msg The input u message.
     */
    void pwm_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    /**
     * @brief Convert a vector of PWM values to a ROS message.
     * @param vec The vector of PWM values.
     * @return The ROS message.
     */
    std_msgs::msg::Int16MultiArray vec_to_msg(std::vector<std::uint16_t> vec);
    std_msgs::msg::Float64MultiArray vec_to_msg(std::vector<double> vec);

    std::string joy_topic_;
    std::string ref_topic_;
    std::string uin_topic_;
    std::string pwm_topic_;
    int i2c_bus_;
    int i2c_address_;
    int pwm_gain_;
    int pwm_idle_;
    
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ref_pub_;
    
    //CONTROLLER NODE IN BETWEEN HERE
    //rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr ref_sub_;
    //rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr uin_pub_;
    
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr uin_sub_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr pwm_pub_;

    std::unique_ptr<GripperInterfaceDriver> gripper_driver_;
};

#endif  // GRIPPER_INTERFACE_ROS_HPP
