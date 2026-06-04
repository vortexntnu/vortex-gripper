#pragma once

#include "gripper_interface/gripper_interface_driver.hpp"

#include <boost/asio.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>

#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

class GripperInterface : public rclcpp::Node {
public:
    GripperInterface();
    ~GripperInterface() override;

private:
    void extract_parameters();

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

    void encoder_angles_callback(
        const std::vector<double>& angles,
        serial_status status);

    std_msgs::msg::Int16MultiArray vec_to_msg(
        const std::vector<std::uint16_t>& vec);

    std::string joy_topic_;
    std::string pwm_topic_;
    std::string joint_state_topic_;

    int pwm_gain_;
    int pwm_idle_;

    std::string serial_port_;
    unsigned int serial_baudrate_;

    boost::asio::io_context asio_io_;
    std::thread asio_thread_;

    std::unique_ptr<GripperInterfaceDriver> gripper_driver_;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr pwm_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
};
