#include "gripper_interface/gripper_interface_node.hpp"

#include <spdlog/spdlog.h>

#include <cstdint>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

GripperInterface::GripperInterface() : Node("gripper_interface_node") {
    extract_parameters();

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        joy_topic_, 10,
        std::bind(&GripperInterface::joy_callback, this,
                  std::placeholders::_1));

    pwm_pub_ =
        this->create_publisher<std_msgs::msg::Int16MultiArray>(pwm_topic_, 10);

    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        joint_state_topic_, 10);

    gripper_driver_ = std::make_unique<GripperInterfaceDriver>(
        asio_io_,
        serial_port_,
        serial_baudrate_,
        pwm_gain_,
        pwm_idle_);

    if (gripper_driver_->init_serial() != serial_status::OK) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize gripper serial interface");
        return;
    }

    gripper_driver_->start_read_encoders(
        [this](const std::vector<double>& angles, serial_status status) {
            this->encoder_angles_callback(angles, status);
        });

    asio_thread_ = std::thread([this]() {
        asio_io_.run();
    });

    spdlog::info("Gripper interface node started");
}

GripperInterface::~GripperInterface() {
    asio_io_.stop();

    if (asio_thread_.joinable()) {
        asio_thread_.join();
    }
}

void GripperInterface::extract_parameters() {
    this->declare_parameter<std::string>("topics.joy");
    this->declare_parameter<std::string>("topics.pwm");
    this->declare_parameter<std::string>("topics.joint_state");

    this->declare_parameter<int>("pwm.gain");
    this->declare_parameter<int>("pwm.idle");

    this->declare_parameter<std::string>("serial.port", "/dev/ttyUSB0");
    this->declare_parameter<int>("serial.baudrate", 115200);

    this->joy_topic_ = this->get_parameter("topics.joy").as_string();
    this->pwm_topic_ = this->get_parameter("topics.pwm").as_string();
    this->joint_state_topic_ =
        this->get_parameter("topics.joint_state").as_string();

    this->pwm_gain_ = this->get_parameter("pwm.gain").as_int();
    this->pwm_idle_ = this->get_parameter("pwm.idle").as_int();

    this->serial_port_ = this->get_parameter("serial.port").as_string();
    this->serial_baudrate_ =
        static_cast<unsigned int>(this->get_parameter("serial.baudrate").as_int());
}

void GripperInterface::joy_callback(
    const sensor_msgs::msg::Joy::SharedPtr msg) {
    constexpr std::size_t shoulder_axis = 1;
    constexpr std::size_t wrist_axis = 0;
    constexpr std::size_t grip_axis = 3;

    constexpr std::size_t start_button = 0;
    constexpr std::size_t stop_button = 1;

    if (msg->axes.size() <= grip_axis) {
        RCLCPP_WARN(this->get_logger(), "Joy message does not contain enough axes");
        return;
    }

    if (msg->buttons.size() <= stop_button) {
        RCLCPP_WARN(this->get_logger(), "Joy message does not contain enough buttons");
        return;
    }

    const double shoulder_value = msg->axes[shoulder_axis];
    const double wrist_value = msg->axes[wrist_axis];
    const double grip_value = msg->axes[grip_axis];

    std::vector<std::uint16_t> pwm_values;
    pwm_values.reserve(3);

    pwm_values.push_back(gripper_driver_->joy_to_pwm(shoulder_value));
    pwm_values.push_back(gripper_driver_->joy_to_pwm(wrist_value));
    pwm_values.push_back(gripper_driver_->joy_to_pwm(grip_value));

    std_msgs::msg::Int16MultiArray pwm_msg = vec_to_msg(pwm_values);
    pwm_pub_->publish(pwm_msg);

    if (gripper_driver_->send_pwm(pwm_values) != serial_status::OK) {
        RCLCPP_WARN(this->get_logger(), "Error sending gripper PWM over serial");
    }

    if (msg->buttons[start_button]) {
        if (gripper_driver_->start_gripper() != serial_status::OK) {
            RCLCPP_WARN(this->get_logger(), "Error sending gripper start command");
        }
    } else if (msg->buttons[stop_button]) {
        if (gripper_driver_->stop_gripper() != serial_status::OK) {
            RCLCPP_WARN(this->get_logger(), "Error sending gripper stop command");
        }
    }
}

void GripperInterface::encoder_angles_callback(
    const std::vector<double>& angles,
    serial_status status) {
    if (status != serial_status::OK) {
        RCLCPP_WARN(this->get_logger(), "Encoder read failed");
        return;
    }

    if (angles.empty()) {
        return;
    }

    auto joint_state_msg = sensor_msgs::msg::JointState();

    joint_state_msg.header.stamp = this->now();
    joint_state_msg.name = {"wrist", "grip"};
    joint_state_msg.position = angles;

    joint_state_pub_->publish(joint_state_msg);
}

std_msgs::msg::Int16MultiArray GripperInterface::vec_to_msg(
    const std::vector<std::uint16_t>& vec) {
    std_msgs::msg::Int16MultiArray msg;

    for (std::uint16_t value : vec) {
        msg.data.push_back(static_cast<std::int16_t>(value));
    }

    return msg;
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GripperInterface>());
    rclcpp::shutdown();
    return 0;
}
