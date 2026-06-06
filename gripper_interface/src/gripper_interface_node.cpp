#include "gripper_interface/gripper_interface_node.hpp"

#include <spdlog/spdlog.h>

#include <cstdint>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

static const char* serial_status_to_string(serial_status status) {
    switch (status) {
        case serial_status::OK:
            return "OK";
        case serial_status::ERR_NOT_INITIALIZED:
            return "ERR_NOT_INITIALIZED";
        case serial_status::ERR_WRITE_FAILED:
            return "ERR_WRITE_FAILED";
        case serial_status::ERR_READ_FAILED:
            return "ERR_READ_FAILED";
        case serial_status::ERR_BAD_PACKET:
            return "ERR_BAD_PACKET";
        default:
            return "UNKNOWN";
    }
}

GripperInterface::GripperInterface() : Node("gripper_interface_node") {
    RCLCPP_INFO(this->get_logger(), "Constructing gripper interface node");

    extract_parameters();

    RCLCPP_INFO(this->get_logger(), "Creating joy subscription on topic: %s",
                joy_topic_.c_str());
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        joy_topic_, 10,
        std::bind(&GripperInterface::joy_callback, this,
                  std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Creating PWM publisher on topic: %s",
                pwm_topic_.c_str());
    pwm_pub_ =
        this->create_publisher<std_msgs::msg::Int16MultiArray>(pwm_topic_, 10);

    RCLCPP_INFO(this->get_logger(), "Creating joint state publisher on topic: %s",
                joint_state_topic_.c_str());
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        joint_state_topic_, 10);

    RCLCPP_INFO(this->get_logger(),
                "Creating gripper driver: port=%s baud=%u pwm_gain=%d pwm_idle=%d",
                serial_port_.c_str(),
                serial_baudrate_,
                pwm_gain_,
                pwm_idle_);

    gripper_driver_ = std::make_unique<GripperInterfaceDriver>(
        asio_io_,
        serial_port_,
        serial_baudrate_,
        pwm_gain_,
        pwm_idle_);

    const auto init_status = gripper_driver_->init_serial();

    if (init_status != serial_status::OK) {
        RCLCPP_FATAL(this->get_logger(),
                     "Failed to initialize gripper serial interface on %s: %s",
                     serial_port_.c_str(),
                     serial_status_to_string(init_status));

        throw std::runtime_error("Failed to initialize gripper serial interface");
    }

    RCLCPP_INFO(this->get_logger(),
                "Successfully initialized serial interface on %s",
                serial_port_.c_str());

    gripper_driver_->start_read_encoders(
        [this](const std::vector<double>& angles, serial_status status) {
            this->encoder_angles_callback(angles, status);
        });

    RCLCPP_INFO(this->get_logger(), "Starting Boost.Asio IO thread");

    asio_thread_ = std::thread([this]() {
        RCLCPP_INFO(this->get_logger(), "Boost.Asio IO thread entered run()");
        asio_io_.run();
        RCLCPP_WARN(this->get_logger(), "Boost.Asio IO thread exited run()");
    });

    RCLCPP_INFO(this->get_logger(), "Gripper interface node started");
}

GripperInterface::~GripperInterface() {
    RCLCPP_INFO(this->get_logger(), "Destroying gripper interface node");

    asio_io_.stop();

    if (asio_thread_.joinable()) {
        RCLCPP_INFO(this->get_logger(), "Joining Boost.Asio IO thread");
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

    RCLCPP_INFO(this->get_logger(), "Loaded parameters:");
    RCLCPP_INFO(this->get_logger(), "  topics.joy         = %s", joy_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  topics.pwm         = %s", pwm_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  topics.joint_state = %s", joint_state_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  pwm.gain           = %d", pwm_gain_);
    RCLCPP_INFO(this->get_logger(), "  pwm.idle           = %d", pwm_idle_);
    RCLCPP_INFO(this->get_logger(), "  serial.port        = %s", serial_port_.c_str());
    RCLCPP_INFO(this->get_logger(), "  serial.baudrate    = %u", serial_baudrate_);
}

void GripperInterface::joy_callback(
    const sensor_msgs::msg::Joy::SharedPtr msg) {
    constexpr std::size_t shoulder_axis = 0;
    constexpr std::size_t wrist_axis = 1;

    constexpr std::size_t start_button = 0;
    constexpr std::size_t stop_button = 1;

    RCLCPP_DEBUG(this->get_logger(),
                 "Joy callback received: axes=%zu buttons=%zu",
                 msg->axes.size(),
                 msg->buttons.size());

    if (msg->axes.size() < 2) {
        RCLCPP_WARN(this->get_logger(),
                    "Joy message does not contain enough axes: got %zu, need at least 2",
                    msg->axes.size());
        return;
    }

    if (msg->buttons.size() < 2) {
        RCLCPP_WARN(this->get_logger(),
                    "Joy message does not contain enough buttons: got %zu, need at least 2",
                    msg->buttons.size());
        return;
    }

    const double shoulder_value = msg->axes[shoulder_axis];
    const double wrist_value = msg->axes[wrist_axis];

    constexpr std::uint16_t neutral_pwm = 1500;

    std::vector<std::uint16_t> pwm_values;
    pwm_values.reserve(3);

    pwm_values.push_back(gripper_driver_->joy_to_pwm(shoulder_value));
    pwm_values.push_back(gripper_driver_->joy_to_pwm(wrist_value));
    pwm_values.push_back(neutral_pwm);

    RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        500,
        "Joy axes: shoulder=%.3f wrist=%.3f grip=neutral -> PWM: %u %u %u",
        shoulder_value,
        wrist_value,
        pwm_values[0],
        pwm_values[1],
        pwm_values[2]);

    std_msgs::msg::Int16MultiArray pwm_msg = vec_to_msg(pwm_values);
    pwm_pub_->publish(pwm_msg);

    const auto pwm_status = gripper_driver_->send_pwm(pwm_values);

    if (pwm_status != serial_status::OK) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            500,
            "send_pwm failed with status: %s",
            serial_status_to_string(pwm_status));
    } else {
        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "send_pwm OK");
    }

    if (msg->buttons[start_button]) {
        RCLCPP_INFO(this->get_logger(), "START button pressed");

        const auto start_status = gripper_driver_->start_gripper();

        if (start_status != serial_status::OK) {
            RCLCPP_WARN(this->get_logger(),
                        "start_gripper failed with status: %s",
                        serial_status_to_string(start_status));
        } else {
            RCLCPP_INFO(this->get_logger(), "start_gripper OK");
        }
    } else if (msg->buttons[stop_button]) {
        RCLCPP_INFO(this->get_logger(), "STOP button pressed");

        const auto stop_status = gripper_driver_->stop_gripper();

        if (stop_status != serial_status::OK) {
            RCLCPP_WARN(this->get_logger(),
                        "stop_gripper failed with status: %s",
                        serial_status_to_string(stop_status));
        } else {
            RCLCPP_INFO(this->get_logger(), "stop_gripper OK");
        }
    }
}

void GripperInterface::encoder_angles_callback(
    const std::vector<double>& angles,
    serial_status status) {
    if (status != serial_status::OK) {
        RCLCPP_WARN(this->get_logger(),
                    "Encoder read failed with status: %s",
                    serial_status_to_string(status));
        return;
    }

    if (angles.empty()) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Received empty encoder angle vector");
        return;
    }

    RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "Received encoder angles: count=%zu first=%.4f second=%.4f",
        angles.size(),
        angles.size() > 0 ? angles[0] : 0.0,
        angles.size() > 1 ? angles[1] : 0.0);

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
