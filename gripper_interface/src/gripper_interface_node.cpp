#include "gripper_interface/gripper_interface_node.hpp"

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
        i2c_bus_, i2c_address_, pwm_gain_, pwm_idle_);

    watchdog_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&GripperInterface::encoder_angles_callback, this));

    last_msg_time_ = this->now();
    spdlog::info("Gripper interface node started");
}

void GripperInterface::extract_parameters() {
    this->declare_parameter<std::string>("topics.joy");
    this->declare_parameter<std::string>("topics.pwm");
    this->declare_parameter<std::string>("topics.joint_state");
    this->declare_parameter<int>("pwm.gain");
    this->declare_parameter<int>("pwm.idle");
    this->declare_parameter<int>("i2c.bus");
    this->declare_parameter<int>("i2c.address");

    this->joy_topic_ = this->get_parameter("topics.joy").as_string();
    this->pwm_topic_ = this->get_parameter("topics.pwm").as_string();
    this->joint_state_topic_ =
        this->get_parameter("topics.joint_state").as_string();
    this->pwm_gain_ = this->get_parameter("pwm.gain").as_int();
    this->pwm_idle_ = this->get_parameter("pwm.idle").as_int();
    this->i2c_bus_ = this->get_parameter("i2c.bus").as_int();
    this->i2c_address_ = this->get_parameter("i2c.address").as_int();
}

void GripperInterface::joy_callback(
    const sensor_msgs::msg::Joy::SharedPtr msg) {
    std::vector<std::uint16_t> pwm_values;
    double shoulder_value = msg->axes[1];
    double wrist_value = msg->axes[0];
    double grip_value = msg->axes[3];

    std::uint16_t shoulder_pwm = gripper_driver_->joy_to_pwm(shoulder_value);
    std::uint16_t wrist_pwm = gripper_driver_->joy_to_pwm(wrist_value);
    std::uint16_t grip_pwm = gripper_driver_->joy_to_pwm(grip_value);

    pwm_values.push_back(shoulder_pwm);
    pwm_values.push_back(wrist_pwm);
    pwm_values.push_back(grip_pwm);

    std_msgs::msg::Int16MultiArray pwm_msg = vec_to_msg(pwm_values);
    pwm_pub_->publish(pwm_msg);

    gripper_driver_->send_pwm(pwm_values);

    if (msg->buttons[0]) {
        gripper_driver_->start_gripper();
    } else if (msg->buttons[1]) {
        gripper_driver_->stop_gripper();
    }
}

void GripperInterface::encoder_angles_callback() {
    std::vector<double> angles = gripper_driver_->encoder_read();
    if (angles.empty()) {
        return;
    }

    auto joint_state_msg = sensor_msgs::msg::JointState();

    joint_state_msg.header.stamp = this->now();
    joint_state_msg.name = {"shoulder", "wrist", "grip"};
    joint_state_msg.position = angles;

    joint_state_pub_->publish(joint_state_msg);
}

std_msgs::msg::Int16MultiArray GripperInterface::vec_to_msg(
    std::vector<std::uint16_t> vec) {
    std_msgs::msg::Int16MultiArray msg;
    for (std::uint16_t value : vec) {
        msg.data.push_back(value);
    }
    return msg;
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GripperInterface>());
    rclcpp::shutdown();
    return 0;
}
