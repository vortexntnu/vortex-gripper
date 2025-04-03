#include "gripper_interface/gripper_interface_ros.hpp"

GripperInterfaceNode::GripperInterfaceNode() : Node("gripper_interface_node") {
    extract_parameters();

    // needed for subscriber and publisher
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos_sensor_data = rclcpp::QoS(
        rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        joy_topic_, qos_sensor_data,
        std::bind(&GripperInterfaceNode::joy_callback, this,
                  std::placeholders::_1));
    ref_pub_ =
        this->create_publisher<std_msgs::msg::Float64MultiArray>(ref_topic_, 10);
    
    uin_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        uin_topic_, qos_sensor_data,
        std::bind(&GripperInterfaceNode::pwm_callback, this,
                    std::placeholders::_1));
    pwm_pub_ =
        this->create_publisher<std_msgs::msg::Int16MultiArray>(pwm_topic_, 10);
    
    gripper_driver_ = std::make_unique<GripperInterfaceDriver>(
        i2c_bus_, i2c_address_, pwm_gain_, pwm_idle_);

    RCLCPP_INFO(this->get_logger(), "Gripper interface node started.");
}

void GripperInterfaceNode::extract_parameters() {
    this->declare_parameter<std::string>("topics.joy");
    this->declare_parameter<std::string>("topics.ref");
    this->declare_parameter<std::string>("topics.uin");
    this->declare_parameter<std::string>("topics.pwm");
    this->declare_parameter<int>("pwm.gain");
    this->declare_parameter<int>("pwm.idle");
    this->declare_parameter<int>("i2c.bus");
    this->declare_parameter<int>("i2c.address");

    this->joy_topic_ = this->get_parameter("topics.joy").as_string();
    this->ref_topic_ = this->get_parameter("topics.ref").as_string();
    this->uin_topic_ = this->get_parameter("topics.uin").as_string();
    this->pwm_topic_ = this->get_parameter("topics.pwm").as_string();
    this->pwm_gain_ = this->get_parameter("pwm.gain").as_int();
    this->pwm_idle_ = this->get_parameter("pwm.idle").as_int();
    this->i2c_bus_ = this->get_parameter("i2c.bus").as_int();
    this->i2c_address_ = this->get_parameter("i2c.address").as_int();
}

void GripperInterfaceNode::joy_callback(
    const sensor_msgs::msg::Joy::SharedPtr msg) {
    std::vector<double> ref_values;
    double shoulder_ref = msg->axes[1];
    double wrist_ref = msg->axes[0];
    double grip_ref = msg->axes[3];

    ref_values.push_back(shoulder_ref);
    ref_values.push_back(wrist_ref);
    ref_values.push_back(grip_ref);

    std_msgs::msg::Float64MultiArray ref_msg = vec_to_msg(ref_values);
    ref_pub_->publish(ref_msg);
}

void GripperInterfaceNode::pwm_callback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    std::vector<std::uint16_t> pwm_values;
    std::transform(msg->data.begin(), msg->data.end(), std::back_inserter(pwm_values),
                   [this](double value) { return gripper_driver_->u_to_pwm(value); });
    std_msgs::msg::Int16MultiArray pwm_msg = vec_to_msg(pwm_values);
    pwm_pub_->publish(pwm_msg);
    gripper_driver_->send_pwm(pwm_values);
}

std_msgs::msg::Int16MultiArray GripperInterfaceNode::vec_to_msg(
    std::vector<std::uint16_t> vec) {
    std_msgs::msg::Int16MultiArray msg;
    for (std::uint16_t value : vec) {
        msg.data.push_back(value);
    }
    return msg;
}

std_msgs::msg::Float64MultiArray GripperInterfaceNode::vec_to_msg(
    std::vector<double> vec) {
    std_msgs::msg::Float64MultiArray msg;
    for (double value : vec) {
        msg.data.push_back(value);
    }
    return msg;
}