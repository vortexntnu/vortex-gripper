#include "gripper_controller/gripper_controller_ros.hpp"

GripperControllerNode::GripperControllerNode() : Node ("gripper_controller_node") {
    this->extract_parameters();

    // needed for subscriber and publisher
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos_sensor_data = rclcpp::QoS(
        rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

    pos_sub_ =
        this->create_subscription<std_msgs::msg::Float64MultiArray>(
            pos_topic_, qos_sensor_data,
            std::bind(&GripperControllerNode::pos_callback, this,
                      std::placeholders::_1));
    ref_sub_ =
        this->create_subscription<std_msgs::msg::Float64MultiArray>(
            ref_topic_, qos_sensor_data,
            std::bind(&GripperControllerNode::ref_callback, this,
                      std::placeholders::_1));

    uin_pub_ =
        this->create_publisher<std_msgs::msg::Float64MultiArray>(
            uin_topic_, qos_sensor_data);

    gripper_controller_ = std::make_unique<GripperControllerDriver>();
    
    RCLCPP_INFO(this->get_logger(),
                "\"gripper_controller_node\" correctly initialized");
}

void GripperControllerNode::extract_parameters() {
    this->declare_parameter<std::string>("topics.pos");
    this->declare_parameter<std::string>("topics.ref");
    this->declare_parameter<std::string>("topics.uin");

    this->pos_topic_ = this->get_parameter("topics.pos").as_string();
    this->ref_topic_ = this->get_parameter("topics.ref").as_string();
    this->uin_topic_ = this->get_parameter("topics.uin").as_string();
}

void GripperControllerNode::pos_callback(const std_msgs::msg::Float64MultiArray::SharedPtr pos) {
    this->gripper_controller_->pos = this->msg_to_vec(pos);
    this->control_loop();
}

void GripperControllerNode::ref_callback(const std_msgs::msg::Float64MultiArray::SharedPtr ref) {
    this->gripper_controller_->ref = this->msg_to_vec(ref);
    this->control_loop();
}

void GripperControllerNode::control_loop() {
    gripper_controller_->update();
    std::vector<double> uin = gripper_controller_->compute();
    uin_pub_->publish(this->vec_to_msg(uin));
}

std_msgs::msg::Float64MultiArray GripperControllerNode::vec_to_msg(
    std::vector<double> vec) {
    std_msgs::msg::Float64MultiArray msg;
    for (double value : vec) {
        msg.data.push_back(value);
    }
    return msg;
}

std::vector<double> GripperControllerNode::msg_to_vec(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    std::vector<double> vec;
    for (double value : msg->data) {
        vec.push_back(value);
    }
    return vec;
}