#include "gripper_controller/gripper_controller_ros.hpp"

#include <chrono>
#include <functional>
#include <mutex>
#include <spdlog/spdlog.h>
#include "gripper_controller/gripper_controller_translator.hpp"

const auto start_message = R"(

 ____       _                          ____            _             _ _
/ ___|_ __ (_)_ __ _ __   ___ _ __   / ___|___  _ __ | |_ _ __ ___ | | | ___ _ __
| |  _| '_ \| | '_ \| '_ \ / _ \ '__|| |   / _ \| '_ \| __| '__/ _ \| | |/ _ \ '__|
| |_| | |_) | | |_) | |_) |  __/ |   | |__| (_) | | | | |_| | | (_) | | |  __/ |
 \____|  .__/|_| .__/| .__/ \___|_|    \____\___/|_| |_|\__|_|  \___/|_|_|\___|_|
        |_|    |_|   |_|

)";

namespace vortex::controller {

// ---------------------------------------------------------------------------
// CONSTRUCTOR
// ---------------------------------------------------------------------------

GripperControllerNode::GripperControllerNode(const rclcpp::NodeOptions & options)
: Node("gripper_controller_node", options) {
  time_step_ = std::chrono::milliseconds(10);

  set_controller_params();

  set_subscribers_and_publisher();

  spdlog::info(start_message);
}

// ---------------------------------------------------------------------------
// SETUP
// ---------------------------------------------------------------------------

void GripperControllerNode::set_controller_params() {
  const int time_step_ms =
    this->declare_parameter<int>("time_step_ms", 10);
  time_step_ = std::chrono::milliseconds(time_step_ms);

  const double kp_roll =
    this->declare_parameter<double>("kp.roll", 1.0);
  const double kp_pinch =
    this->declare_parameter<double>("kp.pinch", 1.0);

  types::Matrix2d proportional_gain_matrix = types::Matrix2d::Zero();
  proportional_gain_matrix(0, 0) = kp_roll;
  proportional_gain_matrix(1, 1) = kp_pinch;

  controller_.set_kp(proportional_gain_matrix);
  controller_.set_time_step(static_cast<double>(time_step_ms) / 1000.0);

  spdlog::info("GripperController: kp_roll={:.3f} kp_pinch={:.3f} dt={}ms",
    kp_roll, kp_pinch, time_step_ms);
}

void GripperControllerNode::set_subscribers_and_publisher() {
  const std::string reference_topic =
    this->declare_parameter<std::string>("topics.reference");
  const std::string state_topic =
    this->declare_parameter<std::string>("topics.state");
  const std::string control_topic =
    this->declare_parameter<std::string>("topics.control");

  auto qos_sensor_data = vortex::utils::qos_profiles::sensor_data_profile(1);

  reference_sub_ =
    this->create_subscription<vortex_msgs::msg::GripperReferenceFilter>(
      reference_topic, qos_sensor_data,
      std::bind(&GripperControllerNode::reference_callback, this,
        std::placeholders::_1));

  state_sub_ =
    this->create_subscription<vortex_msgs::msg::GripperState>(
      state_topic, qos_sensor_data,
      std::bind(&GripperControllerNode::state_callback, this,
        std::placeholders::_1));

  control_pub_ =
    this->create_publisher<vortex_msgs::msg::GripperStateVelocityCommand>(
      control_topic, qos_sensor_data);

  // Timer is created last so that controller_ and the publisher are fully
  // initialised before the first publish_control() fires.
  control_timer_ = this->create_wall_timer(
    time_step_,
    std::bind(&GripperControllerNode::publish_control, this));
}

// ---------------------------------------------------------------------------
// CALLBACKS
// ---------------------------------------------------------------------------

void GripperControllerNode::reference_callback(
  const vortex_msgs::msg::GripperReferenceFilter::SharedPtr reference_msg) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  roll_ref_ = reference_msg->roll;
  pinch_ref_ = reference_msg->pinch;
}

void GripperControllerNode::state_callback(
  const vortex_msgs::msg::GripperState::SharedPtr state_msg) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  roll_measured_ = state_msg->roll;
  pinch_measured_ = state_msg->pinch;
}

// ---------------------------------------------------------------------------
// CONTROL LOOP
// ---------------------------------------------------------------------------

void GripperControllerNode::publish_control() {
  types::GripperState measured_state;
  types::GripperState reference_state;

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    measured_state.roll = roll_measured_;
    measured_state.pinch = pinch_measured_;
    reference_state.roll = roll_ref_;
    reference_state.pinch = pinch_ref_;
  }

  const types::Vector2d velocity_command =
    controller_.calculate_velocity(measured_state, reference_state);

  vortex_msgs::msg::GripperStateVelocityCommand velocity_command_msg =
    gripper_controller::translator::velocity_command_to_gripper_velocity_command_msg(
      velocity_command);
  velocity_command_msg.header.stamp = this->now();

  control_pub_->publish(velocity_command_msg);
}

  RCLCPP_COMPONENTS_REGISTER_NODE(GripperControllerNode)

} // namespace vortex::controller
