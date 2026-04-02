#include "gripper_interface/gripper_interface_ros.hpp"

#include <spdlog/spdlog.h>

const auto start_message = R"(

  ____       _                   ___       _             __
 / ___|_ __ (_)_ __  _ __   ___|_ _|_ __ | |_ ___ _ __ / _| __ _  ___ ___
| |  _| '__| | '_ \| '_ \ / _ \| || '_ \| __/ _ \ '__| |_ / _` |/ __/ _ \
| |_| | |  | | |_) | |_) |  __/| || | | | ||  __/ |  |  _| (_| | (_|  __/
 \____|_|  |_| .__/| .__/ \___|___|_| |_|\__\___|_|  |_|  \__,_|\___\___|
             |_|   |_|

)";

namespace vortex::interface {

// ---------------------------------------------------------------------------
// Construction / Destruction
// ---------------------------------------------------------------------------

GripperInterfaceNode::GripperInterfaceNode(const rclcpp::NodeOptions& options)
: Node("gripper_interface_node", options) {
    declare_and_load_parameters();
    init_can_driver();   // throws on CAN open failure — node will not start
    setup_pub_sub();
    spdlog::info(start_message);
}

GripperInterfaceNode::~GripperInterfaceNode() {
    // can_driver_ destructor sends STOP_GRIPPER and closes the socket.
    // Nothing extra needed here, but log it clearly.
    spdlog::info("GripperInterfaceNode: shutting down, sending STOP_GRIPPER");
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------

void GripperInterfaceNode::declare_and_load_parameters() {
    can_interface_ = this->declare_parameter<std::string>("can_interface", "can0");

    const int poll_ms = this->declare_parameter<int>("encoder_poll_ms", 20);
    poll_period_ = std::chrono::milliseconds(poll_ms);

    spdlog::info("GripperInterfaceNode: can_interface={} encoder_poll_ms={}",
                 can_interface_, poll_ms);
}

void GripperInterfaceNode::init_can_driver() {
    can_driver_ = std::make_unique<GripperCanDriver>(can_interface_);
    can_driver_->start_gripper();
}

void GripperInterfaceNode::setup_pub_sub() {
    const std::string state_topic =
        this->declare_parameter<std::string>("topics.gripper_state");
    const std::string cmd_topic =
        this->declare_parameter<std::string>("topics.velocity_command");

    auto qos = vortex::utils::qos_profiles::sensor_data_profile(1);

    state_pub_ = this->create_publisher<vortex_msgs::msg::GripperState>(
        state_topic, qos);

    velocity_cmd_sub_ =
        this->create_subscription<vortex_msgs::msg::GripperStateVelocityCommand>(
            cmd_topic, qos,
            std::bind(&GripperInterfaceNode::velocity_command_callback,
                      this, std::placeholders::_1));

    // Poll timer: runs at encoder_poll_ms to check the non-blocking CAN socket.
    // The MCU transmits encoder frames at its own TC0 rate; we just need to
    // drain them promptly. Any frame received triggers an immediate publish.
    encoder_poll_timer_ = this->create_wall_timer(
        poll_period_,
        std::bind(&GripperInterfaceNode::encoder_poll_callback, this));

    spdlog::info("GripperInterfaceNode: state_pub={} cmd_sub={}",
                 state_topic, cmd_topic);
}

// ---------------------------------------------------------------------------
// Timer: drain CAN socket, publish on each valid encoder frame
// ---------------------------------------------------------------------------

void GripperInterfaceNode::encoder_poll_callback() {
    // Drain all pending frames in one timer tick — the socket may have buffered
    // multiple frames if the poll period is longer than the MCU TX period.
    while (true) {
        auto maybe_frame = can_driver_->read_encoder_frame();
        if (!maybe_frame.has_value()) {
            break;
        }

        vortex_msgs::msg::GripperState state_msg =
            gripper_interface::translator::encoder_frame_to_gripper_state(
                maybe_frame.value());

        state_msg.header.stamp = this->now();
        state_pub_->publish(state_msg);
    }
}

// ---------------------------------------------------------------------------
// Subscription: velocity command → CAN PWM
// ---------------------------------------------------------------------------

void GripperInterfaceNode::velocity_command_callback(
    const vortex_msgs::msg::GripperStateVelocityCommand::SharedPtr msg) {

    const gripper_interface::types::RawPwmFrame pwm_frame =
        gripper_interface::translator::velocity_command_to_pwm_frame(*msg);

    can_driver_->send_pwm(pwm_frame);
}

RCLCPP_COMPONENTS_REGISTER_NODE(GripperInterfaceNode)

}  // namespace vortex::interface
