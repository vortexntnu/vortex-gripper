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
    init_can();
    setup_pub_sub();
    spdlog::info(start_message);
}

GripperInterfaceNode::~GripperInterfaceNode() {
    spdlog::info("GripperInterfaceNode: shutting down");
    can_.stop_async_receive();
    can_.send(0x45b, nullptr, 0);  // STOP_GRIPPER
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------

void GripperInterfaceNode::declare_and_load_parameters() {
    can_interface_name_ = this->declare_parameter<std::string>("can_interface", "can0");
    spdlog::info("GripperInterfaceNode: can_interface={}", can_interface_name_);
}

void GripperInterfaceNode::init_can() {
    const can_status status = can_.init(can_interface_name_);
    if (status != can_status::OK) {
        throw std::runtime_error(
            "GripperInterfaceNode: failed to initialise CAN on '" +
            can_interface_name_ + "' (status " +
            std::to_string(static_cast<int>(status)) + ")");
    }


    // Filter to SEND_ANGLES (0x46a) only — discard all other bus traffic.
    can_.set_filter(0x46a, CAN_SFF_MASK);

    // Start receive thread. on_can_frame is called for every frame that passes
    // the filter, plus ERR_RECEIVE on 1-second timeouts (handled gracefully).
    can_.start_async_receive(
        [this](const struct canfd_frame& frame, can_status s) {
            on_can_frame(frame, s);
        });

    can_.send(0x45a, nullptr, 0);  // START_GRIPPER
    spdlog::info("GripperInterfaceNode: CAN initialised on {}", can_interface_name_);
}

void GripperInterfaceNode::setup_pub_sub() {
    const std::string state_topic =
        this->declare_parameter<std::string>("topics.gripper_state", "/gripper/state");
    const std::string cmd_topic =
        this->declare_parameter<std::string>("topics.velocity_command", "/gripper/velocity_command");

    auto qos = vortex::utils::qos_profiles::sensor_data_profile(1);

    state_pub_ = this->create_publisher<vortex_msgs::msg::GripperState>(state_topic, qos);

    velocity_cmd_sub_ =
        this->create_subscription<vortex_msgs::msg::GripperStateVelocityCommand>(
            cmd_topic, qos,
            std::bind(&GripperInterfaceNode::velocity_command_callback,
                      this, std::placeholders::_1));

    spdlog::info("GripperInterfaceNode: publishing GripperState on '{}'", state_topic);
    spdlog::info("GripperInterfaceNode: subscribing velocity commands on '{}'", cmd_topic);
}

// ---------------------------------------------------------------------------
// CAN receive (called from can_interface's internal thread)
// ---------------------------------------------------------------------------

void GripperInterfaceNode::on_can_frame(const struct canfd_frame& frame,
                                        can_status status) {
    if (status == can_status::ERR_RECEIVE) {
        return;  // 1-second timeout, not an error
    }

    if (status != can_status::OK) {
        spdlog::warn("GripperInterfaceNode: CAN receive error (status {})",
                     static_cast<int>(status));
        return;
    }

    if ((frame.can_id & CAN_SFF_MASK) != 0x46a) {  // not SEND_ANGLES
        return;
    }

    if (frame.len < 4) {
        spdlog::warn("GripperInterfaceNode: SEND_ANGLES frame too short (len={})",
                     static_cast<int>(frame.len));
        return;
    }

    gripper_interface::types::RawEncoderFrame encoder_frame{};
    std::memcpy(encoder_frame.bytes.data(), frame.data,
                std::min<size_t>(frame.len, encoder_frame.bytes.size()));

    vortex_msgs::msg::GripperState state_msg =
        gripper_interface::translator::encoder_frame_to_gripper_state(encoder_frame);
    state_msg.header.stamp = this->now();

    {
        std::lock_guard<std::mutex> lock(pub_mutex_);
        state_pub_->publish(state_msg);
    }
}

// ---------------------------------------------------------------------------
// Velocity command → CAN (called from ROS executor thread)
// ---------------------------------------------------------------------------

void GripperInterfaceNode::velocity_command_callback(
    const vortex_msgs::msg::GripperStateVelocityCommand::SharedPtr msg) {

    const gripper_interface::types::RawPwmFrame pwm_frame =
        gripper_interface::translator::velocity_command_to_pwm_frame(*msg);

    can_.send(0x469,                                          // SET_PWM
              pwm_frame.bytes.data(),
              static_cast<uint8_t>(pwm_frame.bytes.size()),
              /*use_brs=*/false);
}

RCLCPP_COMPONENTS_REGISTER_NODE(GripperInterfaceNode)

}  // namespace vortex::interface
