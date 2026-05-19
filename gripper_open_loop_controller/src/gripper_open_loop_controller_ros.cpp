#include "gripper_open_loop_controller/gripper_open_loop_controller_ros.hpp"

#include <chrono>
#include <memory>
#include <spdlog/spdlog.h>
#include <rclcpp_components/register_node_macro.hpp>
#include <vortex/utils/ros/qos_profiles.hpp>

const auto start_message = R"(

 ____       _                          ___                   _                          ____            _             _ _
/ ___|_ __ (_)_ __ _ __   ___ _ __    / _ \ _ __   ___ _ __ | |    ___   ___  _ __     / ___|___  _ __ | |_ _ __ ___ | | | ___ _ __
| |  _| '_ \| | '_ \| '_ \ / _ \ '__|| | | | '_ \ / _ \ '_ \| |   / _ \ / _ \| '_ \   | |   / _ \| '_ \| __| '__/ _ \| | |/ _ \ '__|
| |_| | |_) | | |_) | |_) |  __/ |   | |_| | |_) |  __/ | | | |__| (_) | (_) | |_) |  | |__| (_) | | | | |_| | | (_) | | |  __/ |
 \____|  .__/|_| .__/| .__/ \___|_|    \___/| .__/ \___|_| |_|_____\___/ \___/| .__/   \____\___/|_| |_|\__|_|  \___/|_|_|\___|_|
        |_|    |_|   |_|                    |_|                               |_|

)";

namespace vortex::controller {

GripperOpenLoopControllerNode::GripperOpenLoopControllerNode(
    const rclcpp::NodeOptions& options)
: Node("gripper_open_loop_controller_node", options) {
    set_controller_params();

    set_publisher();

    set_action_server();

    spdlog::info(start_message);
}

GripperOpenLoopControllerNode::~GripperOpenLoopControllerNode() {
    preempted_ = true;
    if (execute_thread_.joinable()) {
        execute_thread_.join();
    }
}

void GripperOpenLoopControllerNode::set_controller_params() {
    const int time_step_ms_param =
        this->declare_parameter<int>("time_step_ms", 10);
    time_step_ms_ = std::chrono::milliseconds(time_step_ms_param);

    spdlog::info("GripperOpenLoopController: dt={}ms", time_step_ms_param);
}

void GripperOpenLoopControllerNode::set_publisher() {
    const std::string control_topic =
        this->declare_parameter<std::string>("topics.control");

    const auto qos_sensor_data =
        vortex::utils::qos_profiles::sensor_data_profile(1);

    control_pub_ =
        this->create_publisher<vortex_msgs::msg::GripperStateVelocityCommand>(
            control_topic, qos_sensor_data);
}

void GripperOpenLoopControllerNode::set_action_server() {
    const std::string action_server_name =
        this->declare_parameter<std::string>(
            "action_servers.gripper_open_loop_controller");

    action_server_ = rclcpp_action::create_server<
        vortex_msgs::action::GripperOpenLoopCommand>(
        this, action_server_name,
        [this](const auto& uuid, auto goal) {
            return handle_goal(uuid, std::move(goal));
        },
        [this](auto goal_handle) { return handle_cancel(goal_handle); },
        [this](auto goal_handle) { handle_accepted(goal_handle); });
}

rclcpp_action::GoalResponse GripperOpenLoopControllerNode::handle_goal(
    const rclcpp_action::GoalUUID& /*uuid*/,
    std::shared_ptr<const vortex_msgs::action::GripperOpenLoopCommand::Goal>
    /*goal*/) {
    spdlog::info("Accepted open-loop goal request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GripperOpenLoopControllerNode::handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<
        vortex_msgs::action::GripperOpenLoopCommand>> /*goal_handle*/) {
    spdlog::info("Received request to cancel open-loop goal");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void GripperOpenLoopControllerNode::handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<
        vortex_msgs::action::GripperOpenLoopCommand>> goal_handle) {
    std::lock_guard<std::mutex> lock(execute_mutex_);
    preempted_ = true;
    if (execute_thread_.joinable()) {
        execute_thread_.join();
    }
    preempted_ = false;

    execute_thread_ =
        std::thread([this, goal_handle]() { execute(goal_handle); });
}

void GripperOpenLoopControllerNode::publish_zero_velocity() {
    if (!control_pub_) {
        return;
    }
    auto stop_message =
        std::make_unique<vortex_msgs::msg::GripperStateVelocityCommand>();
    stop_message->header.stamp = this->now();
    stop_message->roll_velocity = 0.0;
    stop_message->pinch_velocity = 0.0;
    control_pub_->publish(std::move(stop_message));
}

void GripperOpenLoopControllerNode::execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<
        vortex_msgs::action::GripperOpenLoopCommand>> goal_handle) {
    const auto goal = goal_handle->get_goal();

    double requested_duration_seconds = goal->duration_seconds;
    if (requested_duration_seconds <= 0.0) {
        spdlog::warn(
            "GripperOpenLoopController: Invalid duration_seconds received "
            "(<= 0). Aborting goal.");
        auto failure_result =
            std::make_shared<vortex_msgs::action::GripperOpenLoopCommand::Result>();
        failure_result->success = false;
        goal_handle->abort(failure_result);
        return;
    }

    using GoalT = vortex_msgs::action::GripperOpenLoopCommand::Goal;
    const double requested_roll_velocity =
        (goal->mode == GoalT::ONLY_PINCH) ? 0.0 : goal->roll_velocity;
    const double requested_pinch_velocity =
        (goal->mode == GoalT::ONLY_ROLL) ? 0.0 : goal->pinch_velocity;

    spdlog::info(
        "GripperOpenLoopController: roll_vel={:.3f} pinch_vel={:.3f} "
        "duration={:.3f}s mode={}",
        requested_roll_velocity, requested_pinch_velocity,
        requested_duration_seconds, goal->mode);

    auto feedback =
        std::make_shared<vortex_msgs::action::GripperOpenLoopCommand::Feedback>();
    auto result =
        std::make_shared<vortex_msgs::action::GripperOpenLoopCommand::Result>();

    rclcpp::Rate loop_rate(1000.0 / time_step_ms_.count());
    const auto start_time = this->now();

    while (rclcpp::ok()) {
        const double elapsed_seconds =
            (this->now() - start_time).seconds();

        if (preempted_.load()) {
            publish_zero_velocity();
            result->success = false;
            goal_handle->abort(result);
            spdlog::info("Open-loop goal preempted by newer goal");
            return;
        }

        if (goal_handle->is_canceling()) {
            publish_zero_velocity();
            result->success = false;
            goal_handle->canceled(result);
            spdlog::info("Open-loop goal canceled");
            return;
        }

        if (elapsed_seconds >= requested_duration_seconds) {
            publish_zero_velocity();
            result->success = true;
            goal_handle->succeed(result);
            spdlog::info("Open-loop goal duration reached");
            return;
        }

        auto velocity_command_msg =
            std::make_unique<vortex_msgs::msg::GripperStateVelocityCommand>();
        velocity_command_msg->header.stamp = this->now();
        velocity_command_msg->roll_velocity = requested_roll_velocity;
        velocity_command_msg->pinch_velocity = requested_pinch_velocity;
        control_pub_->publish(std::move(velocity_command_msg));

        feedback->elapsed_seconds = elapsed_seconds;
        goal_handle->publish_feedback(feedback);

        loop_rate.sleep();
    }

    if (!rclcpp::ok() && goal_handle->is_active()) {
        result->success = false;
        try {
            goal_handle->abort(result);
        } catch (...) {
            // Ignore exceptions during shutdown
        }
    }
}

}  // namespace vortex::controller

RCLCPP_COMPONENTS_REGISTER_NODE(vortex::controller::GripperOpenLoopControllerNode)
