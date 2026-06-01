#include "gripper_open_loop/gripper_open_loop_ros.hpp"
#include <spdlog/spdlog.h>
#include <rclcpp_components/register_node_macro.hpp>
#include <vortex/utils/ros/qos_profiles.hpp>

namespace vortex::open_loop {

GripperOpenLoopNode::GripperOpenLoopNode(const rclcpp::NodeOptions& options)
    : Node("gripper_open_loop_node", options) {
    duration_ = this->declare_parameter<double>("duration", 1.0);
    set_publisher_and_action_server();
    spdlog::info("GripperOpenLoop: ready  duration={:.3f}s", duration_);
}

void GripperOpenLoopNode::set_publisher_and_action_server() {
    const std::string velocity_topic =
        this->declare_parameter<std::string>("topics.velocity_command");
    const std::string action_name =
        this->declare_parameter<std::string>("action_servers.gripper_open_loop");

    auto qos = vortex::utils::qos_profiles::sensor_data_profile(1);
    velocity_pub_ =
        this->create_publisher<vortex_msgs::msg::GripperStateVelocityCommand>(
            velocity_topic, qos);

    cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    action_server_ = rclcpp_action::create_server<GripperOpenLoop>(
        this,
        action_name,
        std::bind(&GripperOpenLoopNode::handle_goal,     this,
                  std::placeholders::_1, std::placeholders::_2),
        std::bind(&GripperOpenLoopNode::handle_cancel,   this,
                  std::placeholders::_1),
        std::bind(&GripperOpenLoopNode::handle_accepted, this,
                  std::placeholders::_1),
        rcl_action_server_get_default_options(), cb_group_);
}

rclcpp_action::GoalResponse GripperOpenLoopNode::handle_goal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const GripperOpenLoop::Goal> goal) {
    (void)uuid;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (goal_handle_ && goal_handle_->is_active()) {
            spdlog::info("GripperOpenLoop: aborting current goal, accepting new one");
            preempted_goal_id_ = goal_handle_->get_goal_id();
        }
    }
    spdlog::info("GripperOpenLoop: goal accepted  roll_delta={:.4f}  pinch_delta={:.4f}",
                 goal->roll_delta, goal->pinch_delta);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GripperOpenLoopNode::handle_cancel(
    const std::shared_ptr<GoalHandle> goal_handle) {
    (void)goal_handle;
    spdlog::info("GripperOpenLoop: cancel requested");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void GripperOpenLoopNode::handle_accepted(
    const std::shared_ptr<GoalHandle> goal_handle) {
    std::thread{std::bind(&GripperOpenLoopNode::execute, this,
                std::placeholders::_1), goal_handle}.detach();
}

void GripperOpenLoopNode::execute(const std::shared_ptr<GoalHandle> goal_handle) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        goal_handle_ = goal_handle;
    }

    const double roll_delta  = goal_handle->get_goal()->roll_delta;
    const double pinch_delta = goal_handle->get_goal()->pinch_delta;

    // Velocity needed to cover the requested delta in exactly duration_ seconds
    const double roll_vel  = roll_delta  / duration_;
    const double pinch_vel = pinch_delta / duration_;

    spdlog::info("GripperOpenLoop: running  roll_vel={:.4f} rad/s  pinch_vel={:.4f} rad/s  for {:.3f}s",
                 roll_vel, pinch_vel, duration_);

    auto feedback = std::make_shared<GripperOpenLoop::Feedback>();
    auto result   = std::make_shared<GripperOpenLoop::Result>();

    const rclcpp::Time start_time = this->now();
    rclcpp::Rate loop_rate(100);  // 100 Hz

    while (rclcpp::ok()) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (goal_handle->get_goal_id() == preempted_goal_id_) {
                publish_velocity(0.0, 0.0);
                result->success = false;
                goal_handle->abort(result);
                spdlog::info("GripperOpenLoop: goal aborted (preempted)");
                return;
            }
            if (goal_handle->is_canceling()) {
                publish_velocity(0.0, 0.0);
                result->success = false;
                goal_handle->canceled(result);
                spdlog::info("GripperOpenLoop: goal canceled");
                return;
            }
        }

        const double elapsed = (this->now() - start_time).seconds();

        if (elapsed >= duration_) {
            publish_velocity(0.0, 0.0);
            result->success = true;
            goal_handle->succeed(result);
            spdlog::info("GripperOpenLoop: goal succeeded  elapsed={:.3f}s", elapsed);
            return;
        }

        publish_velocity(roll_vel, pinch_vel);

        feedback->elapsed_time = elapsed;
        goal_handle->publish_feedback(feedback);

        loop_rate.sleep();
    }

    // Node shutdown during execution
    if (goal_handle->is_active()) {
        publish_velocity(0.0, 0.0);
        result->success = false;
        try { goal_handle->abort(result); } catch (...) {}
    }
}

void GripperOpenLoopNode::publish_velocity(double roll_vel, double pinch_vel) {
    vortex_msgs::msg::GripperStateVelocityCommand msg;
    msg.header.stamp  = this->now();
    msg.roll_velocity  = roll_vel;
    msg.pinch_velocity = pinch_vel;
    velocity_pub_->publish(msg);
}

}  // namespace vortex::open_loop

RCLCPP_COMPONENTS_REGISTER_NODE(vortex::open_loop::GripperOpenLoopNode)
