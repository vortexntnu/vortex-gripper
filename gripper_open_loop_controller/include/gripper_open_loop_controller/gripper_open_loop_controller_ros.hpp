#ifndef GRIPPER_OPEN_LOOP_CONTROLLER__GRIPPER_OPEN_LOOP_CONTROLLER_ROS_HPP_
#define GRIPPER_OPEN_LOOP_CONTROLLER__GRIPPER_OPEN_LOOP_CONTROLLER_ROS_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <vortex_msgs/action/gripper_open_loop_command.hpp>
#include <vortex_msgs/msg/gripper_state_velocity_command.hpp>

namespace vortex::controller {

// @brief Open-loop gripper controller. Owns an action server that accepts a
//        velocity command for a fixed duration and republishes that command
//        on a velocity topic at a fixed rate until the duration elapses.
//        Intended as a feedback-free fallback for cases where the gripper
//        state is not observable.
class GripperOpenLoopControllerNode : public rclcpp::Node {
   public:
    explicit GripperOpenLoopControllerNode(const rclcpp::NodeOptions& options);

    ~GripperOpenLoopControllerNode();

   private:
    void set_controller_params();

    void set_publisher();

    void set_action_server();

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& /*uuid*/,
        std::shared_ptr<const vortex_msgs::action::GripperOpenLoopCommand::Goal>
        /*goal*/);

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<
            vortex_msgs::action::GripperOpenLoopCommand>> /*goal_handle*/);

    void handle_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<
            vortex_msgs::action::GripperOpenLoopCommand>> goal_handle);

    void execute(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<
            vortex_msgs::action::GripperOpenLoopCommand>> goal_handle);

    // @brief Publish a single zero-velocity command. Called after the goal
    //        ends so that downstream consumers do not continue actuating.
    void publish_zero_velocity();

    rclcpp_action::Server<vortex_msgs::action::GripperOpenLoopCommand>::SharedPtr
        action_server_;

    rclcpp::Publisher<vortex_msgs::msg::GripperStateVelocityCommand>::SharedPtr
        control_pub_;

    std::chrono::milliseconds time_step_ms_{};

    std::atomic<bool> preempted_{false};
    std::mutex execute_mutex_;
    std::thread execute_thread_;
};

}  // namespace vortex::controller

#endif  // GRIPPER_OPEN_LOOP_CONTROLLER__GRIPPER_OPEN_LOOP_CONTROLLER_ROS_HPP_
