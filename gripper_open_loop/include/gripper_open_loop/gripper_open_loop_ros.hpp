#ifndef GRIPPER_OPEN_LOOP__GRIPPER_OPEN_LOOP_ROS_HPP_
#define GRIPPER_OPEN_LOOP__GRIPPER_OPEN_LOOP_ROS_HPP_

#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <vortex_msgs/action/gripper_open_loop.hpp>
#include <vortex_msgs/msg/gripper_state_velocity_command.hpp>
#include <vortex/utils/ros/qos_profiles.hpp>

namespace vortex::open_loop {

class GripperOpenLoopNode : public rclcpp::Node {
public:
    explicit GripperOpenLoopNode(const rclcpp::NodeOptions& options);

private:
    using GripperOpenLoop = vortex_msgs::action::GripperOpenLoop;
    using GoalHandle = rclcpp_action::ServerGoalHandle<GripperOpenLoop>;

    void set_publisher_and_action_server();

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const GripperOpenLoop::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandle> goal_handle);

    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);

    void execute(const std::shared_ptr<GoalHandle> goal_handle);

    void publish_velocity(double roll_vel, double pinch_vel);

    rclcpp::Publisher<vortex_msgs::msg::GripperStateVelocityCommand>::SharedPtr
        velocity_pub_;

    rclcpp_action::Server<GripperOpenLoop>::SharedPtr action_server_;
    rclcpp::CallbackGroup::SharedPtr cb_group_;

    std::mutex mutex_;
    std::shared_ptr<GoalHandle> goal_handle_;
    rclcpp_action::GoalUUID preempted_goal_id_;

    // Duration to run the velocity command (tunable param, seconds)
    double duration_;
};

}  // namespace vortex::open_loop

#endif  // GRIPPER_OPEN_LOOP__GRIPPER_OPEN_LOOP_ROS_HPP_
