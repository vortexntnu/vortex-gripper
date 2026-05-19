#ifndef GRIPPER_REFERENCE_FILTER__GRIPPER_REFERENCE_FILTER_ROS_HPP_
#define GRIPPER_REFERENCE_FILTER__GRIPPER_REFERENCE_FILTER_ROS_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <vortex_msgs/action/gripper_reference_filter_waypoint.hpp>
#include <vortex_msgs/msg/gripper_reference_filter.hpp>
#include <vortex_msgs/msg/gripper_state.hpp>
#include <vortex_msgs/msg/gripper_waypoint.hpp>
#include "gripper_reference_filter/gripper_eigen_typedefs.hpp"
#include "gripper_reference_filter/gripper_reference_filter.hpp"

namespace vortex::guidance {

class GripperReferenceFilterNode : public rclcpp::Node {
   public:
    explicit GripperReferenceFilterNode(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    ~GripperReferenceFilterNode();

   private:
    // @brief Declare ROS parameters and create the subscription/publisher pair.
    void set_subscribers_and_publisher();

    // @brief Declare ROS parameters and create the action server.
    void set_action_server();

    // @brief Declare ROS parameters and construct the reference filter instance.
    void set_refererence_filter();

    // @brief Latch the measured gripper state for later use as the initial
    //        reference seed and for mode-masking the goal.
    void reference_callback(
        const vortex_msgs::msg::GripperState::SharedPtr state_msg);

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& /*uuid*/,
        std::shared_ptr<
            const vortex_msgs::action::GripperReferenceFilterWaypoint::Goal>
        /*goal*/);

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<
            vortex_msgs::action::GripperReferenceFilterWaypoint>>
        /*goal_handle*/);

    void handle_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<
            vortex_msgs::action::GripperReferenceFilterWaypoint>> goal_handle);

    void execute(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<
            vortex_msgs::action::GripperReferenceFilterWaypoint>> goal_handle);

    // @brief Latch the filter's current 2D output as the held reference and
    //        publish it once. Called whenever a goal ends so that the next
    //        republish_held_reference_tick has a non-default value to send.
    void latch_current_state_as_held_reference();

    // @brief Periodically republish the held reference when no goal is active.
    //        This keeps downstream consumers receiving the last committed
    //        reference rather than a default-zero message, which prevented
    //        the convergence check from spuriously succeeding on a new
    //        zero-valued goal.
    void republish_held_reference_tick();

    rclcpp_action::Server<
        vortex_msgs::action::GripperReferenceFilterWaypoint>::SharedPtr
        action_server_;

    std::unique_ptr<GripperReferenceFilter> gripper_reference_filter_{};

    rclcpp::Publisher<vortex_msgs::msg::GripperReferenceFilter>::SharedPtr
        reference_pub_;

    rclcpp::Subscription<vortex_msgs::msg::GripperState>::SharedPtr
        reference_sub_;

    std::chrono::milliseconds time_step_ms_{};

    Eigen::Vector2d measured_reference_{Eigen::Vector2d::Zero()};

    std::mutex mutex_;

    rclcpp::TimerBase::SharedPtr held_reference_republish_timer_;
    vortex_msgs::msg::GripperReferenceFilter last_published_reference_;
    bool holding_reference_{false};

    std::atomic<bool> preempted_{false};
    std::mutex execute_mutex_;
    std::thread execute_thread_;
};

}  // namespace vortex::guidance

#endif  // GRIPPER_REFERENCE_FILTER__GRIPPER_REFERENCE_FILTER_ROS_HPP_
