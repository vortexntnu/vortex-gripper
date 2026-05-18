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
    void set_subscribers_and_publisher();

    void set_action_server();

    void set_refererence_filter();

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

    void publish_hold_reference();

    void publish_hold_timer();

    rclcpp_action::Server<
        vortex_msgs::action::GripperReferenceFilterWaypoint>::SharedPtr
        action_server_;

    std::unique_ptr<GripperReferenceFilter> gripper_reference_filter_{};

    rclcpp::Publisher<vortex_msgs::msg::GripperReferenceFilter>::SharedPtr
        reference_pub_;

    rclcpp::Subscription<vortex_msgs::msg::GripperState>::SharedPtr
        reference_sub_;

    std::chrono::milliseconds time_step_{};

    Eigen::Vector6d filter_state_;

    Eigen::Vector2d reference_;

    std::mutex mutex_;

    rclcpp::TimerBase::SharedPtr hold_timer_;
    vortex_msgs::msg::GripperReferenceFilter hold_reference_msg_;
    bool hold_active_{false};

    std::atomic<bool> preempted_{false};
    std::mutex execute_mutex_;
    std::thread execute_thread_;
};

}  // namespace vortex::guidance

#endif  // GRIPPER_REFERENCE_FILTER__GRIPPER_REFERENCE_FILTER_ROS_HPP_
