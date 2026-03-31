#ifndef GRIPPER_REFERENCE_FILTER__GRIPPER_REFERENCE_FILTER_ROS_HPP_
#define GRIPPER_REFERENCE_FILTER__GRIPPER_REFERENCE_FILTER_ROS_HPP_

#include <memory>
#include <mutex>
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

   private:
    // @brief Set the subscribers and publishers
    void set_subscribers_and_publisher();

    // @brief Set the action server
    void set_action_server();

    // @brief Initializes the reference filter with ROS parameters
    void set_refererence_filter();

    // @brief Callback for incoming gripper state messages
    // @param msg The gripper state message
    void reference_callback(
        const vortex_msgs::msg::GripperState::SharedPtr msg);

    // @brief Handle the goal request
    // @param uuid The goal UUID
    // @param goal The goal message
    // @return The goal response
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<
            const vortex_msgs::action::GripperReferenceFilterWaypoint::Goal> goal);

    // @brief Handle the cancel request
    // @param goal_handle The goal handle
    // @return The cancel response
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<
            vortex_msgs::action::GripperReferenceFilterWaypoint>> goal_handle);

    // @brief Handle the accepted request
    // @param goal_handle The goal handle
    void handle_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<
            vortex_msgs::action::GripperReferenceFilterWaypoint>> goal_handle);

    // @brief Execute the goal
    // @param goal_handle The goal handle
    void execute(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<
            vortex_msgs::action::GripperReferenceFilterWaypoint>> goal_handle);

    // @brief Fill the initial state vector from current gripper state
    // @return 6D state vector [roll, pinch, roll_dot, pinch_dot, roll_dotdot, pinch_dotdot]
    Eigen::Vector6d fill_reference_state();

    // @brief Fill the reference goal vector from a GripperWaypoint
    // @param goal The gripper waypoint message
    // @return 2D reference vector [roll, pinch]
    Eigen::Vector2d fill_reference_goal(
        const vortex_msgs::msg::GripperWaypoint& goal);

    // @brief Apply mode logic to the reference vector
    // @param reference_in The input reference vector
    // @param mode The mode (ROLL_AND_PINCH, ONLY_ROLL, ONLY_PINCH)
    // @return The modified reference vector
    Eigen::Vector2d apply_mode_logic(
        const Eigen::Vector2d& reference_in, uint8_t mode);

    // @brief Publish a hold reference message at the current gripper state
    void publish_hold_reference();

    // @brief Fill the reference filter output message from the current state
    // @return The gripper reference filter message
    vortex_msgs::msg::GripperReferenceFilter fill_reference_msg();

    // Action server
    rclcpp_action::Server<
        vortex_msgs::action::GripperReferenceFilterWaypoint>::SharedPtr
        action_server_;

    // Reference filter instance
    std::unique_ptr<GripperReferenceFilter> gripper_reference_filter_{};

    // Publisher: smoothed reference to controller
    rclcpp::Publisher<vortex_msgs::msg::GripperReferenceFilter>::SharedPtr
        reference_pub_;

    // Subscriber: raw gripper state from hardware
    rclcpp::Subscription<vortex_msgs::msg::GripperState>::SharedPtr
        reference_sub_;

    std::chrono::milliseconds time_step_{};

    // x_ is [roll, pinch, roll_dot, pinch_dot, roll_dotdot, pinch_dotdot]
    // 6D state vector (2 DOF x position/velocity/acceleration)
    Eigen::Vector6d x_;

    // reference_ holds the live gripper state [roll, pinch] from the subscriber
    Eigen::Vector2d reference_;

    std::mutex mutex_;

    rclcpp_action::GoalUUID preempted_goal_id_;

    std::shared_ptr<rclcpp_action::ServerGoalHandle<
        vortex_msgs::action::GripperReferenceFilterWaypoint>>
        goal_handle_;

    rclcpp::CallbackGroup::SharedPtr cb_group_;
};

}  // namespace vortex::guidance

#endif  // GRIPPER_REFERENCE_FILTER__GRIPPER_REFERENCE_FILTER_ROS_HPP_
