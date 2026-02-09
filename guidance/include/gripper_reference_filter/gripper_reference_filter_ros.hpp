#ifndef GRIPPER_REFERENCE_FILTER__GRIPPER_REFERENCE_FILTER_ROS_HPP_
#define GRIPPER_REFERENCE_FILTER__GRIPPER_REFERENCE_FILTER_ROS_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <vortex_msgs/action/reference_filter_waypoint.hpp> //TODO: Need to change this to gripper_reference_filter_waypoint when made in vortex_msgs 
#include <vortex_msgs/msg/reference_filter.hpp>
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

    // @brief Initializes the reference filter with ROS parameters.
    void set_refererence_filter();


    // @brief Handle the goal request
    // @param uuid The goal UUID
    // @param goal The goal message
    // @return The goal response
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<
            const vortex_msgs::action::ReferenceFilterWaypoint::Goal> goal); // TODO: make a GripperReferenceFilterWaypoint in vortex_msgs 
                    

    // @brief Handle the cancel request
    // @param goal_handle The goal handle
    // @return The cancel response
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<
            vortex_msgs::action::ReferenceFilterWaypoint>> goal_handle);

    // @brief Handle the accepted request
    // @param goal_handle The goal handle
    void handle_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<
            vortex_msgs::action::ReferenceFilterWaypoint>> goal_handle);

    // @brief Execute the goal
    // @param goal_handle The goal handle
    void execute(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<
            vortex_msgs::action::ReferenceFilterWaypoint>> goal_handle);

    Eigen::Vector6d fill_reference_state();

    Eigen::Vector2d fill_reference_goal(const geometry_msgs::msg::Pose& goal);

    Eigen::Vector2d apply_mode_logic(const Eigen::Vector2d& r_in, uint8_t mode);

    void publish_hold_reference();

    vortex_msgs::msg::ReferenceFilter fill_reference_msg();

    rclcpp_action::Server<
        vortex_msgs::action::ReferenceFilterWaypoint>::SharedPtr action_server_;

    std::unique_ptr<ReferenceFilter> reference_filter_{};

    rclcpp::Publisher<vortex_msgs::msg::ReferenceFilter>::SharedPtr
        reference_pub_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr  //TODO: PoseStamped should become GripperState topic eventually
        reference_sub_;



    rclcpp::TimerBase::SharedPtr reference_pub_timer_;

    std::chrono::milliseconds time_step_{};



    // x is [nu, nu_dot] (ref. page 336 in Fossen, 2021
    // nu is 2 degree of freedom (roll of the gripper AND pinch)
    Eigen::Vector6d x_;

    // The reference signal vector with 2 degrees of freedom [nu]
    Eigen::Vector2d r_;

    std::mutex mutex_;

    rclcpp_action::GoalUUID preempted_goal_id_;

    std::shared_ptr<rclcpp_action::ServerGoalHandle<
        vortex_msgs::action::ReferenceFilterWaypoint>>   //TODO: this needs to be changed to GripperReferenceFilterWaypoint
        goal_handle_;

    rclcpp::CallbackGroup::SharedPtr cb_group_;
};

}  // namespace vortex::guidance

#endif  // GRIPPER_REFERENCE_FILTER__GRIPPER_REFERENCE_FILTER_ROS_HPP_
