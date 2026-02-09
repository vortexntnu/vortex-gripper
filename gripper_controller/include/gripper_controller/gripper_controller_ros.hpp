#ifndef GRIPPER_CONTROLLER_DP__GRIPPER_CONTROLLER_ROS_HPP_
#define GRIPPER_CONTROLLER_DP__GRIPPER_CONTROLLER_ROS_HPP_

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>


class GripperControllerNode()  : public rclcpp::Node {
  public: 
    GripperControllerNode(); 
    explicit  GripperControllerNode(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
        );
  private:
    // @brief Set the subscribers and publishers
    void set_subscribers_and_publisher();
    
    //TODO: Define the callback for the GripperReference node after it's been made
    //
    //
    //
    //
    //
    //

    //TODO: You won't be needing this one (PoseStamped) nephew, consult Cyprian if remove

    // @brief Callback for the reference topic
    // @param msg The reference message
    void reference_callback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    //TODO: Nor will you probably need this (PoseWithCovStamped) one too, consult Cyprian if remove

    // @brief Callback for the pose topic 
    // @pram msg The pose message 
    void pose_callback(
        const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

    
    rclcpp::Publisher<vortex_msgs::msg::ReferenceFilter>::SharedPtr
        reference_pub_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
        reference_sub_;

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr
        pose_sub_;

};

#endif // GRIPPER_CONTROLLER_DP__GRIPPER_CONTROLLER_ROS_HPP_
