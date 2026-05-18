#ifndef GRIPPER_REFERENCE_FILTER__GRIPPER_REFERENCE_FILTER_ROS_UTILS_HPP_
#define GRIPPER_REFERENCE_FILTER__GRIPPER_REFERENCE_FILTER_ROS_UTILS_HPP_

#include <vortex_msgs/msg/gripper_reference_filter.hpp>
#include <vortex_msgs/msg/gripper_waypoint.hpp>
#include "gripper_reference_filter/gripper_eigen_typedefs.hpp"

namespace vortex::guidance {

inline Eigen::Vector6d fill_reference_state(
    const Eigen::Vector2d& current_reference) {
    Eigen::Vector6d state_vector = Eigen::Vector6d::Zero();
    state_vector(0) = current_reference(0);
    state_vector(1) = current_reference(1);
    return state_vector;
}

inline Eigen::Vector2d fill_reference_goal(
    const vortex_msgs::msg::GripperWaypoint& waypoint_goal) {
    Eigen::Vector2d reference;
    reference << waypoint_goal.roll, waypoint_goal.pinch;
    return reference;
}

inline vortex_msgs::msg::GripperReferenceFilter fill_reference_msg(
    const Eigen::Vector6d& state_vector) {
    vortex_msgs::msg::GripperReferenceFilter reference_msg;
    reference_msg.roll = state_vector(0);
    reference_msg.pinch = state_vector(1);
    return reference_msg;
}

inline Eigen::Vector2d apply_mode_logic(
    const Eigen::Vector2d& requested_reference,
    const Eigen::Vector2d& current_reference,
    uint8_t mode) {
    Eigen::Vector2d resolved_reference = requested_reference;
    switch (mode) {
        case vortex_msgs::msg::GripperWaypoint::ROLL_AND_PINCH:
            break;
        case vortex_msgs::msg::GripperWaypoint::ONLY_ROLL:
            resolved_reference(1) = current_reference(1);
            break;
        case vortex_msgs::msg::GripperWaypoint::ONLY_PINCH:
            resolved_reference(0) = current_reference(0);
            break;
    }
    return resolved_reference;
}

}  // namespace vortex::guidance

#endif  // GRIPPER_REFERENCE_FILTER__GRIPPER_REFERENCE_FILTER_ROS_UTILS_HPP_
