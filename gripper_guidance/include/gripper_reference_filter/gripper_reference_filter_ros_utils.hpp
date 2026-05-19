#ifndef GRIPPER_REFERENCE_FILTER__GRIPPER_REFERENCE_FILTER_ROS_UTILS_HPP_
#define GRIPPER_REFERENCE_FILTER__GRIPPER_REFERENCE_FILTER_ROS_UTILS_HPP_

#include <vortex_msgs/msg/gripper_reference_filter.hpp>
#include <vortex_msgs/msg/gripper_waypoint.hpp>
#include "gripper_reference_filter/gripper_eigen_typedefs.hpp"

namespace vortex::guidance {

// @brief Extract the [roll, pinch] reference from a GripperWaypoint.
// @param waypoint_goal The action goal waypoint.
// @return 2D vector [roll, pinch].
inline Eigen::Vector2d fill_reference_goal(
    const vortex_msgs::msg::GripperWaypoint& waypoint_goal) {
    Eigen::Vector2d reference;
    reference << waypoint_goal.roll, waypoint_goal.pinch;
    return reference;
}

// @brief Build the wire-format GripperReferenceFilter message from a 2D
//        reference output. Only the position components (roll, pinch) are
//        published; higher-order filter state is intentionally not exposed.
// @param reference_output 2D vector [roll, pinch].
// @return Message ready for publication.
inline vortex_msgs::msg::GripperReferenceFilter fill_reference_msg(
    const Eigen::Vector2d& reference_output) {
    vortex_msgs::msg::GripperReferenceFilter reference_msg;
    reference_msg.roll = reference_output(0);
    reference_msg.pinch = reference_output(1);
    return reference_msg;
}

// @brief Apply mode logic to a requested reference. Axes excluded by the mode
//        are pinned to the corresponding component of current_reference so
//        that the unaffected axis does not drift.
// @param requested_reference  2D vector [roll, pinch] from the action goal.
// @param current_reference    2D vector [roll, pinch] currently being held.
// @param mode                 GripperWaypoint mode constant.
// @return The resolved reference after applying the mode mask.
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

// @brief Compute the convergence error between two references, with the axis
//        excluded by the mode masked to zero so it does not contribute to the
//        norm-based convergence check.
// @param current_reference  2D vector [roll, pinch] currently produced by the filter.
// @param goal_reference     2D vector [roll, pinch] requested by the goal.
// @param mode               GripperWaypoint mode constant.
// @return 2D error vector with masked axes set to zero.
inline Eigen::Vector2d compute_convergence_error(
    const Eigen::Vector2d& current_reference,
    const Eigen::Vector2d& goal_reference,
    uint8_t mode) {
    Eigen::Vector2d convergence_error = current_reference - goal_reference;
    switch (mode) {
        case vortex_msgs::msg::GripperWaypoint::ONLY_ROLL:
            convergence_error(1) = 0.0;
            break;
        case vortex_msgs::msg::GripperWaypoint::ONLY_PINCH:
            convergence_error(0) = 0.0;
            break;
        case vortex_msgs::msg::GripperWaypoint::ROLL_AND_PINCH:
        default:
            break;
    }
    return convergence_error;
}

}  // namespace vortex::guidance

#endif  // GRIPPER_REFERENCE_FILTER__GRIPPER_REFERENCE_FILTER_ROS_UTILS_HPP_
