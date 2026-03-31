// gripper_controller_translator.hpp
// Mirrors gripper_state_translator.hpp on the input side.
// Translates controller velocity commands back through gear ratio
// to a JointState velocity msg for the interface node.

#include "sensor_msgs/msg/joint_state.hpp"

// TODO: Must match the gear ratios in gripper_state_translator.hpp exactly.
// If those are updated, update these too — consider sharing via a common header.
constexpr double CONTROLLER_GEAR_RATIO_ROLL  = 1.0;  // TODO: measure
constexpr double CONTROLLER_GEAR_RATIO_PINCH = 1.0;  // TODO: measure

constexpr std::size_t CONTROLLER_MIN_VELOCITY_SIZE = 2;

inline sensor_msgs::msg::JointState translate_to_joint_velocity(
    double roll_vel, double pinch_vel)
{
    sensor_msgs::msg::JointState msg;
    msg.name     = {"roll", "grip"};
    msg.velocity = {roll_vel  / CONTROLLER_GEAR_RATIO_ROLL,
                    pinch_vel / CONTROLLER_GEAR_RATIO_PINCH};
    // position and effort left empty — velocity command only
    return msg;
}
