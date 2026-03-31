#include <vector>
#include <stdexcept>
#include "sensor_msgs/msg/joint_state.hpp"
#include "vortex_msgs/msg/gripper_state.hpp"

namespace vortex::guidance {

// TODO: Calibrate gear ratios experimentally before deployment.
// Roll:  suspect direct drive but may have a worm gear reduction.
// Pinch: leadscrew/rack — gear_ratio_pinch converts encoder radians
//        to jaw displacement (metres or radians depending on convention).
constexpr double GEAR_RATIO_ROLL  = 1.0;  // TODO: measure — direct drive assumed for now
constexpr double GEAR_RATIO_PINCH = 1.0;  // TODO: measure — leadscrew pitch unknown

constexpr std::size_t ENCODER_INDEX_ROLL  = 0;  // wrist → roll
constexpr std::size_t ENCODER_INDEX_PINCH = 1;  // grip  → pinch
constexpr std::size_t ENCODER_MIN_SIZE    = 2;

inline vortex_msgs::msg::GripperState translate_to_gripper_state(
    const sensor_msgs::msg::JointState::SharedPtr& msg)
  {
    if (msg->position.size() < ENCODER_MIN_SIZE) {
        throw std::runtime_error(
            "JointState has fewer than 2 positions — expected [roll, grip]");
    }

    vortex_msgs::msg::GripperState state;
    state.roll  = msg->position[ENCODER_INDEX_ROLL]  * GEAR_RATIO_ROLL;
    state.pinch = msg->position[ENCODER_INDEX_PINCH] * GEAR_RATIO_PINCH;
    return state;
  }
}
