#ifndef GRIPPER_CONTROLLER__GRIPPER_CONTROLLER_TYPEDEFS_HPP_
#define GRIPPER_CONTROLLER__GRIPPER_CONTROLLER_TYPEDEFS_HPP_

#include <eigen3/Eigen/Dense>

namespace types {

using Vector2d = Eigen::Vector2d;
using Matrix2d = Eigen::Matrix2d;

struct GripperState {
    double roll  = 0.0;
    double pinch = 0.0;
};

// Velocity saturation limits — tune after gear ratio calibration
constexpr double MAX_ROLL_VEL  = 1.0;  // TODO: tune
constexpr double MAX_PINCH_VEL = 1.0;  // TODO: tune

}  // namespace types

#endif  // GRIPPER_CONTROLLER__GRIPPER_CONTROLLER_TYPEDEFS_HPP_
