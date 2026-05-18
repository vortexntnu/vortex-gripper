#include "gripper_controller/gripper_controller.hpp"

#include <algorithm>

GripperController::GripperController()
    : Kp_(types::Matrix2d::Identity()), timestep_(0.01) {}

types::Vector2d GripperController::calculate_velocity(
    const types::GripperState& measured_state,
    const types::GripperState& reference_state) {

    const types::Vector2d position_error = [&] {
        types::Vector2d error;
        error(0) = reference_state.roll  - measured_state.roll;
        error(1) = reference_state.pinch - measured_state.pinch;
        return error;
    }();

    types::Vector2d velocity_command = Kp_ * position_error;

    velocity_command(0) = std::clamp(velocity_command(0),
                                     -types::MAX_ROLL_VEL,
                                      types::MAX_ROLL_VEL);
    velocity_command(1) = std::clamp(velocity_command(1),
                                     -types::MAX_PINCH_VEL,
                                      types::MAX_PINCH_VEL);

    return velocity_command;
}

void GripperController::set_kp(const types::Matrix2d& proportional_gain_matrix) {
    Kp_ = proportional_gain_matrix;
}

void GripperController::set_time_step(double timestep) {
    timestep_ = timestep;
}
