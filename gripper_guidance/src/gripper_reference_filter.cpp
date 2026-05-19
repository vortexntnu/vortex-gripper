#include "gripper_reference_filter/gripper_reference_filter.hpp"

namespace vortex::guidance {

GripperReferenceFilter::GripperReferenceFilter(const GripperReferenceFilterParams& params) {
    Ad_.setZero();
    Bd_.setZero();
    filter_state_.setZero();
    calculate_Ad(params.omega, params.zeta);
    calculate_Bd(params.omega);
}

void GripperReferenceFilter::reset(const Eigen::Vector2d& initial_reference) {
    filter_state_.setZero();
    filter_state_(0) = initial_reference(0);
    filter_state_(1) = initial_reference(1);
}

void GripperReferenceFilter::step(const Eigen::Vector2d& goal_reference,
                                  double time_step_seconds) {
    const Eigen::Vector6d state_derivative =
        calculate_state_derivative(filter_state_, goal_reference);
    filter_state_ += state_derivative * time_step_seconds;
}

void GripperReferenceFilter::snap_to(const Eigen::Vector2d& goal_reference) {
    filter_state_.head(2) = goal_reference;
}

Eigen::Vector2d GripperReferenceFilter::reference_output() const {
    return filter_state_.head(2);
}

Eigen::Vector6d GripperReferenceFilter::calculate_state_derivative(
    const Eigen::Vector6d& state, const Eigen::Vector2d& reference) const {
    const Eigen::Vector6d state_derivative = Ad_ * state + Bd_ * reference;
    return state_derivative;
}

void GripperReferenceFilter::calculate_Ad(const Eigen::Vector2d& omega,
                                          const Eigen::Vector2d& zeta) {
    const Eigen::Matrix2d omega_diag = omega.asDiagonal();
    const Eigen::Matrix2d zeta_diag = zeta.asDiagonal();
    const Eigen::Matrix2d omega_diag_squared = omega_diag * omega_diag;
    const Eigen::Matrix2d omega_diag_cubed = omega_diag_squared * omega_diag;

    Ad_.block<2,2>(0, 2) = Eigen::Matrix2d::Identity();

    Ad_.block<2,2>(2, 4) = Eigen::Matrix2d::Identity();

    Ad_.block<2,2>(4, 0) = -omega_diag_cubed;
    Ad_.block<2,2>(4, 2) = -2 * (2*zeta_diag + Eigen::Matrix2d::Identity()) * omega_diag_squared;
    Ad_.block<2,2>(4, 4) = -2 * (2*zeta_diag + Eigen::Matrix2d::Identity()) * omega_diag;
}

void GripperReferenceFilter::calculate_Bd(const Eigen::Vector2d& omega) {
    const Eigen::Matrix2d omega_diag = omega.asDiagonal();
    const Eigen::Matrix2d omega_diag_squared = omega_diag * omega_diag;
    const Eigen::Matrix2d omega_diag_cubed = omega_diag_squared * omega_diag;

    Bd_.block<2,2>(4, 0) = omega_diag_cubed;
}

}  // namespace vortex::guidance
