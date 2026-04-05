#include "gripper_reference_filter/gripper_reference_filter.hpp"

namespace vortex::guidance {

GripperReferenceFilter::GripperReferenceFilter(const GripperReferenceFilterParams& params) {
   Ad_.setZero(); 
   Bd_.setZero();
   calculate_Ad(params.omega, params.zeta);
   calculate_Bd(params.omega);
}

Eigen::Vector6d GripperReferenceFilter::calculate_x_dot(const Eigen::Vector6d& x,
                                                  const Eigen::Vector2d& r) {
    Eigen::Vector6d x_dot = Ad_ * x + Bd_ * r;

    return x_dot;
}


void GripperReferenceFilter::calculate_Ad(const Eigen::Vector2d& omega,
                                   const Eigen::Vector2d& zeta) {
    Eigen::Matrix2d omega_diag = omega.asDiagonal();
    Eigen::Matrix2d zeta_diag = zeta.asDiagonal(); 
    Eigen::Matrix2d omega_diag_squared = omega_diag * omega_diag;
    Eigen::Matrix2d omega_diag_cubed = omega_diag_squared * omega_diag;
    
    Ad_.block<2,2>(0, 2) = Eigen::Matrix2d::Identity();

    Ad_.block<2,2>(2, 4) = Eigen::Matrix2d::Identity();

    Ad_.block<2,2>(4, 0) = -omega_diag_cubed;
    Ad_.block<2,2>(4, 2) = -2 * (2*zeta_diag + Eigen::Matrix2d::Identity()) * omega_diag_squared;
    Ad_.block<2,2>(4, 4) = -2 * (2*zeta_diag + Eigen::Matrix2d::Identity()) * omega_diag;
}

void GripperReferenceFilter::calculate_Bd(const Eigen::Vector2d& omega) {
    Eigen::Matrix2d omega_diag = omega.asDiagonal();
    Eigen::Matrix2d omega_diag_squared = omega_diag * omega_diag;
    Eigen::Matrix2d omega_diag_cubed = omega_diag_squared * omega_diag;
    
    Bd_.block<2,2>(4, 0) = omega_diag_cubed;
}

}  // namespace vortex::guidance
