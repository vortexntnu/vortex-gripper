#ifndef GRIPPER_REFERENCE_FILTER__GRIPPER_REFERENCE_FILTER_HPP_
#define GRIPPER_REFERENCE_FILTER__GRIPPER_REFERENCE_FILTER_HPP_

#include "gripper_reference_filter/gripper_eigen_typedefs.hpp"

namespace vortex::guidance {

struct GripperReferenceFilterParams {
    Eigen::Vector2d omega = Eigen::Vector2d::Zero();
    Eigen::Vector2d zeta = Eigen::Vector2d::Zero();
};

class GripperReferenceFilter {
   public:
    explicit GripperReferenceFilter(const ReferenceFilterParams& params);

    // @brief Calculate the state derivative
    // @param x The state vector 6x1
    // @param r The reference vector 2x1
    // @return The state derivative 6x1
    // REF: Handbook of Marine Craft Hydrodynamics and Motion Control, Fossen
    // 2021 p. 336 eq: 12.5
    Eigen::Vector2d calculate_x_dot(const Eigen::Vector6d& x,
                                     const Eigen::Vector2d& r);

    // @brief Calculate the state transition matrix
    // REF: Handbook of Marine Craft Hydrodynamics and Motion Control, Fossen
    // 2021 p. 336 eq: 12.6
    void calculate_Ad(const Eigen::Vector2d& omega,
                      const Eigen::Vector2d& zeta);

    // @brief Calculate the input matrix
    // REF: Handbook of Marine Craft Hydrodynamics and Motion Control, Fossen
    // 2021 p. 336 eq: 12.6 
    void calculate_Bd(const Eigen::Vector2d& omega);

   private:
    Eigen::Matrix6d Ad_;
    Eigen::Matrix6x2d Bd_;
};

}  // namespace vortex::guidance

#endif  // GRIPPER_REFERENCE_FILTER__GRIPPER_REFERENCE_FILTER_HPP_
