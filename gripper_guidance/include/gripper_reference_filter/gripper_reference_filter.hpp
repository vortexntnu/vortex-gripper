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
    explicit GripperReferenceFilter(const GripperReferenceFilterParams& params);

    // @brief Reset the internal 6D filter state. Position components are seeded
    //        from the supplied reference; velocity and acceleration components
    //        are zeroed.
    // @param initial_reference 2D vector [roll, pinch]
    void reset(const Eigen::Vector2d& initial_reference);

    // @brief Integrate the filter one time step toward the goal reference.
    // @param goal_reference     2D vector [roll, pinch]
    // @param time_step_seconds  integration step in seconds
    void step(const Eigen::Vector2d& goal_reference, double time_step_seconds);

    // @brief Snap the position components of the filter state to the goal
    //        reference. Used at convergence so that the published output
    //        exactly matches the requested goal.
    // @param goal_reference 2D vector [roll, pinch]
    void snap_to(const Eigen::Vector2d& goal_reference);

    // @brief Return the position output of the filter (roll, pinch). The
    //        higher-order state components are intentionally not exposed on
    //        the wire.
    // @return 2D vector [roll, pinch]
    Eigen::Vector2d reference_output() const;

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
    // @brief Calculate the state derivative
    // @param state The state vector 6x1
    // @param reference The reference vector 2x1
    // @return The state derivative 6x1
    // REF: Handbook of Marine Craft Hydrodynamics and Motion Control, Fossen
    // 2021 p. 336 eq: 12.5
    Eigen::Vector6d calculate_state_derivative(
        const Eigen::Vector6d& state,
        const Eigen::Vector2d& reference) const;

    Eigen::Matrix6d Ad_;
    Eigen::Matrix6x2d Bd_;
    Eigen::Vector6d filter_state_;
};

}  // namespace vortex::guidance

#endif  // GRIPPER_REFERENCE_FILTER__GRIPPER_REFERENCE_FILTER_HPP_
