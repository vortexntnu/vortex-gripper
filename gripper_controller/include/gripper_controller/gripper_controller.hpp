#ifndef GRIPPER_CONTROLLER__GRIPPER_CONTROLLER_HPP_
#define GRIPPER_CONTROLLER__GRIPPER_CONTROLLER_HPP_

#include "gripper_controller/gripper_controller_typedefs.hpp"


class GripperController {
   public:
    GripperController();

    // @brief Calculate velocity command from position error
    // @param state:     struct containing measured gripper state [roll, pinch]
    // @param reference: struct containing desired gripper state [roll, pinch]
    // @return 2D vector containing velocity commands [roll_vel, pinch_vel]
    types::Vector2d calculate_velocity(const types::GripperState& state,
                                       const types::GripperState& reference);

    // @brief Set the proportional gain matrix
    // @param Kp: 2x2 matrix containing the proportional gain matrix
    void set_kp(const types::Matrix2d& Kp);

    // @brief Set the time step
    // @param dt: time step in seconds
    void set_time_step(double timestep);

   private:
    types::Matrix2d Kp_;
    double timestep_;
};


#endif  // GRIPPER_CONTROLLER__GRIPPER_CONTROLLER_HPP_
