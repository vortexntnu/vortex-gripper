#ifndef GRIPPER_CONTROLLER__GRIPPER_CONTROLLER_HPP_
#define GRIPPER_CONTROLLER__GRIPPER_CONTROLLER_HPP_

#include "gripper_controller/gripper_controller_typedefs.hpp"


class GripperController {
   public:
    GripperController();

    // @brief Calculate velocity command from position error
    // @param measured_state:  struct containing measured gripper state [roll, pinch]
    // @param reference_state: struct containing desired gripper state [roll, pinch]
    // @return 2D vector containing velocity commands [roll_vel, pinch_vel]
    types::Vector2d calculate_velocity(const types::GripperState& measured_state,
                                       const types::GripperState& reference_state);

    // @brief Set the proportional gain matrix
    // @param proportional_gain_matrix: 2x2 matrix containing the proportional gain matrix
    void set_kp(const types::Matrix2d& proportional_gain_matrix);

    // @brief Set the controller loop period in milliseconds
    // @param time_step_ms: time step in milliseconds
    void set_time_step_ms(double time_step_ms);

   private:
    types::Matrix2d Kp_;
    double time_step_ms_;
};


#endif  // GRIPPER_CONTROLLER__GRIPPER_CONTROLLER_HPP_
