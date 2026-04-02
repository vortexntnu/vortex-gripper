#ifndef GRIPPER_INTERFACE__GRIPPER_CAN_IDS_HPP_
#define GRIPPER_INTERFACE__GRIPPER_CAN_IDS_HPP_

#include <cstdint>

// ---------------------------------------------------------------------------
// CAN IDs — must match can_common.h on the MCU firmware side.
// Update these if the firmware IDs change.
// ---------------------------------------------------------------------------
namespace gripper_can_ids {

// Host → MCU
constexpr uint32_t SET_PWM        = 0x469;  // 4 bytes: uint16_t[2] duty_us [roll, pinch]
constexpr uint32_t START_GRIPPER  = 0x45a;  // 0 bytes: enable servos
constexpr uint32_t STOP_GRIPPER   = 0x45b;  // 0 bytes: disable servos
constexpr uint32_t RESET_MCU      = 0x45c;  // 0 bytes: NVIC_SystemReset

// MCU → Host
constexpr uint32_t SEND_ANGLES    = 0x46a;  // 6 bytes: uint16_t[3] raw encoder angles
                                             // [wrist, grip, unused] (only first 4 bytes valid
                                             // when NUM_ENCODERS=2)

}  // namespace gripper_can_ids

#endif  // GRIPPER_INTERFACE__GRIPPER_CAN_IDS_HPP_
