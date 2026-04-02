#ifndef GRIPPER_INTERFACE__GRIPPER_INTERFACE_TRANSLATOR_HPP_
#define GRIPPER_INTERFACE__GRIPPER_INTERFACE_TRANSLATOR_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <vortex_msgs/msg/gripper_state.hpp>
#include <vortex_msgs/msg/gripper_state_velocity_command.hpp>

#include "gripper_interface/gripper_interface_typedefs.hpp"

// ---------------------------------------------------------------------------
// Responsibility: pure, stateless conversions between raw CAN byte payloads
// and ROS/domain types. No I2C, no file descriptors, no ROS dependencies
// beyond message types.
//
// Encoding conventions (matching MCU firmware):
//   - Encoder values: big-endian uint16_t, 14-bit (mask with 0x3FFF)
//   - PWM values:     big-endian uint16_t, microseconds
// ---------------------------------------------------------------------------

namespace gripper_interface::translator {

// ---------------------------------------------------------------------------
// Encoder helpers (mirroring MCU's raw_angle_to_radians / i2c_to_encoder_angles)
// ---------------------------------------------------------------------------

inline uint16_t bytes_to_uint16(uint8_t msb, uint8_t lsb) {
    return static_cast<uint16_t>((static_cast<uint16_t>(msb) << 8) | lsb);
}

inline double raw_angle_to_radians(uint16_t raw) {
    return (static_cast<double>(raw & 0x3FFF) / types::ENCODER_COUNTS) * types::TWO_PI;
}

// ---------------------------------------------------------------------------
// RawEncoderFrame → GripperState (position, radians)
//
// Byte layout from MCU (NUM_ENCODERS=2 path):
//   bytes[0..1] = wrist encoder  → roll
//   bytes[2..3] = grip  encoder  → pinch
//   bytes[4..5] = unused
//
// WARNING: firmware read_encoders() uses `buf = out + enc_num` instead of
// `out + enc_num * 2`. Verify byte layout on hardware — if enc_num indexing
// is wrong the bytes[2..3] pinch data may be corrupted.
// ---------------------------------------------------------------------------
inline vortex_msgs::msg::GripperState
encoder_frame_to_gripper_state(const types::RawEncoderFrame& frame) {
    const uint16_t raw_roll  = bytes_to_uint16(frame.bytes[0], frame.bytes[1]);
    const uint16_t raw_pinch = bytes_to_uint16(frame.bytes[2], frame.bytes[3]);

    vortex_msgs::msg::GripperState msg;
    msg.roll  = raw_angle_to_radians(raw_roll)  * types::GEAR_RATIO_ROLL;
    msg.pinch = raw_angle_to_radians(raw_pinch) * types::GEAR_RATIO_PINCH;
    return msg;
}

// ---------------------------------------------------------------------------
// VelocityCommand → RawPwmFrame
//
// Velocity is normalised by MAX_VEL, then mapped to [PWM_MIN_US, PWM_MAX_US].
// Idle (zero velocity) = PWM_IDLE_US = 1500 µs.
//
// Formula: duty_us = IDLE + clamp(vel / MAX_VEL, -1, 1) * RANGE
//
// The 4-byte payload sent on CAN SET_PWM:
//   bytes[0..1] = roll  duty (µs) big-endian uint16_t
//   bytes[2..3] = pinch duty (µs) big-endian uint16_t
// ---------------------------------------------------------------------------
inline uint16_t velocity_to_duty_us(double velocity, double max_velocity) {
    const double normalised = std::clamp(velocity / max_velocity, -1.0, 1.0);
    const double duty = types::PWM_IDLE_US + normalised * types::PWM_RANGE_US;
    return static_cast<uint16_t>(
        std::clamp(duty,
                   static_cast<double>(types::PWM_MIN_US),
                   static_cast<double>(types::PWM_MAX_US)));
}

inline types::RawPwmFrame velocity_command_to_pwm_frame(
    const vortex_msgs::msg::GripperStateVelocityCommand& cmd) {

    const uint16_t roll_us  = velocity_to_duty_us(cmd.roll_velocity,  types::MAX_ROLL_VEL);
    const uint16_t pinch_us = velocity_to_duty_us(cmd.pinch_velocity, types::MAX_PINCH_VEL);

    types::RawPwmFrame frame;
    // Big-endian packing — MSB first, matching MCU memcpy expectation
    frame.bytes[0] = static_cast<uint8_t>((roll_us  >> 8) & 0xFF);
    frame.bytes[1] = static_cast<uint8_t>( roll_us        & 0xFF);
    frame.bytes[2] = static_cast<uint8_t>((pinch_us >> 8) & 0xFF);
    frame.bytes[3] = static_cast<uint8_t>( pinch_us       & 0xFF);
    return frame;
}

}  // namespace gripper_interface::translator

#endif  // GRIPPER_INTERFACE__GRIPPER_INTERFACE_TRANSLATOR_HPP_
