#ifndef GRIPPER_INTERFACE__GRIPPER_INTERFACE_TYPEDEFS_HPP_
#define GRIPPER_INTERFACE__GRIPPER_INTERFACE_TYPEDEFS_HPP_

#include <cstdint>
#include <array>

namespace gripper_interface::types {

// ---------------------------------------------------------------------------
// Raw CAN payloads — exactly what crosses the CAN bus.
// All multi-byte fields are big-endian (MCU packs MSB first).
// ---------------------------------------------------------------------------

// MCU → Host: 6 bytes, indices [0..1]=wrist, [2..3]=grip, [4..5]=unused
// Raw 14-bit encoder value (AS5048 / compatible) packed in uint16_t.
// NOTE: firmware bug suspected in read_encoders() — buf = out + enc_num
// should likely be out + enc_num*2. Verify on hardware before trusting
// bytes 2-3 when running NUM_ENCODERS=2.
struct RawEncoderFrame {
    std::array<uint8_t, 6> bytes{};
};

// Host → MCU: 4 bytes = uint16_t[2], duty cycle in microseconds [roll, pinch]
// Servo range: 700 µs (full reverse) – 1500 µs (idle) – 2300 µs (full forward)
struct RawPwmFrame {
    std::array<uint8_t, 4> bytes{};
};

// Decoded encoder angles in radians
struct EncoderAngles {
    double roll  = 0.0;  // wrist encoder → roll
    double pinch = 0.0;  // grip  encoder → pinch
};

// Velocity command from controller (rad/s)
struct VelocityCommand {
    double roll_vel  = 0.0;
    double pinch_vel = 0.0;
};

// ---------------------------------------------------------------------------
// Servo / encoder constants
// ---------------------------------------------------------------------------
constexpr double ENCODER_COUNTS    = 16383.0;  // 0x3FFF — 14-bit AS5048
constexpr double TWO_PI            = 6.283185307179586;

// Servo PWM limits (microseconds) — from tcc.c comments
constexpr uint16_t PWM_IDLE_US     = 1500;
constexpr uint16_t PWM_MAX_US      = 2300;
constexpr uint16_t PWM_MIN_US      = 700;
constexpr uint16_t PWM_RANGE_US    = PWM_MAX_US - PWM_IDLE_US;  // 800

// Gear ratios — both assumed 1.0 until experimentally calibrated
// TODO: measure worm/leadscrew reduction after servo installation
constexpr double GEAR_RATIO_ROLL   = 1.0;
constexpr double GEAR_RATIO_PINCH  = 1.0;

// Velocity saturation — must match types::MAX_ROLL_VEL / MAX_PINCH_VEL
// in gripper_controller_typedefs.hpp
constexpr double MAX_ROLL_VEL      = 1.0;  // rad/s
constexpr double MAX_PINCH_VEL     = 1.0;  // rad/s

}  // namespace gripper_interface::types

#endif  // GRIPPER_INTERFACE__GRIPPER_INTERFACE_TYPEDEFS_HPP_
