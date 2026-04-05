#include <gtest/gtest.h>
#include <cmath>
#include <cstdint>

#include "gripper_interface/gripper_interface_translator.hpp"
#include "gripper_interface/gripper_interface_typedefs.hpp"

using namespace gripper_interface;

// ---------------------------------------------------------------------------
// Helper: decode a big-endian uint16_t from two consecutive bytes
// ---------------------------------------------------------------------------
static uint16_t decode_be(uint8_t msb, uint8_t lsb)
{
    return static_cast<uint16_t>((static_cast<uint16_t>(msb) << 8) | lsb);
}

// ===========================================================================
// bytes_to_uint16
// ===========================================================================

TEST(BytesToUint16, Zero_ProducesZero)
{
    EXPECT_EQ(translator::bytes_to_uint16(0x00, 0x00), 0u);
}

TEST(BytesToUint16, MsbOnly_ShiftedCorrectly)
{
    EXPECT_EQ(translator::bytes_to_uint16(0x01, 0x00), 256u);
}

TEST(BytesToUint16, LsbOnly_NotShifted)
{
    EXPECT_EQ(translator::bytes_to_uint16(0x00, 0xFF), 255u);
}

TEST(BytesToUint16, BothBytes_BigEndianAssembly)
{
    // 0x12 << 8 | 0x34 = 0x1234 = 4660
    EXPECT_EQ(translator::bytes_to_uint16(0x12, 0x34), 0x1234u);
}

TEST(BytesToUint16, MaxValue_ProducesFFFF)
{
    EXPECT_EQ(translator::bytes_to_uint16(0xFF, 0xFF), 0xFFFFu);
}

// ===========================================================================
// raw_angle_to_radians
// ===========================================================================

TEST(RawAngleToRadians, Zero_ProducesZero)
{
    EXPECT_DOUBLE_EQ(translator::raw_angle_to_radians(0), 0.0);
}

TEST(RawAngleToRadians, HalfScale_ProducesPi)
{
    // 8192 / 16384 * 2π = π
    const double expected = (8192.0 / 16384.0) * 2.0 * M_PI;
    EXPECT_NEAR(translator::raw_angle_to_radians(8192), expected, 1e-10);
}

TEST(RawAngleToRadians, FullScale14Bit_JustUnderTwoPi)
{
    // 0x3FFF = 16383 — largest valid 14-bit value
    const double expected = (16383.0 / 16384.0) * 2.0 * M_PI;
    EXPECT_NEAR(translator::raw_angle_to_radians(0x3FFF), expected, 1e-10);
}

TEST(RawAngleToRadians, Bit14Set_MaskedToZero)
{
    // 0x4000 — bit 14 set, after 0x3FFF mask → 0 → 0 radians
    EXPECT_DOUBLE_EQ(translator::raw_angle_to_radians(0x4000), 0.0);
}

TEST(RawAngleToRadians, UpperBitsMasked_EquivalentToMaskedValue)
{
    // 0xFFFF masked with 0x3FFF == 0x3FFF
    EXPECT_DOUBLE_EQ(translator::raw_angle_to_radians(0xFFFF),
                     translator::raw_angle_to_radians(0x3FFF));
}

TEST(RawAngleToRadians, OutputAlwaysNonNegative)
{
    // Masking ensures raw is never negative after cast
    EXPECT_GE(translator::raw_angle_to_radians(0x8000), 0.0);
    EXPECT_GE(translator::raw_angle_to_radians(0xFFFF), 0.0);
}

// ===========================================================================
// encoder_frame_to_gripper_state
// ===========================================================================

TEST(EncoderFrameToGripperState, ZeroFrame_ProducesZeroState)
{
    types::RawEncoderFrame frame{};
    auto state = translator::encoder_frame_to_gripper_state(frame);

    EXPECT_DOUBLE_EQ(state.roll,  0.0);
    EXPECT_DOUBLE_EQ(state.pinch, 0.0);
}

TEST(EncoderFrameToGripperState, RollBytesOnly_PinchRemainsZero)
{
    types::RawEncoderFrame frame{};
    frame.bytes[0] = 0x20;
    frame.bytes[1] = 0x00;

    auto state = translator::encoder_frame_to_gripper_state(frame);

    EXPECT_GT(state.roll, 0.0);
    EXPECT_DOUBLE_EQ(state.pinch, 0.0);
}

TEST(EncoderFrameToGripperState, PinchBytesOnly_RollRemainsZero)
{
    types::RawEncoderFrame frame{};
    frame.bytes[2] = 0x20;
    frame.bytes[3] = 0x00;

    auto state = translator::encoder_frame_to_gripper_state(frame);

    EXPECT_DOUBLE_EQ(state.roll, 0.0);
    EXPECT_GT(state.pinch, 0.0);
}

TEST(EncoderFrameToGripperState, IdenticalRawValues_ProducesIdenticalAngles)
{
    // With GEAR_RATIO_ROLL == GEAR_RATIO_PINCH == 1.0, same raw → same output
    types::RawEncoderFrame frame{};
    frame.bytes[0] = 0x10;
    frame.bytes[1] = 0x00;
    frame.bytes[2] = 0x10;
    frame.bytes[3] = 0x00;

    auto state = translator::encoder_frame_to_gripper_state(frame);

    EXPECT_DOUBLE_EQ(state.roll, state.pinch);
}

TEST(EncoderFrameToGripperState, RollMatchesManualCalculation)
{
    types::RawEncoderFrame frame{};
    frame.bytes[0] = 0x10;
    frame.bytes[1] = 0x80;

    const uint16_t raw      = translator::bytes_to_uint16(0x10, 0x80);
    const double   expected = translator::raw_angle_to_radians(raw) * types::GEAR_RATIO_ROLL;

    auto state = translator::encoder_frame_to_gripper_state(frame);

    EXPECT_DOUBLE_EQ(state.roll, expected);
}

TEST(EncoderFrameToGripperState, PinchMatchesManualCalculation)
{
    types::RawEncoderFrame frame{};
    frame.bytes[2] = 0x08;
    frame.bytes[3] = 0x40;

    const uint16_t raw      = translator::bytes_to_uint16(0x08, 0x40);
    const double   expected = translator::raw_angle_to_radians(raw) * types::GEAR_RATIO_PINCH;

    auto state = translator::encoder_frame_to_gripper_state(frame);

    EXPECT_DOUBLE_EQ(state.pinch, expected);
}

TEST(EncoderFrameToGripperState, UnusedBytes_DoNotAffectOutput)
{
    types::RawEncoderFrame frame_a{};
    frame_a.bytes[0] = 0x10;
    frame_a.bytes[1] = 0x00;
    frame_a.bytes[2] = 0x08;
    frame_a.bytes[3] = 0x00;
    frame_a.bytes[4] = 0x00;
    frame_a.bytes[5] = 0x00;

    types::RawEncoderFrame frame_b = frame_a;
    frame_b.bytes[4] = 0xFF;
    frame_b.bytes[5] = 0xFF;

    auto state_a = translator::encoder_frame_to_gripper_state(frame_a);
    auto state_b = translator::encoder_frame_to_gripper_state(frame_b);

    EXPECT_DOUBLE_EQ(state_a.roll,  state_b.roll);
    EXPECT_DOUBLE_EQ(state_a.pinch, state_b.pinch);
}

// ===========================================================================
// velocity_to_duty_us
// ===========================================================================

TEST(VelocityToDutyUs, ZeroVelocity_ProducesIdlePwm)
{
    EXPECT_EQ(translator::velocity_to_duty_us(0.0, 1.0),
              static_cast<uint16_t>(types::PWM_IDLE_US));
}

TEST(VelocityToDutyUs, MaxPositiveVelocity_ProducesMaxPwm)
{
    EXPECT_EQ(translator::velocity_to_duty_us(1.0, 1.0),
              static_cast<uint16_t>(types::PWM_MAX_US));
}

TEST(VelocityToDutyUs, MaxNegativeVelocity_ProducesMinPwm)
{
    EXPECT_EQ(translator::velocity_to_duty_us(-1.0, 1.0),
              static_cast<uint16_t>(types::PWM_MIN_US));
}

TEST(VelocityToDutyUs, HalfPositiveVelocity_MidpointBetweenIdleAndMax)
{
    const uint16_t expected =
        static_cast<uint16_t>(types::PWM_IDLE_US + 0.5 * types::PWM_RANGE_US);

    EXPECT_EQ(translator::velocity_to_duty_us(0.5, 1.0), expected);
}

TEST(VelocityToDutyUs, HalfNegativeVelocity_MidpointBetweenMinAndIdle)
{
    const uint16_t expected =
        static_cast<uint16_t>(types::PWM_IDLE_US - 0.5 * types::PWM_RANGE_US);

    EXPECT_EQ(translator::velocity_to_duty_us(-0.5, 1.0), expected);
}

TEST(VelocityToDutyUs, OverspeedPositive_ClampsToMaxPwm)
{
    EXPECT_EQ(translator::velocity_to_duty_us(999.0, 1.0),
              static_cast<uint16_t>(types::PWM_MAX_US));
}

TEST(VelocityToDutyUs, OverspeedNegative_ClampsToMinPwm)
{
    EXPECT_EQ(translator::velocity_to_duty_us(-999.0, 1.0),
              static_cast<uint16_t>(types::PWM_MIN_US));
}

TEST(VelocityToDutyUs, ClampingIsSymmetricAboutIdle)
{
    const uint16_t pos = translator::velocity_to_duty_us( 999.0, 1.0);
    const uint16_t neg = translator::velocity_to_duty_us(-999.0, 1.0);

    EXPECT_EQ(pos + neg, static_cast<uint16_t>(2 * types::PWM_IDLE_US));
}

TEST(VelocityToDutyUs, OutputAlwaysWithinHardwareLimits)
{
    for (double v : {-10.0, -1.0, -0.5, 0.0, 0.5, 1.0, 10.0}) {
        const uint16_t duty = translator::velocity_to_duty_us(v, 1.0);
        EXPECT_GE(duty, static_cast<uint16_t>(types::PWM_MIN_US));
        EXPECT_LE(duty, static_cast<uint16_t>(types::PWM_MAX_US));
    }
}

// ===========================================================================
// velocity_command_to_pwm_frame
// ===========================================================================

TEST(VelocityCommandToPwmFrame, ZeroCommand_AllBytesEncodeIdlePwm)
{
    vortex_msgs::msg::GripperStateVelocityCommand cmd;
    cmd.roll_velocity  = 0.0;
    cmd.pinch_velocity = 0.0;

    auto frame = translator::velocity_command_to_pwm_frame(cmd);

    EXPECT_EQ(decode_be(frame.bytes[0], frame.bytes[1]),
              static_cast<uint16_t>(types::PWM_IDLE_US));
    EXPECT_EQ(decode_be(frame.bytes[2], frame.bytes[3]),
              static_cast<uint16_t>(types::PWM_IDLE_US));
}

TEST(VelocityCommandToPwmFrame, FullPositiveRoll_RollBytesEncodeMaxPwm_PinchIdle)
{
    vortex_msgs::msg::GripperStateVelocityCommand cmd;
    cmd.roll_velocity  =  types::MAX_ROLL_VEL;
    cmd.pinch_velocity = 0.0;

    auto frame = translator::velocity_command_to_pwm_frame(cmd);

    EXPECT_EQ(decode_be(frame.bytes[0], frame.bytes[1]),
              static_cast<uint16_t>(types::PWM_MAX_US));
    EXPECT_EQ(decode_be(frame.bytes[2], frame.bytes[3]),
              static_cast<uint16_t>(types::PWM_IDLE_US));
}

TEST(VelocityCommandToPwmFrame, FullNegativeRoll_RollBytesEncodeMinPwm_PinchIdle)
{
    vortex_msgs::msg::GripperStateVelocityCommand cmd;
    cmd.roll_velocity  = -types::MAX_ROLL_VEL;
    cmd.pinch_velocity = 0.0;

    auto frame = translator::velocity_command_to_pwm_frame(cmd);

    EXPECT_EQ(decode_be(frame.bytes[0], frame.bytes[1]),
              static_cast<uint16_t>(types::PWM_MIN_US));
    EXPECT_EQ(decode_be(frame.bytes[2], frame.bytes[3]),
              static_cast<uint16_t>(types::PWM_IDLE_US));
}

TEST(VelocityCommandToPwmFrame, FullPositivePinch_PinchBytesEncodeMaxPwm_RollIdle)
{
    vortex_msgs::msg::GripperStateVelocityCommand cmd;
    cmd.roll_velocity  = 0.0;
    cmd.pinch_velocity = types::MAX_PINCH_VEL;

    auto frame = translator::velocity_command_to_pwm_frame(cmd);

    EXPECT_EQ(decode_be(frame.bytes[0], frame.bytes[1]),
              static_cast<uint16_t>(types::PWM_IDLE_US));
    EXPECT_EQ(decode_be(frame.bytes[2], frame.bytes[3]),
              static_cast<uint16_t>(types::PWM_MAX_US));
}

TEST(VelocityCommandToPwmFrame, FullNegativePinch_PinchBytesEncodeMinPwm_RollIdle)
{
    vortex_msgs::msg::GripperStateVelocityCommand cmd;
    cmd.roll_velocity  = 0.0;
    cmd.pinch_velocity = -types::MAX_PINCH_VEL;

    auto frame = translator::velocity_command_to_pwm_frame(cmd);

    EXPECT_EQ(decode_be(frame.bytes[0], frame.bytes[1]),
              static_cast<uint16_t>(types::PWM_IDLE_US));
    EXPECT_EQ(decode_be(frame.bytes[2], frame.bytes[3]),
              static_cast<uint16_t>(types::PWM_MIN_US));
}

TEST(VelocityCommandToPwmFrame, AxesAreIndependent_OppositeExtremes)
{
    vortex_msgs::msg::GripperStateVelocityCommand cmd;
    cmd.roll_velocity  =  types::MAX_ROLL_VEL;
    cmd.pinch_velocity = -types::MAX_PINCH_VEL;

    auto frame = translator::velocity_command_to_pwm_frame(cmd);

    EXPECT_EQ(decode_be(frame.bytes[0], frame.bytes[1]),
              static_cast<uint16_t>(types::PWM_MAX_US));
    EXPECT_EQ(decode_be(frame.bytes[2], frame.bytes[3]),
              static_cast<uint16_t>(types::PWM_MIN_US));
}

TEST(VelocityCommandToPwmFrame, BigEndianPacking_MsbInLowerByteIndex)
{
    // 0.5 * MAX_ROLL_VEL → duty = 1750 µs = 0x06D6
    // bytes[0] must be 0x06, bytes[1] must be 0xD6
    vortex_msgs::msg::GripperStateVelocityCommand cmd;
    cmd.roll_velocity  = 0.5 * types::MAX_ROLL_VEL;
    cmd.pinch_velocity = 0.0;

    auto frame = translator::velocity_command_to_pwm_frame(cmd);
    const uint16_t expected =
        static_cast<uint16_t>(types::PWM_IDLE_US + 0.5 * types::PWM_RANGE_US);

    EXPECT_EQ(frame.bytes[0], static_cast<uint8_t>((expected >> 8) & 0xFF));
    EXPECT_EQ(frame.bytes[1], static_cast<uint8_t>( expected       & 0xFF));
}

TEST(VelocityCommandToPwmFrame, PwmSymmetry_PositiveAndNegativeSymmetricAboutIdle)
{
    vortex_msgs::msg::GripperStateVelocityCommand pos_cmd;
    pos_cmd.roll_velocity  =  0.7 * types::MAX_ROLL_VEL;
    pos_cmd.pinch_velocity = 0.0;

    vortex_msgs::msg::GripperStateVelocityCommand neg_cmd;
    neg_cmd.roll_velocity  = -0.7 * types::MAX_ROLL_VEL;
    neg_cmd.pinch_velocity = 0.0;

    auto pos_frame = translator::velocity_command_to_pwm_frame(pos_cmd);
    auto neg_frame = translator::velocity_command_to_pwm_frame(neg_cmd);

    const uint16_t pos_us = decode_be(pos_frame.bytes[0], pos_frame.bytes[1]);
    const uint16_t neg_us = decode_be(neg_frame.bytes[0], neg_frame.bytes[1]);

    EXPECT_EQ(pos_us + neg_us, static_cast<uint16_t>(2 * types::PWM_IDLE_US));
}

TEST(VelocityCommandToPwmFrame, AllOutputBytesWithinHardwarePwmRange)
{
    for (double rv : {-2.0, -1.0, 0.0, 1.0, 2.0}) {
        for (double pv : {-2.0, -1.0, 0.0, 1.0, 2.0}) {
            vortex_msgs::msg::GripperStateVelocityCommand cmd;
            cmd.roll_velocity  = rv;
            cmd.pinch_velocity = pv;

            auto frame = translator::velocity_command_to_pwm_frame(cmd);

            const uint16_t roll_us  = decode_be(frame.bytes[0], frame.bytes[1]);
            const uint16_t pinch_us = decode_be(frame.bytes[2], frame.bytes[3]);

            EXPECT_GE(roll_us,  static_cast<uint16_t>(types::PWM_MIN_US));
            EXPECT_LE(roll_us,  static_cast<uint16_t>(types::PWM_MAX_US));
            EXPECT_GE(pinch_us, static_cast<uint16_t>(types::PWM_MIN_US));
            EXPECT_LE(pinch_us, static_cast<uint16_t>(types::PWM_MAX_US));
        }
    }
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
