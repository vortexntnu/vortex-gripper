#include <gtest/gtest.h>
#include "gripper_controller/gripper_controller.hpp"
#include "gripper_controller/gripper_controller_typedefs.hpp"

// ---------------------------------------------------------------------------
// Test fixture
// Sets up a GripperController with identity Kp (gain = 1.0 on both axes)
// so that expected output equals the position error exactly, up to saturation.
// ---------------------------------------------------------------------------
class GripperControllerTest : public ::testing::Test {
   protected:
    void SetUp() override {
        types::Matrix2d identity_gain = types::Matrix2d::Identity();
        controller_.set_kp(identity_gain);
        controller_.set_time_step(0.01);
    }

    // @brief Helper: run calculate_velocity with explicit scalar inputs.
    types::Vector2d compute_velocity(double measured_roll,
                                     double measured_pinch,
                                     double reference_roll,
                                     double reference_pinch) {
        types::GripperState measured_state;
        measured_state.roll  = measured_roll;
        measured_state.pinch = measured_pinch;

        types::GripperState reference_state;
        reference_state.roll  = reference_roll;
        reference_state.pinch = reference_pinch;

        return controller_.calculate_velocity(measured_state, reference_state);
    }

    GripperController controller_;
};

// ---------------------------------------------------------------------------
// Zero error
// ---------------------------------------------------------------------------

TEST_F(GripperControllerTest, ZeroError_ProducesZeroVelocity) {
    const types::Vector2d velocity_command = compute_velocity(0.0, 0.0, 0.0, 0.0);

    EXPECT_DOUBLE_EQ(velocity_command(0), 0.0);
    EXPECT_DOUBLE_EQ(velocity_command(1), 0.0);
}

TEST_F(GripperControllerTest, StateEqualsReference_ProducesZeroVelocity) {
    const types::Vector2d velocity_command = compute_velocity(0.5, 0.3, 0.5, 0.3);

    EXPECT_DOUBLE_EQ(velocity_command(0), 0.0);
    EXPECT_DOUBLE_EQ(velocity_command(1), 0.0);
}

// ---------------------------------------------------------------------------
// Correct sign and axis isolation
// ---------------------------------------------------------------------------

TEST_F(GripperControllerTest, PositiveRollError_ProducesPositiveRollVelocity) {
    const types::Vector2d velocity_command = compute_velocity(0.0, 0.0, 0.5, 0.0);

    EXPECT_GT(velocity_command(0), 0.0);
    EXPECT_DOUBLE_EQ(velocity_command(1), 0.0);
}

TEST_F(GripperControllerTest, NegativeRollError_ProducesNegativeRollVelocity) {
    const types::Vector2d velocity_command = compute_velocity(0.5, 0.0, 0.0, 0.0);

    EXPECT_LT(velocity_command(0), 0.0);
    EXPECT_DOUBLE_EQ(velocity_command(1), 0.0);
}

TEST_F(GripperControllerTest, PositivePinchError_ProducesPositivePinchVelocity) {
    const types::Vector2d velocity_command = compute_velocity(0.0, 0.0, 0.0, 0.5);

    EXPECT_DOUBLE_EQ(velocity_command(0), 0.0);
    EXPECT_GT(velocity_command(1), 0.0);
}

TEST_F(GripperControllerTest, NegativePinchError_ProducesNegativePinchVelocity) {
    const types::Vector2d velocity_command = compute_velocity(0.0, 0.5, 0.0, 0.0);

    EXPECT_DOUBLE_EQ(velocity_command(0), 0.0);
    EXPECT_LT(velocity_command(1), 0.0);
}

// ---------------------------------------------------------------------------
// Proportionality — with identity Kp, output equals error exactly
// (as long as error stays within saturation limits)
// ---------------------------------------------------------------------------

TEST_F(GripperControllerTest, OutputProportionalToError_WithinSaturationLimits) {
    const double roll_error  = 0.3;
    const double pinch_error = 0.6;

    const types::Vector2d velocity_command =
        compute_velocity(0.0, 0.0, roll_error, pinch_error);

    EXPECT_NEAR(velocity_command(0), roll_error,  1e-9);
    EXPECT_NEAR(velocity_command(1), pinch_error, 1e-9);
}

// ---------------------------------------------------------------------------
// Velocity saturation
// ---------------------------------------------------------------------------

TEST_F(GripperControllerTest, LargePositiveRollError_SaturatesAtMaxRollVelocity) {
    // Error of 100.0 with gain 1.0 would produce 100.0 — well above MAX_ROLL_VEL
    const types::Vector2d velocity_command = compute_velocity(0.0, 0.0, 100.0, 0.0);

    EXPECT_DOUBLE_EQ(velocity_command(0), types::MAX_ROLL_VEL);
}

TEST_F(GripperControllerTest, LargeNegativeRollError_SaturatesAtNegativeMaxRollVelocity) {
    const types::Vector2d velocity_command = compute_velocity(100.0, 0.0, 0.0, 0.0);

    EXPECT_DOUBLE_EQ(velocity_command(0), -types::MAX_ROLL_VEL);
}

TEST_F(GripperControllerTest, LargePositivePinchError_SaturatesAtMaxPinchVelocity) {
    const types::Vector2d velocity_command = compute_velocity(0.0, 0.0, 0.0, 100.0);

    EXPECT_DOUBLE_EQ(velocity_command(1), types::MAX_PINCH_VEL);
}

TEST_F(GripperControllerTest, LargeNegativePinchError_SaturatesAtNegativeMaxPinchVelocity) {
    const types::Vector2d velocity_command = compute_velocity(0.0, 100.0, 0.0, 0.0);

    EXPECT_DOUBLE_EQ(velocity_command(1), -types::MAX_PINCH_VEL);
}

TEST_F(GripperControllerTest, SaturationIsSymmetric_RollAxis) {
    const types::Vector2d positive_saturation = compute_velocity(0.0, 0.0,  100.0, 0.0);
    const types::Vector2d negative_saturation = compute_velocity(100.0, 0.0, 0.0,  0.0);

    EXPECT_DOUBLE_EQ(positive_saturation(0), -negative_saturation(0));
}

TEST_F(GripperControllerTest, SaturationIsSymmetric_PinchAxis) {
    const types::Vector2d positive_saturation = compute_velocity(0.0, 0.0,   0.0, 100.0);
    const types::Vector2d negative_saturation = compute_velocity(0.0, 100.0, 0.0, 0.0);

    EXPECT_DOUBLE_EQ(positive_saturation(1), -negative_saturation(1));
}

// ---------------------------------------------------------------------------
// Gain matrix effects
// ---------------------------------------------------------------------------

TEST_F(GripperControllerTest, ZeroGainMatrix_AlwaysProducesZeroVelocity) {
    controller_.set_kp(types::Matrix2d::Zero());

    const types::Vector2d velocity_command = compute_velocity(0.0, 0.0, 1.0, 1.0);

    EXPECT_DOUBLE_EQ(velocity_command(0), 0.0);
    EXPECT_DOUBLE_EQ(velocity_command(1), 0.0);
}

TEST_F(GripperControllerTest, ScaledDiagonalGain_ScalesOutputCorrectly) {
    types::Matrix2d scaled_gain = types::Matrix2d::Zero();
    scaled_gain(0, 0) = 0.5;  // roll gain
    scaled_gain(1, 1) = 2.0;  // pinch gain
    controller_.set_kp(scaled_gain);

    // Error of 0.4 on both axes — both stay within saturation after scaling
    const types::Vector2d velocity_command = compute_velocity(0.0, 0.0, 0.4, 0.4);

    EXPECT_NEAR(velocity_command(0), 0.5 * 0.4, 1e-9);
    EXPECT_NEAR(velocity_command(1), 2.0 * 0.4, 1e-9);
}

TEST_F(GripperControllerTest, OffDiagonalGain_CrossCouplingPropagatesCorrectly) {
    // Set cross-coupling: roll error should also drive pinch output
    types::Matrix2d cross_coupled_gain = types::Matrix2d::Identity();
    cross_coupled_gain(1, 0) = 0.5;  // pinch output gains from roll error
    controller_.set_kp(cross_coupled_gain);

    // Pure roll error of 0.4
    const types::Vector2d velocity_command = compute_velocity(0.0, 0.0, 0.4, 0.0);

    EXPECT_NEAR(velocity_command(0), 0.4,        1e-9);  // direct roll term
    EXPECT_NEAR(velocity_command(1), 0.5 * 0.4,  1e-9);  // cross-coupled pinch term
}

// ---------------------------------------------------------------------------
// set_kp takes effect on the next call
// ---------------------------------------------------------------------------

TEST_F(GripperControllerTest, UpdatedGain_TakesEffectImmediately) {
    // First call with identity gain
    const types::Vector2d first_velocity = compute_velocity(0.0, 0.0, 0.4, 0.0);
    EXPECT_NEAR(first_velocity(0), 0.4, 1e-9);

    // Double the roll gain
    types::Matrix2d doubled_gain = types::Matrix2d::Identity();
    doubled_gain(0, 0) = 2.0;
    controller_.set_kp(doubled_gain);

    const types::Vector2d second_velocity = compute_velocity(0.0, 0.0, 0.4, 0.0);
    EXPECT_NEAR(second_velocity(0), 0.8, 1e-9);
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
