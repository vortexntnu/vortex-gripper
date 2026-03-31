#include <gtest/gtest.h>
#include "gripper_reference_filter/gripper_reference_filter.hpp"

using namespace vortex::guidance;

class ReferenceFilterTest : public ::testing::Test {
protected:
    void SetUp() override {
        GripperReferenceFilterParams params;
        params.omega << 1.0, 1.0;
        params.zeta  << 1.0, 1.0;
        filter = std::make_unique<GripperReferenceFilter>(params);
    }

    Eigen::Vector6d integrate_steps(Eigen::Vector6d x, const Eigen::Vector2d& r, int steps, double dt = 0.01) {
        for (int i = 0; i < steps; ++i) {
            x += dt * filter->calculate_x_dot(x, r);
        }
        return x;
    }

    bool position_converged_to_reference(const Eigen::Vector6d& x, const Eigen::Vector2d& r, double tol = 1e-2) {
        return std::abs(x(0) - r(0)) < tol && std::abs(x(1) - r(1)) < tol;
    }

    bool velocity_within_bounds(const Eigen::Vector6d& x, double vel_bound = 10.0, double acc_bound = 100.0) {
        return std::abs(x(2)) < vel_bound && std::abs(x(4)) < acc_bound;
    }

    bool no_overshoot(const Eigen::Vector6d& x, const Eigen::Vector2d& r, double tol = 1e-6) {
        return x(0) <= r(0) + tol && x(1) <= r(1) + tol;
    }

    std::unique_ptr<GripperReferenceFilter> filter;
};

// Zero input, zero state -> zero derivative
TEST_F(ReferenceFilterTest, ZeroInputZeroState) {
    Eigen::Vector6d x_dot = filter->calculate_x_dot(Eigen::Vector6d::Zero(), Eigen::Vector2d::Zero());
    EXPECT_TRUE(x_dot.isZero(1e-9));
}

// Step response: x_d should converge to r over time (Fossen Eq. 12.11-12.12)
TEST_F(ReferenceFilterTest, StepResponseConverges) {
    Eigen::Vector2d r;
    r << 1.0, 0.5;

    Eigen::Vector6d x = integrate_steps(Eigen::Vector6d::Zero(), r, 2000);

    EXPECT_TRUE(position_converged_to_reference(x, r));
}

// Velocity and acceleration states should stay bounded throughout transient
TEST_F(ReferenceFilterTest, VelocityAndAccelerationBounded) {
    Eigen::Vector2d r;
    r << 1.0, 1.0;
    Eigen::Vector6d x = Eigen::Vector6d::Zero();

    for (int i = 0; i < 1000; ++i) {
        x += 0.01 * filter->calculate_x_dot(x, r);
        EXPECT_TRUE(velocity_within_bounds(x));
    }
}

// Critically damped (zeta=1) should produce no overshoot
TEST_F(ReferenceFilterTest, NoCriticallyDampedOvershoot) {
    Eigen::Vector2d r;
    r << 1.0, 1.0;
    Eigen::Vector6d x = Eigen::Vector6d::Zero();

    for (int i = 0; i < 2000; ++i) {
        x += 0.01 * filter->calculate_x_dot(x, r);
        EXPECT_TRUE(no_overshoot(x, r));
    }
}

// namespace vortex::guidance

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
