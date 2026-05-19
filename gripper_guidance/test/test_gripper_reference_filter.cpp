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

    Eigen::Vector2d integrate_to_steady_state(const Eigen::Vector2d& initial_reference,
                                              const Eigen::Vector2d& goal_reference,
                                              int steps,
                                              double time_step_seconds = 0.01) {
        filter->reset(initial_reference);
        for (int i = 0; i < steps; ++i) {
            filter->step(goal_reference, time_step_seconds);
        }
        return filter->reference_output();
    }

    bool position_converged_to_reference(const Eigen::Vector2d& output,
                                         const Eigen::Vector2d& goal_reference,
                                         double tol = 1e-2) {
        return std::abs(output(0) - goal_reference(0)) < tol &&
               std::abs(output(1) - goal_reference(1)) < tol;
    }

    bool no_overshoot(const Eigen::Vector2d& output,
                      const Eigen::Vector2d& goal_reference,
                      double tol = 1e-6) {
        return output(0) <= goal_reference(0) + tol &&
               output(1) <= goal_reference(1) + tol;
    }

    std::unique_ptr<GripperReferenceFilter> filter;
};

// Reset seeds the position output but produces no motion until step() is called.
TEST_F(ReferenceFilterTest, ResetSeedsPositionOutput) {
    Eigen::Vector2d seed;
    seed << 0.5, -0.25;

    filter->reset(seed);

    EXPECT_TRUE(filter->reference_output().isApprox(seed, 1e-9));
}

// Step response: filter output converges to the goal reference over time
// (Fossen Eq. 12.11-12.12).
TEST_F(ReferenceFilterTest, StepResponseConverges) {
    Eigen::Vector2d goal;
    goal << 1.0, 0.5;

    const Eigen::Vector2d output =
        integrate_to_steady_state(Eigen::Vector2d::Zero(), goal, 2000);

    EXPECT_TRUE(position_converged_to_reference(output, goal));
}

// Critically damped (zeta=1) should produce no overshoot.
TEST_F(ReferenceFilterTest, NoCriticallyDampedOvershoot) {
    Eigen::Vector2d goal;
    goal << 1.0, 1.0;

    filter->reset(Eigen::Vector2d::Zero());
    for (int i = 0; i < 2000; ++i) {
        filter->step(goal, 0.01);
        EXPECT_TRUE(no_overshoot(filter->reference_output(), goal));
    }
}

// snap_to forces the position output to match the goal exactly.
TEST_F(ReferenceFilterTest, SnapToMatchesGoal) {
    Eigen::Vector2d goal;
    goal << 3.14, -0.7;

    filter->reset(Eigen::Vector2d::Zero());
    filter->snap_to(goal);

    EXPECT_TRUE(filter->reference_output().isApprox(goal, 1e-9));
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
