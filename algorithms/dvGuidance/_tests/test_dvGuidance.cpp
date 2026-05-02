#include "dvGuidanceTestHelpers.hpp"
#include <gtest/gtest.h>

TEST(DvGuidanceTest, Setup) { testDvGuidanceSetup(); }

TEST(DvGuidanceTest, ReferenceTestAtBurnStart) {
    // Sampled at the burn start time so the rotation about the 3rd burn-frame axis is zero.
    testDvGuidance(Eigen::Vector3f{5.0F, 5.0F, 5.0F},  // dvInrtlCmd
                   Eigen::Vector3f{1.0F, 0.0F, 0.0F},  // dvRotVecUnit (orthogonal seed)
                   0.5F,                               // dvRotVecMag
                   /* burnStartTime = */ 500000000U,   // 0.5 s
                   /* callTime      = */ 500000000U);  // 0.5 s
}

TEST(DvGuidanceTest, ReferenceTestMidBurn) {
    // 0.5 s into the burn, rotation = dvRotVecMag * dt = 0.25 rad about the 3rd burn-frame axis.
    testDvGuidance(Eigen::Vector3f{5.0F, 5.0F, 5.0F},
                   Eigen::Vector3f{1.0F, 0.0F, 0.0F},
                   0.5F,
                   /* burnStartTime = */ 500000000U,
                   /* callTime      = */ 1000000000U);  // 1.0 s
}

TEST(DvGuidanceTest, ReferenceTestPrelaunch) {
    // callTime < burnStartTime: burnTime is negative; rotation is in the opposite sense.
    testDvGuidance(Eigen::Vector3f{1.0F, 2.0F, -3.0F},
                   Eigen::Vector3f{0.0F, 1.0F, 0.0F},
                   0.1F,
                   /* burnStartTime = */ 1000000000U,  // 1.0 s
                   /* callTime      = */ 500000000U);  // 0.5 s
}
