#include "dvAccumulation/dvAccumulationAlgorithm.h"
#include "dvAccumulationTestHelpers.hpp"
#include "utilities/fsw/timeConstants.h"

#include <gtest/gtest.h>
#include <Eigen/Core>
#include <cmath>
#include <cstdint>

TEST(DvAccumulationTest, SetupTest) { testDvAccumulationSetup(); }

TEST(DvAccumulationTest, ReferenceTest) {
    /*! - a sequence exercising the first call (sets the time reference), integration over advancing
     *    callTimes, and a non-advancing callTime (gated out) — compared to the reference at every step */
    testDvAccumulation({
        {static_cast<uint64_t>(6e7), Eigen::Vector3f{0.1F, -0.2F, 0.3F}},     // first call: sets ref, no integrate
        {static_cast<uint64_t>(7e7), Eigen::Vector3f{0.2F, -0.1F, 0.4F}},     // integrate
        {static_cast<uint64_t>(8e7), Eigen::Vector3f{-0.1F, 0.5F, 0.1F}},     // integrate
        {static_cast<uint64_t>(8e7), Eigen::Vector3f{9.9F, 9.9F, 9.9F}},      // non-advancing -> gated
        {static_cast<uint64_t>(1.1e8), Eigen::Vector3f{-0.05F, 0.0F, 0.1F}},  // integrate
    });
}

TEST(DvAccumulationTest, FirstCallSetsTimeReferenceOnly) {
    /*! - the first update after reInitialize (previousTime == 0) only sets the time reference: zero DV,
     *    time-tag set to the call time */
    DvAccumulationAlgorithm alg{};
    alg.reInitialize();

    const DvAccumulationOutput out = alg.update(static_cast<uint64_t>(5e7), Eigen::Vector3f{1.0F, 2.0F, 3.0F});

    EXPECT_FLOAT_EQ(out.vehAccumDV_B[0], 0.0F);
    EXPECT_FLOAT_EQ(out.vehAccumDV_B[1], 0.0F);
    EXPECT_FLOAT_EQ(out.vehAccumDV_B[2], 0.0F);
    EXPECT_NEAR(out.timeTag, 5e7 * kNano2Sec, 1e-9);
}

TEST(DvAccumulationTest, NonAdvancingCallTimeDoesNotAccumulate) {
    /*! - feeding the same callTime twice integrates only once (strictly-greater gate) */
    DvAccumulationAlgorithm alg{};
    alg.reInitialize();

    const Eigen::Vector3f accel{0.5F, 0.0F, 0.0F};
    alg.update(static_cast<uint64_t>(1e7), accel);                                      // first call: sets ref
    const DvAccumulationOutput first = alg.update(static_cast<uint64_t>(2e7), accel);   // integrate
    const DvAccumulationOutput second = alg.update(static_cast<uint64_t>(2e7), accel);  // gated

    EXPECT_FLOAT_EQ(first.vehAccumDV_B[0], second.vehAccumDV_B[0]);
    EXPECT_FLOAT_EQ(first.vehAccumDV_B[1], second.vehAccumDV_B[1]);
    EXPECT_FLOAT_EQ(first.vehAccumDV_B[2], second.vehAccumDV_B[2]);
    EXPECT_DOUBLE_EQ(first.timeTag, second.timeTag);
}

TEST(DvAccumulationTest, BoundedInputProducesFiniteOutput) {
    /*! - bounded accel over a bounded callTime span keeps the accumulator finite */
    DvAccumulationAlgorithm alg{};
    alg.reInitialize();

    const Eigen::Vector3f accel{10.0F, -10.0F, 5.0F};
    DvAccumulationOutput out{};
    for (uint64_t k = 1U; k <= 10U; ++k) {
        out = alg.update(k * static_cast<uint64_t>(1e7), accel);
    }

    for (int i = 0; i < 3; ++i) {
        EXPECT_TRUE(std::isfinite(out.vehAccumDV_B[i]));
    }
    EXPECT_TRUE(std::isfinite(out.timeTag));
}
