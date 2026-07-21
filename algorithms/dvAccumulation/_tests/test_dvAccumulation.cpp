#include "dvAccumulation/dvAccumulationAlgorithm.h"
#include "dvAccumulationTestHelpers.hpp"
#include "utilities/fsw/timeConstants.h"

#include <gtest/gtest.h>
#include <Eigen/Core>
#include <array>
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

TEST(DvAccumulationTest, KnownAccelerationProducesExpectedDeltaV) {
    /*! - checks the integration against a hand-computed expected value. The first call only sets the
     *    time reference; the second, 0.5 s later, integrates dt * accel = 0.5 * [2, -4, 0] = [1, -2, 0]. */
    DvAccumulationAlgorithm alg{};
    alg.reInitialize();

    const uint64_t t0 = static_cast<uint64_t>(1e9);                                       // 1.0 s
    const uint64_t t1 = static_cast<uint64_t>(15e8);                                      // 1.5 s  (dt = 0.5 s)
    alg.update(t0, Eigen::Vector3f::Zero());                                              // sets the time reference
    const DvAccumulationOutput out = alg.update(t1, Eigen::Vector3f{2.0F, -4.0F, 0.0F});  // integrate dt * accel

    EXPECT_NEAR(out.vehAccumDV_B[0], 1.0F, 1e-5F);
    EXPECT_NEAR(out.vehAccumDV_B[1], -2.0F, 1e-5F);
    EXPECT_NEAR(out.vehAccumDV_B[2], 0.0F, 1e-5F);
    EXPECT_NEAR(out.timeTag, 1.5, 1e-9);
}

TEST(DvAccumulationTest, FirstCallIgnoresAcceleration) {
    /*! - the first update after reInitialize never integrates, whatever acceleration it carries: it only
     *    sets the time reference. Sweep extreme magnitudes; every one must yield zero accumulated DV. */
    const std::array<Eigen::Vector3f, 3> firstCallAccels{
        Eigen::Vector3f{1.0e6F, -1.0e6F, 1.0e6F}, Eigen::Vector3f{-1.0e9F, 1.0e9F, -1.0e9F}, Eigen::Vector3f::Zero()};

    for (const Eigen::Vector3f& accel : firstCallAccels) {
        DvAccumulationAlgorithm alg{};
        alg.reInitialize();
        const DvAccumulationOutput out = alg.update(static_cast<uint64_t>(5e7), accel);
        EXPECT_FLOAT_EQ(out.vehAccumDV_B[0], 0.0F);
        EXPECT_FLOAT_EQ(out.vehAccumDV_B[1], 0.0F);
        EXPECT_FLOAT_EQ(out.vehAccumDV_B[2], 0.0F);
    }
}

TEST(DvAccumulationTest, ReInitializeExceptPersistentStatesKeepsTimeReference) {
    /*! - reInitializeExceptPersistentStates zeros the accumulator but keeps previousTime, so the next
     *    update integrates immediately from the retained time (no re-set of the reference, no lost
     *    interval). The result [1,0,0] rules out both failure modes: [2,0,0] if the accumulator hadn't
     *    reset, [0,0,0] if the time reference had been dropped. */
    DvAccumulationAlgorithm alg{};
    alg.reInitialize();

    const Eigen::Vector3f accel{2.0F, 0.0F, 0.0F};
    alg.update(static_cast<uint64_t>(1e9), accel);  // callTime 1.0 s: sets ref
    const DvAccumulationOutput before =
        alg.update(static_cast<uint64_t>(15e8), accel);  // callTime 1.5 s: dt=0.5 -> [1,0,0]
    EXPECT_NEAR(before.vehAccumDV_B[0], 1.0F, 1e-5F);

    alg.reInitializeExceptPersistentStates();  // accumulator->0, previousTime kept

    const DvAccumulationOutput after =
        alg.update(static_cast<uint64_t>(2e9), accel);  // callTime 2.0 s: dt=0.5 from 1.5 s
    EXPECT_NEAR(after.vehAccumDV_B[0], 1.0F, 1e-5F);
    EXPECT_NEAR(after.vehAccumDV_B[1], 0.0F, 1e-5F);
    EXPECT_NEAR(after.vehAccumDV_B[2], 0.0F, 1e-5F);
    EXPECT_NEAR(after.timeTag, 2.0, 1e-9);
}

TEST(DvAccumulationTest, ReInitializeResetsTimeReference) {
    /*! - reInitialize (unlike reInitializeExceptPersistentStates) also resets previousTime, so the next
     *    update re-sets the time reference: zero DV, no integration, regardless of elapsed time. Contrast
     *    with ReInitializeExceptPersistentStatesKeepsTimeReference, whose next call yields [1,0,0]. */
    DvAccumulationAlgorithm alg{};
    alg.reInitialize();

    const Eigen::Vector3f accel{2.0F, 0.0F, 0.0F};
    alg.update(static_cast<uint64_t>(1e9), accel);   // callTime 1.0 s: sets ref
    alg.update(static_cast<uint64_t>(15e8), accel);  // callTime 1.5 s: dt=0.5 -> [1,0,0]

    alg.reInitialize();  // accumulator->0 AND previousTime->0

    const DvAccumulationOutput after = alg.update(static_cast<uint64_t>(2e9), accel);  // callTime 2.0 s: re-sets ref
    EXPECT_FLOAT_EQ(after.vehAccumDV_B[0], 0.0F);
    EXPECT_FLOAT_EQ(after.vehAccumDV_B[1], 0.0F);
    EXPECT_FLOAT_EQ(after.vehAccumDV_B[2], 0.0F);
    EXPECT_NEAR(after.timeTag, 2.0, 1e-9);  // reference re-set to the new callTime
}

TEST(DvAccumulationTest, BackwardCallTimeIsIgnored) {
    /*! - a callTime strictly earlier than previousTime (clock stepped backward / out-of-order) is
     *    dropped by the strictly-greater gate: accumulator and time reference stay unchanged. */
    DvAccumulationAlgorithm alg{};
    alg.reInitialize();

    const Eigen::Vector3f accel{2.0F, 0.0F, 0.0F};
    alg.update(static_cast<uint64_t>(1e9), accel);  // callTime 1.0 s: sets ref
    const DvAccumulationOutput forward =
        alg.update(static_cast<uint64_t>(2e9), accel);  // callTime 2.0 s: dt=1.0 -> [2,0,0]
    const DvAccumulationOutput backward =
        alg.update(static_cast<uint64_t>(15e8), accel);  // callTime 1.5 s < 2.0 s: ignored

    EXPECT_FLOAT_EQ(backward.vehAccumDV_B[0], forward.vehAccumDV_B[0]);  // accumulator unchanged
    EXPECT_FLOAT_EQ(backward.vehAccumDV_B[1], forward.vehAccumDV_B[1]);
    EXPECT_FLOAT_EQ(backward.vehAccumDV_B[2], forward.vehAccumDV_B[2]);
    EXPECT_DOUBLE_EQ(backward.timeTag, forward.timeTag);  // time reference unchanged (still 2.0 s)
}
