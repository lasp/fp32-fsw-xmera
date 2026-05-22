#include "dvAccumulation/dvAccumulationAlgorithm.h"
#include "dvAccumulationTestHelpers.hpp"
#include "msgPayloadDef/AccDataMsgF32Payload.h"

#include <gtest/gtest.h>
#include <Eigen/Core>
#include <cstdint>
#include <vector>

TEST(DvAccumulationTest, SetupTest) { testDvAccumulationSetup(); }

TEST(DvAccumulationTest, ReferenceTest) {
    /*! - reset snapshot: three packets, latest at t = 5e7 ns */
    const AccDataMsgF32Payload resetSnap = buildAccData(
        {static_cast<uint64_t>(1e7), static_cast<uint64_t>(3e7), static_cast<uint64_t>(5e7)},
        {Eigen::Vector3f{0.0F, 0.0F, 0.0F}, Eigen::Vector3f{0.0F, 0.0F, 0.0F}, Eigen::Vector3f{0.0F, 0.0F, 0.0F}});

    /*! - one update snapshot with measurements after the seed */
    const AccDataMsgF32Payload snap1 = buildAccData(
        {static_cast<uint64_t>(6e7), static_cast<uint64_t>(7e7), static_cast<uint64_t>(8e7)},
        {Eigen::Vector3f{0.1F, -0.2F, 0.3F}, Eigen::Vector3f{0.2F, -0.1F, 0.4F}, Eigen::Vector3f{-0.1F, 0.5F, 0.1F}});

    /*! - a second snapshot mixing already-seen and new packets */
    const AccDataMsgF32Payload snap2 = buildAccData(
        {static_cast<uint64_t>(7e7), static_cast<uint64_t>(9e7), static_cast<uint64_t>(1.1e8)},
        {Eigen::Vector3f{0.0F, 0.0F, 0.0F}, Eigen::Vector3f{0.05F, 0.1F, -0.15F}, Eigen::Vector3f{-0.05F, 0.0F, 0.1F}});

    testDvAccumulation({snap1, snap2}, resetSnap);
}

TEST(DvAccumulationTest, EmptyBufferProducesZero) {
    /*! - all-zero measTimes mean no packets are valid */
    AccDataMsgF32Payload empty{};
    DvAccumulationAlgorithm alg(DvAccumulationConfig::create());
    alg.resetState(empty);

    const DvAccumulationOutput out = alg.update(empty);
    EXPECT_FLOAT_EQ(out.vehAccumDV_B[0], 0.0F);
    EXPECT_FLOAT_EQ(out.vehAccumDV_B[1], 0.0F);
    EXPECT_FLOAT_EQ(out.vehAccumDV_B[2], 0.0F);
    EXPECT_DOUBLE_EQ(out.timeTag, 0.0);
}

TEST(DvAccumulationTest, ResetSeedsPreviousTime) {
    /*! - reset with a buffer whose latest measTime is 1e8 — subsequent update with a packet at
     *    exactly that time should NOT integrate (strict greater-than gate) */
    const AccDataMsgF32Payload resetSnap =
        buildAccData({static_cast<uint64_t>(1e8)}, {Eigen::Vector3f{0.0F, 0.0F, 0.0F}});

    DvAccumulationAlgorithm alg(DvAccumulationConfig::create());
    alg.resetState(resetSnap);

    const AccDataMsgF32Payload noNew = buildAccData({static_cast<uint64_t>(1e8)}, {Eigen::Vector3f{1.0F, 2.0F, 3.0F}});
    const DvAccumulationOutput out = alg.update(noNew);

    EXPECT_FLOAT_EQ(out.vehAccumDV_B[0], 0.0F);
    EXPECT_FLOAT_EQ(out.vehAccumDV_B[1], 0.0F);
    EXPECT_FLOAT_EQ(out.vehAccumDV_B[2], 0.0F);
}

TEST(DvAccumulationTest, OutOfOrderInputStillSortsCorrectly) {
    /*! - packets arrive shuffled in the buffer; the algorithm sorts internally */
    const AccDataMsgF32Payload resetSnap = buildAccData({}, {});  // empty seed → previousTime = 0
    const AccDataMsgF32Payload shuffled = buildAccData(
        {static_cast<uint64_t>(3e7), static_cast<uint64_t>(1e7), static_cast<uint64_t>(2e7)},
        {Eigen::Vector3f{0.3F, 0.0F, 0.0F}, Eigen::Vector3f{0.1F, 0.0F, 0.0F}, Eigen::Vector3f{0.2F, 0.0F, 0.0F}});

    testDvAccumulation({shuffled}, resetSnap);
}

TEST(DvAccumulationTest, RepeatedIdenticalInputsDoNotDoubleAccumulate) {
    /*! - feeding the same snapshot twice should integrate only the first time */
    const AccDataMsgF32Payload resetSnap = buildAccData({}, {});
    const AccDataMsgF32Payload snap = buildAccData(
        {static_cast<uint64_t>(1e7), static_cast<uint64_t>(2e7), static_cast<uint64_t>(3e7)},
        {Eigen::Vector3f{0.1F, 0.2F, 0.3F}, Eigen::Vector3f{0.1F, 0.2F, 0.3F}, Eigen::Vector3f{0.1F, 0.2F, 0.3F}});

    DvAccumulationAlgorithm alg(DvAccumulationConfig::create());
    alg.resetState(resetSnap);

    const DvAccumulationOutput first = alg.update(snap);
    const DvAccumulationOutput second = alg.update(snap);

    EXPECT_FLOAT_EQ(first.vehAccumDV_B[0], second.vehAccumDV_B[0]);
    EXPECT_FLOAT_EQ(first.vehAccumDV_B[1], second.vehAccumDV_B[1]);
    EXPECT_FLOAT_EQ(first.vehAccumDV_B[2], second.vehAccumDV_B[2]);
    EXPECT_DOUBLE_EQ(first.timeTag, second.timeTag);
}

TEST(DvAccumulationTest, AllOlderPacketsDoNotChangeState) {
    /*! - after a seed reset, an update whose packets are all older than the seed must not
     *    integrate anything */
    const AccDataMsgF32Payload resetSnap =
        buildAccData({static_cast<uint64_t>(1e8)}, {Eigen::Vector3f{0.0F, 0.0F, 0.0F}});

    DvAccumulationAlgorithm alg(DvAccumulationConfig::create());
    alg.resetState(resetSnap);

    const AccDataMsgF32Payload allOlder = buildAccData(
        {static_cast<uint64_t>(1e7), static_cast<uint64_t>(5e7), static_cast<uint64_t>(9e7)},
        {Eigen::Vector3f{1.0F, 2.0F, 3.0F}, Eigen::Vector3f{1.0F, 2.0F, 3.0F}, Eigen::Vector3f{1.0F, 2.0F, 3.0F}});
    const DvAccumulationOutput out = alg.update(allOlder);

    EXPECT_FLOAT_EQ(out.vehAccumDV_B[0], 0.0F);
    EXPECT_FLOAT_EQ(out.vehAccumDV_B[1], 0.0F);
    EXPECT_FLOAT_EQ(out.vehAccumDV_B[2], 0.0F);
}

TEST(DvAccumulationTest, BoundedInputProducesFiniteOutput) {
    /*! - with bounded accels and a bounded measTime span, the accumulator must stay finite */
    const AccDataMsgF32Payload resetSnap = buildAccData({}, {});

    DvAccumulationAlgorithm alg(DvAccumulationConfig::create());
    alg.resetState(resetSnap);

    /*! - 10 packets spanning 100 ms with a 10 m/s^2 accel — within range of any realistic sensor */
    std::vector<uint64_t> measTimes;
    std::vector<Eigen::Vector3f> accels;
    for (uint64_t k = 1U; k <= 10U; ++k) {
        measTimes.push_back(k * static_cast<uint64_t>(1e7));
        accels.emplace_back(10.0F, -10.0F, 5.0F);
    }
    const AccDataMsgF32Payload snap = buildAccData(measTimes, accels);

    const DvAccumulationOutput out = alg.update(snap);
    for (int i = 0; i < 3; ++i) {
        EXPECT_TRUE(std::isfinite(out.vehAccumDV_B[i]));
    }
    EXPECT_TRUE(std::isfinite(out.timeTag));
}

TEST(DvAccumulationTest, SingleNewPacketBootstrapSkipsFirst) {
    /*! - after a zero-seed reset, the first update's first packet > 0 is consumed by the
     *    dvInitialized bootstrap (sets previousTime, no integration); subsequent packets
     *    integrate normally. Verify a single-packet snapshot leaves accumulator at zero. */
    const AccDataMsgF32Payload resetSnap = buildAccData({}, {});

    DvAccumulationAlgorithm alg(DvAccumulationConfig::create());
    alg.resetState(resetSnap);

    const AccDataMsgF32Payload singlePacket =
        buildAccData({static_cast<uint64_t>(5e7)}, {Eigen::Vector3f{1.0F, 2.0F, 3.0F}});
    const DvAccumulationOutput out = alg.update(singlePacket);

    EXPECT_FLOAT_EQ(out.vehAccumDV_B[0], 0.0F);
    EXPECT_FLOAT_EQ(out.vehAccumDV_B[1], 0.0F);
    EXPECT_FLOAT_EQ(out.vehAccumDV_B[2], 0.0F);
    EXPECT_NEAR(out.timeTag, 5e7 * kNano2Sec, 1e-9);
}

TEST(DvAccumulationTest, SetConfigDoesNotResetState) {
    /*! - setConfig is meaningful only by shape; for empty Config it should not perturb
     *    the running accumulator. */
    DvAccumulationAlgorithm alg(DvAccumulationConfig::create());
    const AccDataMsgF32Payload resetSnap = buildAccData({}, {});
    alg.resetState(resetSnap);

    const AccDataMsgF32Payload snap =
        buildAccData({static_cast<uint64_t>(1e7), static_cast<uint64_t>(2e7)},
                     {Eigen::Vector3f{0.5F, 0.0F, 0.0F}, Eigen::Vector3f{0.5F, 0.0F, 0.0F}});
    const DvAccumulationOutput before = alg.update(snap);

    alg.setConfig(DvAccumulationConfig::create());

    /*! - feeding the same snap again won't integrate (no packets > previousTime), so the
     *    accumulator is unchanged */
    const DvAccumulationOutput after = alg.update(snap);
    EXPECT_FLOAT_EQ(before.vehAccumDV_B[0], after.vehAccumDV_B[0]);
    EXPECT_FLOAT_EQ(before.vehAccumDV_B[1], after.vehAccumDV_B[1]);
    EXPECT_FLOAT_EQ(before.vehAccumDV_B[2], after.vehAccumDV_B[2]);
}
