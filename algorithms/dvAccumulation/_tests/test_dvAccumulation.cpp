#include "dvAccumulation/dvAccumulationAlgorithm.h"
#include "dvAccumulationTestHelpers.hpp"

#include <gtest/gtest.h>
#include <Eigen/Core>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

TEST(DvAccumulationTest, SetupTest) {
    testDvAccumulationSetup();

    /*! - the configuration validator is the only limit on controlPeriod. Thus this test includes
     *    each rejected value and the accepted value */
    EXPECT_THROW(DvAccumulationConfig::create(0.0F), fsw::invalid_argument);
    EXPECT_THROW(DvAccumulationConfig::create(-0.2F), fsw::invalid_argument);
    EXPECT_THROW(DvAccumulationConfig::create(std::numeric_limits<float>::quiet_NaN()), fsw::invalid_argument);
    EXPECT_THROW(DvAccumulationConfig::create(std::numeric_limits<float>::infinity()), fsw::invalid_argument);
}

TEST(DvAccumulationTest, AccelBiasIsSubtractedNotAdded) {
    /*! - the test uses an acceleration of two times the bias. Thus the sign is clear: a subtraction
     *    gives b, an addition gives 3b, and no bias correction gives 2b. This test does not use the
     *    reference oracle for the expected value, because a hand-written oracle can contain the same
     *    sign error as the algorithm. The expected value is a hand calculation. */
    constexpr float kControlPeriod = 0.5F;
    const Eigen::Vector3f bias{0.02F, -0.05F, 0.01F};
    const Eigen::Vector3f accel = 2.0F * bias;

    DvAccumulationAlgorithm alg{DvAccumulationConfig::create(kControlPeriod)};

    constexpr int kTotalCalls = 5;
    Eigen::Vector3f out = Eigen::Vector3f::Zero();
    for (int k = 0; k < kTotalCalls; ++k) {
        out = alg.update(accel, bias);
    }

    /*! - N calls have N-1 intervals, so the expected Delta-V is bias * (N-1) * controlPeriod */
    const float elapsed = static_cast<float>(kTotalCalls - 1) * kControlPeriod;
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(out[i], bias[i] * elapsed, 1e-6F);
    }
}

TEST(DvAccumulationTest, BiasEqualToAccelerationYieldsZeroDeltaV) {
    /*! - if the bias is equal to the acceleration, the accumulated Delta-V must be zero: the module
     *    measures only bias and no thrust */
    const Eigen::Vector3f accel{0.004166F, -0.01F, 0.002F};
    DvAccumulationAlgorithm alg{DvAccumulationConfig::create(0.2F)};

    Eigen::Vector3f out = Eigen::Vector3f::Zero();
    for (int k = 0; k < 100; ++k) {
        out = alg.update(accel, accel);
    }

    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(out[i], 0.0F, 1e-6F);
    }
}

TEST(DvAccumulationTest, ReferenceTest) {
    /*! - a sequence that includes the first call that starts the window, and the integration in
     *    each later call. The test compares each step to the reference */
    testDvAccumulation(0.5F,
                       {
                           Eigen::Vector3f{0.1F, -0.2F, 0.3F},   // first call: starts the window
                           Eigen::Vector3f{0.2F, -0.1F, 0.4F},   // integrate
                           Eigen::Vector3f{-0.1F, 0.5F, 0.1F},   // integrate
                           Eigen::Vector3f{9.9F, 9.9F, 9.9F},    // integrate
                           Eigen::Vector3f{-0.05F, 0.0F, 0.1F},  // integrate
                       });
}

TEST(DvAccumulationTest, KnownAccelerationProducesExpectedDeltaV) {
    /*! - the test compares the integration to a hand calculation, then to N calls: the accumulated
     *    Delta-V must be equal to accel * elapsed time after the first call, and N calls have N-1
     *    intervals. The first call starts the window to make this true. If the interval count changes
     *    to N or to N-2, this test does not pass. */
    constexpr float kControlPeriod = 0.5F;
    DvAccumulationAlgorithm alg{DvAccumulationConfig::create(kControlPeriod)};

    const Eigen::Vector3f accel{2.0F, -4.0F, 0.0F};
    alg.update(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero());      // starts the window
    Eigen::Vector3f out = alg.update(accel, Eigen::Vector3f::Zero());  // integrate dt * accel = [1, -2, 0]

    EXPECT_NEAR(out[0], 1.0F, 1e-5F);
    EXPECT_NEAR(out[1], -2.0F, 1e-5F);
    EXPECT_NEAR(out[2], 0.0F, 1e-5F);

    /*! - continue to N total calls and compare to accel * elapsed time again */
    constexpr int kTotalCalls = 20;
    for (int k = 2; k < kTotalCalls; ++k) {
        out = alg.update(accel, Eigen::Vector3f::Zero());
    }
    const float elapsed = static_cast<float>(kTotalCalls - 1) * kControlPeriod;
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(out[i], accel[i] * elapsed, 1e-4F);
    }
}

TEST(DvAccumulationTest, FirstCallStartsTheAccumulationClock) {
    /*! - the first update() after construction starts the accumulation window and never integrates.
     *    This is true for each acceleration value. The test uses very large magnitudes. Each one must
     *    give a Delta-V of zero. */
    const std::array<Eigen::Vector3f, 3> firstCallAccels{
        Eigen::Vector3f{1.0e6F, -1.0e6F, 1.0e6F}, Eigen::Vector3f{-1.0e9F, 1.0e9F, -1.0e9F}, Eigen::Vector3f::Zero()};

    for (const Eigen::Vector3f& accel : firstCallAccels) {
        DvAccumulationAlgorithm alg{DvAccumulationConfig::create(0.2F)};
        const Eigen::Vector3f out = alg.update(accel, Eigen::Vector3f::Zero());
        EXPECT_FLOAT_EQ(out[0], 0.0F);
        EXPECT_FLOAT_EQ(out[1], 0.0F);
        EXPECT_FLOAT_EQ(out[2], 0.0F);
    }
}

TEST(DvAccumulationTest, ReInitializeRestartsTheAccumulationClock) {
    /*! - reInitialize() sets the accumulator to zero and also sets firstCall to true. Thus the next
     *    update() starts a new window: a Delta-V of zero and no integration. The [0,0,0] result
     *    detects the two failure modes: [1,0,0] if the window stays open, [2,0,0] if the accumulator
     *    keeps its value. But setConfig() installs the parameters only. It must not start a new
     *    window. */
    DvAccumulationAlgorithm alg{DvAccumulationConfig::create(0.5F)};

    const Eigen::Vector3f accel{2.0F, 0.0F, 0.0F};
    alg.update(accel, Eigen::Vector3f::Zero());  // starts the window
    alg.update(accel, Eigen::Vector3f::Zero());  // dt=0.5 -> [1,0,0]

    alg.reInitialize();  // accumulator->0 AND a new window

    const Eigen::Vector3f after = alg.update(accel, Eigen::Vector3f::Zero());
    EXPECT_FLOAT_EQ(after[0], 0.0F);
    EXPECT_FLOAT_EQ(after[1], 0.0F);
    EXPECT_FLOAT_EQ(after[2], 0.0F);

    /*! - setConfig() changes the step and does not start a new window. Thus the next call
     *    integrates at the new period */
    alg.setConfig(DvAccumulationConfig::create(1.0F));
    const Eigen::Vector3f afterSetConfig = alg.update(accel, Eigen::Vector3f::Zero());
    EXPECT_NEAR(afterSetConfig[0], 2.0F, 1e-5F);  // 1.0 s * 2.0 m/s^2, not zero
}
