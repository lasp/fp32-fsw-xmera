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

    /*! - the configuration validator is the only guard on controlPeriod, so every rejected value is
     *    checked here alongside the accepted one */
    EXPECT_THROW(DvAccumulationConfig::create(0.0F), fsw::invalid_argument);
    EXPECT_THROW(DvAccumulationConfig::create(-0.2F), fsw::invalid_argument);
    EXPECT_THROW(DvAccumulationConfig::create(std::numeric_limits<float>::quiet_NaN()), fsw::invalid_argument);
    EXPECT_THROW(DvAccumulationConfig::create(std::numeric_limits<float>::infinity()), fsw::invalid_argument);
}

TEST(DvAccumulationTest, AccelBiasIsSubtractedNotAdded) {
    /*! - drives an acceleration of exactly twice the bias so the sign is unambiguous: subtracting
     *    leaves b, adding would leave 3b, and ignoring the bias entirely would leave 2b. The expected
     *    value is hand-computed rather than taken from the reference oracle, since a hand-written
     *    oracle can encode the same sign error as the implementation. */
    constexpr float kControlPeriod = 0.5F;
    const Eigen::Vector3f bias{0.02F, -0.05F, 0.01F};
    const Eigen::Vector3f accel = 2.0F * bias;

    DvAccumulationAlgorithm alg{DvAccumulationConfig::create(kControlPeriod)};

    constexpr int kTotalCalls = 5;
    Eigen::Vector3f out = Eigen::Vector3f::Zero();
    for (int k = 0; k < kTotalCalls; ++k) {
        out = alg.update(accel, bias);
    }

    /*! - N calls bound N-1 intervals, so the expected Delta-V is bias * (N-1) * controlPeriod */
    const float elapsed = static_cast<float>(kTotalCalls - 1) * kControlPeriod;
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(out[i], bias[i] * elapsed, 1e-6F);
    }
}

TEST(DvAccumulationTest, BiasEqualToAccelerationYieldsZeroDeltaV) {
    /*! - a bias equal to the acceleration must zero the accumulated Delta-V outright: the module is
     *    measuring pure bias and no real thrust */
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
    /*! - a sequence exercising the window-starting first call and integration over later calls,
     *    compared to the reference at every step */
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
    /*! - checks the integration against a hand-computed expected value, then generalizes to N ticks:
     *    the accumulated Delta-V must equal accel * elapsed time since the first call, where N
     *    samples bound N-1 intervals. That invariant is what the window-starting first call exists to
     *    satisfy, and it fails if the interval count drifts to N or N-2. */
    constexpr float kControlPeriod = 0.5F;
    DvAccumulationAlgorithm alg{DvAccumulationConfig::create(kControlPeriod)};

    const Eigen::Vector3f accel{2.0F, -4.0F, 0.0F};
    alg.update(Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero());      // starts the window
    Eigen::Vector3f out = alg.update(accel, Eigen::Vector3f::Zero());  // integrate dt * accel = [1, -2, 0]

    EXPECT_NEAR(out[0], 1.0F, 1e-5F);
    EXPECT_NEAR(out[1], -2.0F, 1e-5F);
    EXPECT_NEAR(out[2], 0.0F, 1e-5F);

    /*! - continue to N total calls and re-check against accel * elapsed */
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
    /*! - the first update after construction never integrates, whatever acceleration it carries: it
     *    starts the accumulation window. Sweep extreme magnitudes; every one must yield zero DV. */
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
    /*! - reInitialize re-arms firstCall as well as zeroing the accumulator, so the next update
     *    restarts the window: zero DV, no integration. The [0,0,0] result rules out both failure
     *    modes: [1,0,0] if the window had been left open, [2,0,0] if the accumulator hadn't reset.
     *    setConfig, by contrast, installs parameters only and must not re-arm the window. */
    DvAccumulationAlgorithm alg{DvAccumulationConfig::create(0.5F)};

    const Eigen::Vector3f accel{2.0F, 0.0F, 0.0F};
    alg.update(accel, Eigen::Vector3f::Zero());  // starts the window
    alg.update(accel, Eigen::Vector3f::Zero());  // dt=0.5 -> [1,0,0]

    alg.reInitialize();  // accumulator->0 AND window restarted

    const Eigen::Vector3f after = alg.update(accel, Eigen::Vector3f::Zero());
    EXPECT_FLOAT_EQ(after[0], 0.0F);
    EXPECT_FLOAT_EQ(after[1], 0.0F);
    EXPECT_FLOAT_EQ(after[2], 0.0F);

    /*! - setConfig swaps the step without re-arming the window, so the very next call integrates at
     *    the new period rather than starting a fresh one */
    alg.setConfig(DvAccumulationConfig::create(1.0F));
    const Eigen::Vector3f afterSetConfig = alg.update(accel, Eigen::Vector3f::Zero());
    EXPECT_NEAR(afterSetConfig[0], 2.0F, 1e-5F);  // 1.0 s * 2.0 m/s^2, not zero
}
