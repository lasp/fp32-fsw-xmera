#include "momentumManagementTestHelpers.hpp"

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace {

// Wheel speeds and gains below are kept to physically realizable magnitudes (cluster momenta of tens of Nms,
// dumping torques of order 1 Nm), which keeps the fp32 error near 1e-7 and this bound comfortably tight.
constexpr float kAccuracy = 1e-5F;

// Nominal dumping threshold for the standard four-wheel pyramid with Js = 0.1 and speeds
// (10, -25, 50, 100) r/s, which puts the cluster momentum well above the threshold.
constexpr float kNominalHsMin = 100.0F / 6000.0F * 100.0F;

// [1/s] nominal feedback gain. Sized so a cluster momentum of order 10 Nms is dumped over a few hundred
// seconds, i.e. a torque of order 0.5 Nm. Deliberately not 1, so a dropped gain cannot pass unnoticed.
constexpr float kNominalK = 0.05F;

// A threshold above the cluster momentum, used to park the module inside its deadband.
constexpr float kHighHsMin = 1000.0F / 6000.0F * 100.0F;

// [s] integration step; matches the task rate used by the python integration test.
constexpr float kControlPeriod = 0.5F;

// [1/s2] nominal integral gain. Over the ~20 cycles exercised here it contributes a torque comparable to the
// proportional term, so the integral path is clearly visible without leaving physical magnitudes.
constexpr float kNominalKi = 0.01F;

// [Nms2] anti-windup clamps: one far above anything the tests accumulate, one tight enough to suppress the
// integral term almost entirely.
constexpr float kLargeIntegralLimit = 1000.0F;
constexpr float kTightIntegralLimit = 5.0F;

// The integral is switched off by default so the proportional-law expectations below stand on their own.
MomentumManagementControlParameters nominalParams(float hsMin = kNominalHsMin,
                                                  float K = kNominalK,
                                                  float Ki = 0.0F,
                                                  float integralLimit = kLargeIntegralLimit,
                                                  float controlPeriod = kControlPeriod) {
    return {.hsMin = hsMin, .K = K, .Ki = Ki, .integralLimit = integralLimit, .controlPeriod = controlPeriod};
}

}  // namespace

// A threshold above the cluster momentum yields a zero torque.
TEST(MomentumManagement, NoDumpWhenBelowThreshold) {
    MomentumManagementAlgorithm alg{
        MomentumManagementConfig::create(nominalParams(kHighHsMin), makeStandardRwArrayConfig())};

    const auto Lr_B = alg.update(makeWheelSpeeds({10.0F, -25.0F, 50.0F, 100.0F}));

    EXPECT_TRUE(Lr_B.isZero(kAccuracy));
}

TEST(MomentumManagement, MatchesReferenceAcrossCases) {
    const auto rwArrayConfig = makeStandardRwArrayConfig();
    const auto nominalSpeeds = makeWheelSpeeds({10.0F, -25.0F, 50.0F, 100.0F});

    regressionTestMomentumManagement(rwArrayConfig, nominalSpeeds, nominalParams(), kAccuracy);
    regressionTestMomentumManagement(
        rwArrayConfig, makeWheelSpeeds({0.0F, 0.0F, 0.0F, 0.0F}), nominalParams(), kAccuracy);
    regressionTestMomentumManagement(
        rwArrayConfig, makeWheelSpeeds({-150.0F, 120.0F, -90.0F, 60.0F}), nominalParams(1.0F), kAccuracy);
    regressionTestMomentumManagement(
        rwArrayConfig, makeWheelSpeeds({1.0F, 1.0F, 1.0F, 1.0F}), nominalParams(0.0F), kAccuracy);
    // Gains far from the nominal one must carry through unchanged.
    regressionTestMomentumManagement(rwArrayConfig, nominalSpeeds, nominalParams(1.0F, 0.005F), kAccuracy);
    regressionTestMomentumManagement(rwArrayConfig, nominalSpeeds, nominalParams(1.0F, 1e-9F), kAccuracy);
    // The integral path accumulated over many cycles, unclamped and clamped.
    regressionTestMomentumManagement(
        rwArrayConfig, nominalSpeeds, nominalParams(kNominalHsMin, kNominalK, kNominalKi), kAccuracy, 20U);
    regressionTestMomentumManagement(rwArrayConfig,
                                     nominalSpeeds,
                                     nominalParams(kNominalHsMin, kNominalK, kNominalKi, kTightIntegralLimit),
                                     kAccuracy,
                                     20U);
}

// A single wheel spinning about a body axis is dumped straight back along that axis.
TEST(MomentumManagement, SingleWheelDumpsAlongItsSpinAxis) {
    const auto rwArrayConfig = makeRwArrayConfig({{0.0F, 0.0F, 1.0F}}, 0.2F);
    MomentumManagementAlgorithm alg{MomentumManagementConfig::create(nominalParams(1.0F), rwArrayConfig)};

    // hs = 0.2 * 50 = 10 Nms about +z; the excess above hsMin = 1 is 9 Nms, so the torque is -K * 9.
    const auto Lr_B = alg.update(makeWheelSpeeds({50.0F}));

    EXPECT_NEAR(Lr_B[0], 0.0F, kAccuracy);
    EXPECT_NEAR(Lr_B[1], 0.0F, kAccuracy);
    EXPECT_NEAR(Lr_B[2], -kNominalK * 9.0F, kAccuracy);
}

// The torque opposes the stored momentum and acts on exactly the momentum held above the threshold.
TEST(MomentumManagement, TorqueMagnitudeIsGainTimesExcessMomentum) {
    const auto rwArrayConfig = makeStandardRwArrayConfig();
    const auto wheelSpeeds = makeWheelSpeeds({10.0F, -25.0F, 50.0F, 100.0F});

    MomentumManagementAlgorithm alg{MomentumManagementConfig::create(nominalParams(), rwArrayConfig)};
    const auto Lr_B = alg.update(wheelSpeeds);

    const Eigen::Vector3f hs_B = clusterMomentum(rwArrayConfig, wheelSpeeds);

    EXPECT_NEAR(Lr_B.norm(), kNominalK * (hs_B.norm() - kNominalHsMin), kAccuracy);
    EXPECT_LT(Lr_B.normalized().dot(hs_B.normalized()), -1.0F + kAccuracy);
}

// The torque scales linearly with the gain: doubling K doubles the request.
TEST(MomentumManagement, TorqueScalesLinearlyWithGain) {
    const auto rwArrayConfig = makeStandardRwArrayConfig();
    const auto wheelSpeeds = makeWheelSpeeds({10.0F, -25.0F, 50.0F, 100.0F});

    MomentumManagementAlgorithm alg1{
        MomentumManagementConfig::create(nominalParams(kNominalHsMin, 0.05F), rwArrayConfig)};
    MomentumManagementAlgorithm alg2{
        MomentumManagementConfig::create(nominalParams(kNominalHsMin, 0.10F), rwArrayConfig)};

    const Eigen::Vector3f single = alg1.update(wheelSpeeds);
    const Eigen::Vector3f doubled = alg2.update(wheelSpeeds);

    for (Eigen::Index i = 0; i < 3; ++i) {
        EXPECT_NEAR(doubled[i], 2.0F * single[i], kAccuracy) << "component " << i;
    }
}

// The request is recomputed on every call rather than latched to the first one: repeated updates with the
// same wheel speeds all report the same non-zero request.
TEST(MomentumManagement, RequestIsRecomputedEveryUpdate) {
    MomentumManagementAlgorithm alg{MomentumManagementConfig::create(nominalParams(), makeStandardRwArrayConfig())};
    const auto wheelSpeeds = makeWheelSpeeds({10.0F, -25.0F, 50.0F, 100.0F});

    const Eigen::Vector3f first = alg.update(wheelSpeeds);
    ASSERT_FALSE(first.isZero(kAccuracy));

    for (int cycle = 0; cycle < 4; ++cycle) {
        EXPECT_TRUE(alg.update(wheelSpeeds).isApprox(first)) << "cycle " << cycle;
    }
}

// setConfig replaces the configuration, and the very next update uses the new parameters.
TEST(MomentumManagement, SetConfigTakesEffectOnNextUpdate) {
    const auto rwArrayConfig = makeStandardRwArrayConfig();
    MomentumManagementAlgorithm alg{MomentumManagementConfig::create(nominalParams(), rwArrayConfig)};
    const auto wheelSpeeds = makeWheelSpeeds({10.0F, -25.0F, 50.0F, 100.0F});

    EXPECT_FALSE(alg.update(wheelSpeeds).isZero(kAccuracy));

    // A threshold above the cluster momentum switches dumping off from the next update onwards.
    alg.setConfig(MomentumManagementConfig::create(nominalParams(kHighHsMin), rwArrayConfig));
    EXPECT_TRUE(alg.update(wheelSpeeds).isZero(kAccuracy));
}

TEST(MomentumManagement, ConfigRoundTrips) {
    testMomentumManagementSetup(nominalParams(), makeStandardRwArrayConfig(), kAccuracy);
    testMomentumManagementSetup(nominalParams(0.0F, 0.05F), makeRwArrayConfig({{0.0F, 0.0F, 1.0F}}, 0.05F), kAccuracy);
    // Non-unit spin axes are normalized on construction rather than rejected outright at this scale.
    testMomentumManagementSetup(nominalParams(2.5F), makeRwArrayConfig({{3.0F, 4.0F, 0.0F}}, 0.1F), kAccuracy);
}

// ---------------------------------------------------------------------------------------------------
// Integral path
// ---------------------------------------------------------------------------------------------------

// A sustained excess momentum accumulates, so the request grows on every cycle.
TEST(MomentumManagement, IntegralAccumulatesAcrossCycles) {
    MomentumManagementAlgorithm alg{MomentumManagementConfig::create(
        nominalParams(kNominalHsMin, kNominalK, kNominalKi), makeStandardRwArrayConfig())};
    const auto wheelSpeeds = makeWheelSpeeds({10.0F, -25.0F, 50.0F, 100.0F});

    float previous = 0.0F;
    for (uint32_t cycle = 0U; cycle < 10U; ++cycle) {
        const float magnitude = alg.update(wheelSpeeds).norm();
        EXPECT_GT(magnitude, previous) << "cycle " << cycle;
        previous = magnitude;
    }
}

// With Ki = 0 the request is unchanged cycle after cycle; with Ki > 0 the same input grows.
TEST(MomentumManagement, ZeroKiDisablesTheIntegral) {
    const auto rwArrayConfig = makeStandardRwArrayConfig();
    const auto wheelSpeeds = makeWheelSpeeds({10.0F, -25.0F, 50.0F, 100.0F});

    MomentumManagementAlgorithm withoutIntegral{MomentumManagementConfig::create(nominalParams(), rwArrayConfig)};
    MomentumManagementAlgorithm withIntegral{
        MomentumManagementConfig::create(nominalParams(kNominalHsMin, kNominalK, kNominalKi), rwArrayConfig)};

    const Eigen::Vector3f firstWithout = withoutIntegral.update(wheelSpeeds);
    const Eigen::Vector3f firstWith = withIntegral.update(wheelSpeeds);

    Eigen::Vector3f lastWithout = firstWithout;
    Eigen::Vector3f lastWith = firstWith;
    for (uint32_t cycle = 0U; cycle < 10U; ++cycle) {
        lastWithout = withoutIntegral.update(wheelSpeeds);
        lastWith = withIntegral.update(wheelSpeeds);
    }

    EXPECT_TRUE(lastWithout.isApprox(firstWithout));
    EXPECT_GT(lastWith.norm(), firstWith.norm());
}

// The anti-windup clamp bounds how far the integral term can move the request, however long the momentum is
// held: each component is capped at integralLimit, so the shift cannot exceed sqrt(3) * Ki * integralLimit.
TEST(MomentumManagement, IntegralLimitBoundsTheIntegralTerm) {
    const auto rwArrayConfig = makeStandardRwArrayConfig();
    const auto wheelSpeeds = makeWheelSpeeds({10.0F, -25.0F, 50.0F, 100.0F});

    MomentumManagementAlgorithm proportionalOnly{MomentumManagementConfig::create(nominalParams(), rwArrayConfig)};
    const Eigen::Vector3f LrProportional = proportionalOnly.update(wheelSpeeds);

    MomentumManagementAlgorithm clamped{MomentumManagementConfig::create(
        nominalParams(kNominalHsMin, kNominalK, kNominalKi, kTightIntegralLimit), rwArrayConfig)};
    MomentumManagementAlgorithm unlimited{MomentumManagementConfig::create(
        nominalParams(kNominalHsMin, kNominalK, kNominalKi, kLargeIntegralLimit), rwArrayConfig)};

    Eigen::Vector3f LrClamped = Eigen::Vector3f::Zero();
    Eigen::Vector3f LrUnlimited = Eigen::Vector3f::Zero();
    for (uint32_t cycle = 0U; cycle < 20U; ++cycle) {
        LrClamped = clamped.update(wheelSpeeds);
        LrUnlimited = unlimited.update(wheelSpeeds);
    }

    const float clampedBound = std::sqrt(3.0F) * kNominalKi * kTightIntegralLimit;
    EXPECT_LE((LrClamped - LrProportional).norm(), clampedBound + kAccuracy);

    // The same sustained momentum drives an effectively unlimited integral far past that bound.
    EXPECT_GT((LrUnlimited - LrProportional).norm(), 10.0F * clampedBound);
}

// reInitialize() re-seeds the integrator, so the next update reproduces the very first request.
TEST(MomentumManagement, ReInitializeClearsTheIntegral) {
    MomentumManagementAlgorithm alg{MomentumManagementConfig::create(
        nominalParams(kNominalHsMin, kNominalK, kNominalKi), makeStandardRwArrayConfig())};
    const auto wheelSpeeds = makeWheelSpeeds({10.0F, -25.0F, 50.0F, 100.0F});

    const Eigen::Vector3f first = alg.update(wheelSpeeds);
    for (uint32_t cycle = 0U; cycle < 10U; ++cycle) {
        (void)alg.update(wheelSpeeds);
    }
    ASSERT_FALSE(alg.update(wheelSpeeds).isApprox(first));

    alg.reInitialize();
    EXPECT_TRUE(alg.update(wheelSpeeds).isApprox(first));
}

// setConfig installs new parameters without disturbing the integrator, so accumulation continues.
TEST(MomentumManagement, SetConfigPreservesIntegratorState) {
    const auto rwArrayConfig = makeStandardRwArrayConfig();
    const auto params = nominalParams(kNominalHsMin, kNominalK, kNominalKi);
    MomentumManagementAlgorithm alg{MomentumManagementConfig::create(params, rwArrayConfig)};
    const auto wheelSpeeds = makeWheelSpeeds({10.0F, -25.0F, 50.0F, 100.0F});

    const Eigen::Vector3f first = alg.update(wheelSpeeds);
    Eigen::Vector3f accumulated = first;
    for (uint32_t cycle = 0U; cycle < 10U; ++cycle) {
        accumulated = alg.update(wheelSpeeds);
    }

    alg.setConfig(MomentumManagementConfig::create(params, rwArrayConfig));
    const Eigen::Vector3f afterSetConfig = alg.update(wheelSpeeds);

    EXPECT_GT(afterSetConfig.norm(), accumulated.norm());
    EXPECT_FALSE(afterSetConfig.isApprox(first));
}

// ---------------------------------------------------------------------------------------------------
// Config validation
// ---------------------------------------------------------------------------------------------------

TEST(MomentumManagementConfigValidation, RejectsInvalidHsMin) {
    const auto rwArrayConfig = makeStandardRwArrayConfig();

    EXPECT_FALSE(MomentumManagementConfig::isValidHsMin(-1.0F));
    EXPECT_FALSE(MomentumManagementConfig::isValidHsMin(std::numeric_limits<float>::quiet_NaN()));
    EXPECT_FALSE(MomentumManagementConfig::isValidHsMin(std::numeric_limits<float>::infinity()));

    EXPECT_THROW((void)MomentumManagementConfig::create(nominalParams(-1.0F), rwArrayConfig), fsw::invalid_argument);
    EXPECT_THROW(
        (void)MomentumManagementConfig::create(nominalParams(std::numeric_limits<float>::quiet_NaN()), rwArrayConfig),
        fsw::invalid_argument);
    EXPECT_THROW(
        (void)MomentumManagementConfig::create(nominalParams(std::numeric_limits<float>::infinity()), rwArrayConfig),
        fsw::invalid_argument);
}

// A zero threshold means "dump all momentum" and is a legitimate setting.
TEST(MomentumManagementConfigValidation, AcceptsZeroHsMin) {
    EXPECT_TRUE(MomentumManagementConfig::isValidHsMin(0.0F));
    EXPECT_NO_THROW((void)MomentumManagementConfig::create(nominalParams(0.0F), makeStandardRwArrayConfig()));
}

// The gain must be strictly positive: zero would disable dumping entirely and a negative gain would drive the
// wheels away from the threshold instead of towards it.
TEST(MomentumManagementConfigValidation, RejectsInvalidK) {
    const auto rwArrayConfig = makeStandardRwArrayConfig();

    EXPECT_FALSE(MomentumManagementConfig::isValidK(0.0F));
    EXPECT_FALSE(MomentumManagementConfig::isValidK(-1.0F));
    EXPECT_FALSE(MomentumManagementConfig::isValidK(std::numeric_limits<float>::quiet_NaN()));
    EXPECT_FALSE(MomentumManagementConfig::isValidK(std::numeric_limits<float>::infinity()));

    EXPECT_THROW((void)MomentumManagementConfig::create(nominalParams(kNominalHsMin, 0.0F), rwArrayConfig),
                 fsw::invalid_argument);
    EXPECT_THROW((void)MomentumManagementConfig::create(nominalParams(kNominalHsMin, -1.0F), rwArrayConfig),
                 fsw::invalid_argument);
    EXPECT_THROW((void)MomentumManagementConfig::create(
                     nominalParams(kNominalHsMin, std::numeric_limits<float>::quiet_NaN()), rwArrayConfig),
                 fsw::invalid_argument);
}

TEST(MomentumManagementConfigValidation, AcceptsSmallPositiveK) {
    EXPECT_TRUE(MomentumManagementConfig::isValidK(1e-6F));
    EXPECT_NO_THROW(
        (void)MomentumManagementConfig::create(nominalParams(kNominalHsMin, 1e-6F), makeStandardRwArrayConfig()));
}

TEST(MomentumManagementConfigValidation, RejectsInvalidKi) {
    const auto rwArrayConfig = makeStandardRwArrayConfig();

    EXPECT_FALSE(MomentumManagementConfig::isValidKi(-1.0F));
    EXPECT_FALSE(MomentumManagementConfig::isValidKi(std::numeric_limits<float>::quiet_NaN()));
    EXPECT_FALSE(MomentumManagementConfig::isValidKi(std::numeric_limits<float>::infinity()));

    EXPECT_THROW((void)MomentumManagementConfig::create(nominalParams(kNominalHsMin, kNominalK, -1.0F), rwArrayConfig),
                 fsw::invalid_argument);
    EXPECT_THROW((void)MomentumManagementConfig::create(
                     nominalParams(kNominalHsMin, kNominalK, std::numeric_limits<float>::quiet_NaN()), rwArrayConfig),
                 fsw::invalid_argument);
}

// A zero integral gain switches the integral term off and is a legitimate setting.
TEST(MomentumManagementConfigValidation, AcceptsZeroKi) {
    EXPECT_TRUE(MomentumManagementConfig::isValidKi(0.0F));
    EXPECT_NO_THROW((void)MomentumManagementConfig::create(nominalParams(kNominalHsMin, kNominalK, 0.0F),
                                                           makeStandardRwArrayConfig()));
}

// The clamp must be positive whenever the integral is active: a zero limit with Ki > 0 would silently pin the
// integral term to zero, which is a misconfiguration rather than a way to disable it.
TEST(MomentumManagementConfigValidation, RejectsInvalidIntegralLimit) {
    const auto rwArrayConfig = makeStandardRwArrayConfig();

    EXPECT_FALSE(MomentumManagementConfig::isValidIntegralLimit(-1.0F, 0.0F));
    EXPECT_FALSE(MomentumManagementConfig::isValidIntegralLimit(std::numeric_limits<float>::quiet_NaN(), 0.0F));
    EXPECT_FALSE(MomentumManagementConfig::isValidIntegralLimit(0.0F, kNominalKi));

    EXPECT_THROW((void)MomentumManagementConfig::create(nominalParams(kNominalHsMin, kNominalK, kNominalKi, 0.0F),
                                                        rwArrayConfig),
                 fsw::invalid_argument);
    EXPECT_THROW((void)MomentumManagementConfig::create(nominalParams(kNominalHsMin, kNominalK, kNominalKi, -1.0F),
                                                        rwArrayConfig),
                 fsw::invalid_argument);
}

// A non-finite step is rejected even with the integral off: it would poison the integral state, and
// Ki * NaN is NaN even for Ki == 0, so the request would come out non-finite.
TEST(MomentumManagementConfigValidation, RejectsNonFiniteControlPeriodEvenWhenKiIsZero) {
    const MomentumManagementControlParameters params{.hsMin = kNominalHsMin,
                                                     .K = kNominalK,
                                                     .Ki = 0.0F,
                                                     .integralLimit = 0.0F,
                                                     .controlPeriod = std::numeric_limits<float>::quiet_NaN()};
    EXPECT_THROW((void)MomentumManagementConfig::create(params, makeStandardRwArrayConfig()), fsw::invalid_argument);
}

// With the integral switched off the clamp is unused, so a zero limit is acceptable.
TEST(MomentumManagementConfigValidation, AcceptsZeroIntegralLimitWhenKiIsZero) {
    EXPECT_TRUE(MomentumManagementConfig::isValidIntegralLimit(0.0F, 0.0F));
    EXPECT_NO_THROW((void)MomentumManagementConfig::create(nominalParams(kNominalHsMin, kNominalK, 0.0F, 0.0F),
                                                           makeStandardRwArrayConfig()));
}

TEST(MomentumManagementConfigValidation, RejectsInvalidControlPeriod) {
    const auto rwArrayConfig = makeStandardRwArrayConfig();

    // A zero step is only rejected while the integral is active.
    EXPECT_FALSE(MomentumManagementConfig::isValidControlPeriod(0.0F, kNominalKi));
    EXPECT_FALSE(MomentumManagementConfig::isValidControlPeriod(-1.0F, 0.0F));
    EXPECT_FALSE(MomentumManagementConfig::isValidControlPeriod(std::numeric_limits<float>::quiet_NaN(), 0.0F));
    EXPECT_FALSE(MomentumManagementConfig::isValidControlPeriod(std::numeric_limits<float>::infinity(), 0.0F));

    EXPECT_THROW((void)MomentumManagementConfig::create(
                     nominalParams(kNominalHsMin, kNominalK, kNominalKi, kLargeIntegralLimit, 0.0F), rwArrayConfig),
                 fsw::invalid_argument);
    EXPECT_THROW((void)MomentumManagementConfig::create(
                     nominalParams(kNominalHsMin, kNominalK, kNominalKi, kLargeIntegralLimit, -1.0F), rwArrayConfig),
                 fsw::invalid_argument);
}

// Only the integral term consumes the control period, so a purely proportional configuration needs nothing but
// hsMin and K: it may leave controlPeriod and integralLimit at their zero defaults.
TEST(MomentumManagementConfigValidation, AcceptsZeroControlPeriodWhenKiIsZero) {
    EXPECT_TRUE(MomentumManagementConfig::isValidControlPeriod(0.0F, 0.0F));

    const MomentumManagementControlParameters proportionalOnly{
        .hsMin = kNominalHsMin, .K = kNominalK, .Ki = 0.0F, .integralLimit = 0.0F, .controlPeriod = 0.0F};
    EXPECT_NO_THROW((void)MomentumManagementConfig::create(proportionalOnly, makeStandardRwArrayConfig()));

    // ...and the request it produces is the plain proportional one.
    MomentumManagementAlgorithm alg{MomentumManagementConfig::create(proportionalOnly, makeStandardRwArrayConfig())};
    const auto wheelSpeeds = makeWheelSpeeds({10.0F, -25.0F, 50.0F, 100.0F});
    const Eigen::Vector3f first = alg.update(wheelSpeeds);
    EXPECT_TRUE(alg.update(wheelSpeeds).isApprox(first));

    const float hs = clusterMomentum(makeStandardRwArrayConfig(), wheelSpeeds).norm();
    EXPECT_NEAR(first.norm(), kNominalK * (hs - kNominalHsMin), kAccuracy);
}

TEST(MomentumManagementConfigValidation, RejectsTooManyWheels) {
    auto rwArrayConfig = makeStandardRwArrayConfig();
    rwArrayConfig.numRW = kMaxNumRw + 1U;

    EXPECT_FALSE(MomentumManagementConfig::isValidRwArrayConfiguration(rwArrayConfig));
    EXPECT_THROW((void)MomentumManagementConfig::create(nominalParams(1.0F), rwArrayConfig), fsw::invalid_argument);
}

TEST(MomentumManagementConfigValidation, AcceptsExactlyMaxWheels) {
    MomentumManagementRwArrayConfiguration rwArrayConfig;
    rwArrayConfig.numRW = kMaxNumRw;
    for (uint32_t i = 0U; i < kMaxNumRw; ++i) {
        rwArrayConfig.GsMatrix_B.col(i) = Eigen::Vector3f::UnitZ();
        rwArrayConfig.JsList[i] = 0.1F;
    }

    EXPECT_TRUE(MomentumManagementConfig::isValidRwArrayConfiguration(rwArrayConfig));
    EXPECT_NO_THROW((void)MomentumManagementConfig::create(nominalParams(1.0F), rwArrayConfig));
}

TEST(MomentumManagementConfigValidation, RejectsNonUnitSpinAxis) {
    auto rwArrayConfig = makeStandardRwArrayConfig();
    rwArrayConfig.GsMatrix_B.col(1) = Eigen::Vector3f{0.0F, 2.0F, 0.0F};

    EXPECT_FALSE(MomentumManagementConfig::isValidRwArrayConfiguration(rwArrayConfig));
    EXPECT_THROW((void)MomentumManagementConfig::create(nominalParams(1.0F), rwArrayConfig), fsw::invalid_argument);
}

// A zero spin axis cannot be normalized and must be rejected rather than producing NaN downstream.
TEST(MomentumManagementConfigValidation, RejectsZeroSpinAxis) {
    auto rwArrayConfig = makeStandardRwArrayConfig();
    rwArrayConfig.GsMatrix_B.col(2) = Eigen::Vector3f::Zero();

    EXPECT_FALSE(MomentumManagementConfig::isValidRwArrayConfiguration(rwArrayConfig));
    EXPECT_THROW((void)MomentumManagementConfig::create(nominalParams(1.0F), rwArrayConfig), fsw::invalid_argument);
}

TEST(MomentumManagementConfigValidation, RejectsNonFiniteEntries) {
    {
        auto rwArrayConfig = makeStandardRwArrayConfig();
        rwArrayConfig.JsList[2] = std::numeric_limits<float>::quiet_NaN();
        EXPECT_FALSE(MomentumManagementConfig::isValidRwArrayConfiguration(rwArrayConfig));
    }
    {
        auto rwArrayConfig = makeStandardRwArrayConfig();
        rwArrayConfig.GsMatrix_B(0, 0) = std::numeric_limits<float>::infinity();
        EXPECT_FALSE(MomentumManagementConfig::isValidRwArrayConfiguration(rwArrayConfig));
    }
}

// Only the first numRW columns describe real wheels; garbage beyond that must not reject the config.
TEST(MomentumManagementConfigValidation, IgnoresColumnsBeyondNumRw) {
    auto rwArrayConfig = makeStandardRwArrayConfig();
    rwArrayConfig.GsMatrix_B.col(rwArrayConfig.numRW) = Eigen::Vector3f{0.0F, 9.0F, 0.0F};

    EXPECT_TRUE(MomentumManagementConfig::isValidRwArrayConfiguration(rwArrayConfig));
    EXPECT_NO_THROW((void)MomentumManagementConfig::create(nominalParams(1.0F), rwArrayConfig));
}

// ---------------------------------------------------------------------------------------------------
// Edge cases
// ---------------------------------------------------------------------------------------------------

// Regression guard: with hsMin == 0 and zero momentum the dumping law divides 0/0. The algorithm must
// return zero rather than NaN.
TEST(MomentumManagementEdgeCases, ZeroMomentumWithZeroThresholdIsFinite) {
    MomentumManagementAlgorithm alg{MomentumManagementConfig::create(nominalParams(0.0F), makeStandardRwArrayConfig())};

    const auto Lr_B = alg.update(makeWheelSpeeds({0.0F, 0.0F, 0.0F, 0.0F}));

    EXPECT_TRUE(Lr_B.allFinite());
    EXPECT_TRUE(Lr_B.isZero(kAccuracy));
}

// Momentum below the zero tolerance is treated as zero even when the threshold is zero.
TEST(MomentumManagementEdgeCases, NegligibleMomentumIsFinite) {
    MomentumManagementAlgorithm alg{MomentumManagementConfig::create(nominalParams(0.0F), makeStandardRwArrayConfig())};

    const auto Lr_B = alg.update(makeWheelSpeeds({1e-12F, -1e-12F, 1e-12F, 0.0F}));

    EXPECT_TRUE(Lr_B.allFinite());
    EXPECT_TRUE(Lr_B.isZero(kAccuracy));
}

// A zero threshold acts on the whole stored momentum.
TEST(MomentumManagementEdgeCases, ZeroThresholdDumpsEverything) {
    const auto rwArrayConfig = makeStandardRwArrayConfig();
    const auto wheelSpeeds = makeWheelSpeeds({10.0F, -25.0F, 50.0F, 100.0F});

    MomentumManagementAlgorithm alg{MomentumManagementConfig::create(nominalParams(0.0F), rwArrayConfig)};
    const auto Lr_B = alg.update(wheelSpeeds);

    const Eigen::Vector3f hs_B = clusterMomentum(rwArrayConfig, wheelSpeeds);

    for (Eigen::Index i = 0; i < 3; ++i) {
        EXPECT_NEAR(Lr_B[i], -kNominalK * hs_B[i], kAccuracy) << "component " << i;
    }
}

// Exactly at the threshold the excess vanishes, so no torque is requested.
TEST(MomentumManagementEdgeCases, MomentumExactlyAtThresholdDoesNotDump) {
    const auto rwArrayConfig = makeRwArrayConfig({{0.0F, 0.0F, 1.0F}}, 0.2F);
    // hs = 0.2 * 50 = 10 Nms exactly.
    MomentumManagementAlgorithm alg{MomentumManagementConfig::create(nominalParams(10.0F), rwArrayConfig)};

    const auto Lr_B = alg.update(makeWheelSpeeds({50.0F}));

    EXPECT_TRUE(Lr_B.isZero(kAccuracy));
}

// With no wheels configured there is no momentum to dump.
TEST(MomentumManagementEdgeCases, NoWheelsProducesZeroRequest) {
    MomentumManagementRwArrayConfiguration rwArrayConfig;  // numRW defaults to zero
    MomentumManagementAlgorithm alg{MomentumManagementConfig::create(nominalParams(0.0F), rwArrayConfig)};

    const auto Lr_B = alg.update(makeWheelSpeeds({10.0F, -25.0F, 50.0F, 100.0F}));

    EXPECT_TRUE(Lr_B.isZero(kAccuracy));
}

// Speeds in slots past numRW belong to wheels that do not exist and must not contribute.
TEST(MomentumManagementEdgeCases, SpeedsBeyondNumRwAreIgnored) {
    const auto rwArrayConfig = makeRwArrayConfig({{0.0F, 0.0F, 1.0F}}, 0.2F);

    MomentumManagementAlgorithm alg1{MomentumManagementConfig::create(nominalParams(1.0F), rwArrayConfig)};
    const auto withExtra = alg1.update(makeWheelSpeeds({50.0F, 999.0F, -999.0F, 12345.0F}));

    MomentumManagementAlgorithm alg2{MomentumManagementConfig::create(nominalParams(1.0F), rwArrayConfig)};
    const auto withoutExtra = alg2.update(makeWheelSpeeds({50.0F}));

    EXPECT_TRUE(withExtra.isApprox(withoutExtra));
}

// ---------------------------------------------------------------------------------------------------
// Properties
// ---------------------------------------------------------------------------------------------------

// Each property is implemented in the helpers and also registered as a fuzz target.

// The proportional request acts on exactly the momentum held above the threshold.
TEST(MomentumManagementProperties, TorqueActsOnExcessMomentumOnly) {
    const auto rwArrayConfig = makeStandardRwArrayConfig();
    // Momenta spanning a negligible cluster up to a fully loaded one (~55 Nms), all realizable.
    const std::vector<std::vector<float>> speedCases = {
        {10.0F, -25.0F, 50.0F, 100.0F},
        {-150.0F, 120.0F, -90.0F, 60.0F},
        {0.1F, 0.1F, 0.1F, 0.1F},
        {200.0F, 200.0F, 200.0F, 200.0F},
    };

    for (float hsMin : {0.0F, 0.5F, 5.0F, 50.0F}) {
        for (const auto& speeds : speedCases) {
            testProportionalTorqueOpposesExcessMomentum(rwArrayConfig, makeWheelSpeeds(speeds), nominalParams(hsMin));
        }
    }
}

// Reversing every wheel speed reverses the requested torque, with and without the integral engaged.
TEST(MomentumManagementProperties, IsOddInWheelSpeeds) {
    const auto rwArrayConfig = makeStandardRwArrayConfig();
    const auto wheelSpeeds = makeWheelSpeeds({10.0F, -25.0F, 50.0F, 100.0F});

    testTorqueIsOddInWheelSpeeds(rwArrayConfig, wheelSpeeds, nominalParams());
    testTorqueIsOddInWheelSpeeds(rwArrayConfig, wheelSpeeds, nominalParams(kNominalHsMin, kNominalK, kNominalKi), 20U);
}

// The anti-windup bound holds over a long sustained momentum.
TEST(MomentumManagementProperties, IntegralTermStaysBounded) {
    const auto rwArrayConfig = makeStandardRwArrayConfig();
    const auto wheelSpeeds = makeWheelSpeeds({10.0F, -25.0F, 50.0F, 100.0F});

    testIntegralTermStaysBounded(
        rwArrayConfig, wheelSpeeds, nominalParams(kNominalHsMin, kNominalK, kNominalKi, kTightIntegralLimit));
}

// Deliberately unphysical speeds, far beyond any real wheel: a pure overflow guard, so it asserts only that
// nothing becomes inf or NaN and makes no accuracy claim.
TEST(MomentumManagementProperties, LargeSpeedsStayFinite) {
    const auto rwArrayConfig = makeStandardRwArrayConfig();

    testTorqueStaysFinite(
        rwArrayConfig, makeWheelSpeeds({1e6F, -1e6F, 1e6F, -1e6F}), nominalParams(1.0F, kNominalK, kNominalKi), 20U);
}
