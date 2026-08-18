#include "thrMomentumManagementTestHelpers.hpp"

#include <Eigen/Core>
#include <algorithm>
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

ThrMomentumManagementControlParameters nominalParams(float hsMin = kNominalHsMin, float K = kNominalK) {
    return {.hsMin = hsMin, .K = K};
}

}  // namespace

// A threshold above the cluster momentum yields a zero torque.
TEST(ThrMomentumManagement, NoDumpWhenBelowThreshold) {
    ThrMomentumManagementAlgorithm alg{
        ThrMomentumManagementConfig::create(nominalParams(kHighHsMin), makeStandardRwArrayConfig())};

    const auto Lr_B = alg.update(makeWheelSpeeds({10.0F, -25.0F, 50.0F, 100.0F}));

    EXPECT_TRUE(Lr_B.isZero(kAccuracy));
}

TEST(ThrMomentumManagement, MatchesReferenceAcrossCases) {
    const auto rwArrayConfig = makeStandardRwArrayConfig();

    regressionTestThrMomentumManagement(
        rwArrayConfig, makeWheelSpeeds({10.0F, -25.0F, 50.0F, 100.0F}), nominalParams(), kAccuracy);
    regressionTestThrMomentumManagement(
        rwArrayConfig, makeWheelSpeeds({0.0F, 0.0F, 0.0F, 0.0F}), nominalParams(), kAccuracy);
    regressionTestThrMomentumManagement(
        rwArrayConfig, makeWheelSpeeds({-150.0F, 120.0F, -90.0F, 60.0F}), nominalParams(1.0F), kAccuracy);
    regressionTestThrMomentumManagement(
        rwArrayConfig, makeWheelSpeeds({1.0F, 1.0F, 1.0F, 1.0F}), nominalParams(0.0F), kAccuracy);
    // A gain an order of magnitude away must carry through unchanged.
    regressionTestThrMomentumManagement(
        rwArrayConfig, makeWheelSpeeds({10.0F, -25.0F, 50.0F, 100.0F}), nominalParams(1.0F, 0.005F), kAccuracy);
}

// A single wheel spinning about a body axis is dumped straight back along that axis.
TEST(ThrMomentumManagement, SingleWheelDumpsAlongItsSpinAxis) {
    const auto rwArrayConfig = makeRwArrayConfig({{0.0F, 0.0F, 1.0F}}, 0.2F);
    ThrMomentumManagementAlgorithm alg{ThrMomentumManagementConfig::create(nominalParams(1.0F), rwArrayConfig)};

    // hs = 0.2 * 50 = 10 Nms about +z; the excess above hsMin = 1 is 9 Nms, so the torque is -K * 9.
    const auto Lr_B = alg.update(makeWheelSpeeds({50.0F}));

    EXPECT_NEAR(Lr_B[0], 0.0F, kAccuracy);
    EXPECT_NEAR(Lr_B[1], 0.0F, kAccuracy);
    EXPECT_NEAR(Lr_B[2], -kNominalK * 9.0F, kAccuracy);
}

// The torque opposes the stored momentum and acts on exactly the momentum held above the threshold.
TEST(ThrMomentumManagement, TorqueMagnitudeIsGainTimesExcessMomentum) {
    const auto rwArrayConfig = makeStandardRwArrayConfig();
    const auto wheelSpeeds = makeWheelSpeeds({10.0F, -25.0F, 50.0F, 100.0F});

    ThrMomentumManagementAlgorithm alg{ThrMomentumManagementConfig::create(nominalParams(), rwArrayConfig)};
    const auto Lr_B = alg.update(wheelSpeeds);

    const Eigen::Vector3f hs_B = clusterMomentum(rwArrayConfig, wheelSpeeds);

    EXPECT_NEAR(Lr_B.norm(), kNominalK * (hs_B.norm() - kNominalHsMin), kAccuracy);
    EXPECT_LT(Lr_B.normalized().dot(hs_B.normalized()), -1.0F + kAccuracy);
}

// The torque scales linearly with the gain: doubling K doubles the request.
TEST(ThrMomentumManagement, TorqueScalesLinearlyWithGain) {
    const auto rwArrayConfig = makeStandardRwArrayConfig();
    const auto wheelSpeeds = makeWheelSpeeds({10.0F, -25.0F, 50.0F, 100.0F});

    ThrMomentumManagementAlgorithm alg1{
        ThrMomentumManagementConfig::create(nominalParams(kNominalHsMin, 0.05F), rwArrayConfig)};
    ThrMomentumManagementAlgorithm alg2{
        ThrMomentumManagementConfig::create(nominalParams(kNominalHsMin, 0.10F), rwArrayConfig)};

    const Eigen::Vector3f single = alg1.update(wheelSpeeds);
    const Eigen::Vector3f doubled = alg2.update(wheelSpeeds);

    for (Eigen::Index i = 0; i < 3; ++i) {
        EXPECT_NEAR(doubled[i], 2.0F * single[i], kAccuracy) << "component " << i;
    }
}

// The request is recomputed on every call rather than latched to the first one: repeated updates with the
// same wheel speeds all report the same non-zero request.
TEST(ThrMomentumManagement, RequestIsRecomputedEveryUpdate) {
    ThrMomentumManagementAlgorithm alg{
        ThrMomentumManagementConfig::create(nominalParams(), makeStandardRwArrayConfig())};
    const auto wheelSpeeds = makeWheelSpeeds({10.0F, -25.0F, 50.0F, 100.0F});

    const Eigen::Vector3f first = alg.update(wheelSpeeds);
    ASSERT_FALSE(first.isZero(kAccuracy));

    for (int cycle = 0; cycle < 4; ++cycle) {
        EXPECT_TRUE(alg.update(wheelSpeeds).isApprox(first)) << "cycle " << cycle;
    }
}

// setConfig replaces the configuration, and the very next update uses the new parameters.
TEST(ThrMomentumManagement, SetConfigTakesEffectOnNextUpdate) {
    const auto rwArrayConfig = makeStandardRwArrayConfig();
    ThrMomentumManagementAlgorithm alg{ThrMomentumManagementConfig::create(nominalParams(), rwArrayConfig)};
    const auto wheelSpeeds = makeWheelSpeeds({10.0F, -25.0F, 50.0F, 100.0F});

    EXPECT_FALSE(alg.update(wheelSpeeds).isZero(kAccuracy));

    // A threshold above the cluster momentum switches dumping off from the next update onwards.
    alg.setConfig(ThrMomentumManagementConfig::create(nominalParams(kHighHsMin), rwArrayConfig));
    EXPECT_TRUE(alg.update(wheelSpeeds).isZero(kAccuracy));
}

TEST(ThrMomentumManagement, ConfigRoundTrips) {
    testThrMomentumManagementSetup(nominalParams(), makeStandardRwArrayConfig(), kAccuracy);
    testThrMomentumManagementSetup(
        nominalParams(0.0F, 0.05F), makeRwArrayConfig({{0.0F, 0.0F, 1.0F}}, 0.05F), kAccuracy);
    // Non-unit spin axes are normalized on construction rather than rejected outright at this scale.
    testThrMomentumManagementSetup(nominalParams(2.5F), makeRwArrayConfig({{3.0F, 4.0F, 0.0F}}, 0.1F), kAccuracy);
}

// ---------------------------------------------------------------------------------------------------
// Config validation
// ---------------------------------------------------------------------------------------------------

TEST(ThrMomentumManagementConfigValidation, RejectsInvalidHsMin) {
    const auto rwArrayConfig = makeStandardRwArrayConfig();

    EXPECT_FALSE(ThrMomentumManagementConfig::isValidHsMin(-1.0F));
    EXPECT_FALSE(ThrMomentumManagementConfig::isValidHsMin(std::numeric_limits<float>::quiet_NaN()));
    EXPECT_FALSE(ThrMomentumManagementConfig::isValidHsMin(std::numeric_limits<float>::infinity()));

    EXPECT_THROW((void)ThrMomentumManagementConfig::create(nominalParams(-1.0F), rwArrayConfig), fsw::invalid_argument);
    EXPECT_THROW((void)ThrMomentumManagementConfig::create(nominalParams(std::numeric_limits<float>::quiet_NaN()),
                                                           rwArrayConfig),
                 fsw::invalid_argument);
    EXPECT_THROW(
        (void)ThrMomentumManagementConfig::create(nominalParams(std::numeric_limits<float>::infinity()), rwArrayConfig),
        fsw::invalid_argument);
}

// A zero threshold means "dump all momentum" and is a legitimate setting.
TEST(ThrMomentumManagementConfigValidation, AcceptsZeroHsMin) {
    EXPECT_TRUE(ThrMomentumManagementConfig::isValidHsMin(0.0F));
    EXPECT_NO_THROW((void)ThrMomentumManagementConfig::create(nominalParams(0.0F), makeStandardRwArrayConfig()));
}

// The gain must be strictly positive: zero would disable dumping entirely and a negative gain would drive the
// wheels away from the threshold instead of towards it.
TEST(ThrMomentumManagementConfigValidation, RejectsInvalidK) {
    const auto rwArrayConfig = makeStandardRwArrayConfig();

    EXPECT_FALSE(ThrMomentumManagementConfig::isValidK(0.0F));
    EXPECT_FALSE(ThrMomentumManagementConfig::isValidK(-1.0F));
    EXPECT_FALSE(ThrMomentumManagementConfig::isValidK(std::numeric_limits<float>::quiet_NaN()));
    EXPECT_FALSE(ThrMomentumManagementConfig::isValidK(std::numeric_limits<float>::infinity()));

    EXPECT_THROW((void)ThrMomentumManagementConfig::create(nominalParams(kNominalHsMin, 0.0F), rwArrayConfig),
                 fsw::invalid_argument);
    EXPECT_THROW((void)ThrMomentumManagementConfig::create(nominalParams(kNominalHsMin, -1.0F), rwArrayConfig),
                 fsw::invalid_argument);
    EXPECT_THROW((void)ThrMomentumManagementConfig::create(
                     nominalParams(kNominalHsMin, std::numeric_limits<float>::quiet_NaN()), rwArrayConfig),
                 fsw::invalid_argument);
}

TEST(ThrMomentumManagementConfigValidation, AcceptsSmallPositiveK) {
    EXPECT_TRUE(ThrMomentumManagementConfig::isValidK(1e-6F));
    EXPECT_NO_THROW(
        (void)ThrMomentumManagementConfig::create(nominalParams(kNominalHsMin, 1e-6F), makeStandardRwArrayConfig()));
}

TEST(ThrMomentumManagementConfigValidation, RejectsTooManyWheels) {
    auto rwArrayConfig = makeStandardRwArrayConfig();
    rwArrayConfig.numRW = kMaxNumRw + 1U;

    EXPECT_FALSE(ThrMomentumManagementConfig::isValidRwArrayConfiguration(rwArrayConfig));
    EXPECT_THROW((void)ThrMomentumManagementConfig::create(nominalParams(1.0F), rwArrayConfig), fsw::invalid_argument);
}

TEST(ThrMomentumManagementConfigValidation, AcceptsExactlyMaxWheels) {
    ThrMomentumManagementRwArrayConfiguration rwArrayConfig;
    rwArrayConfig.numRW = kMaxNumRw;
    for (uint32_t i = 0U; i < kMaxNumRw; ++i) {
        rwArrayConfig.GsMatrix_B.col(i) = Eigen::Vector3f::UnitZ();
        rwArrayConfig.JsList[i] = 0.1F;
    }

    EXPECT_TRUE(ThrMomentumManagementConfig::isValidRwArrayConfiguration(rwArrayConfig));
    EXPECT_NO_THROW((void)ThrMomentumManagementConfig::create(nominalParams(1.0F), rwArrayConfig));
}

TEST(ThrMomentumManagementConfigValidation, RejectsNonUnitSpinAxis) {
    auto rwArrayConfig = makeStandardRwArrayConfig();
    rwArrayConfig.GsMatrix_B.col(1) = Eigen::Vector3f{0.0F, 2.0F, 0.0F};

    EXPECT_FALSE(ThrMomentumManagementConfig::isValidRwArrayConfiguration(rwArrayConfig));
    EXPECT_THROW((void)ThrMomentumManagementConfig::create(nominalParams(1.0F), rwArrayConfig), fsw::invalid_argument);
}

// A zero spin axis cannot be normalized and must be rejected rather than producing NaN downstream.
TEST(ThrMomentumManagementConfigValidation, RejectsZeroSpinAxis) {
    auto rwArrayConfig = makeStandardRwArrayConfig();
    rwArrayConfig.GsMatrix_B.col(2) = Eigen::Vector3f::Zero();

    EXPECT_FALSE(ThrMomentumManagementConfig::isValidRwArrayConfiguration(rwArrayConfig));
    EXPECT_THROW((void)ThrMomentumManagementConfig::create(nominalParams(1.0F), rwArrayConfig), fsw::invalid_argument);
}

TEST(ThrMomentumManagementConfigValidation, RejectsNonFiniteEntries) {
    {
        auto rwArrayConfig = makeStandardRwArrayConfig();
        rwArrayConfig.JsList[2] = std::numeric_limits<float>::quiet_NaN();
        EXPECT_FALSE(ThrMomentumManagementConfig::isValidRwArrayConfiguration(rwArrayConfig));
    }
    {
        auto rwArrayConfig = makeStandardRwArrayConfig();
        rwArrayConfig.GsMatrix_B(0, 0) = std::numeric_limits<float>::infinity();
        EXPECT_FALSE(ThrMomentumManagementConfig::isValidRwArrayConfiguration(rwArrayConfig));
    }
}

// Only the first numRW columns describe real wheels; garbage beyond that must not reject the config.
TEST(ThrMomentumManagementConfigValidation, IgnoresColumnsBeyondNumRw) {
    auto rwArrayConfig = makeStandardRwArrayConfig();
    rwArrayConfig.GsMatrix_B.col(rwArrayConfig.numRW) = Eigen::Vector3f{0.0F, 9.0F, 0.0F};

    EXPECT_TRUE(ThrMomentumManagementConfig::isValidRwArrayConfiguration(rwArrayConfig));
    EXPECT_NO_THROW((void)ThrMomentumManagementConfig::create(nominalParams(1.0F), rwArrayConfig));
}

// ---------------------------------------------------------------------------------------------------
// Edge cases
// ---------------------------------------------------------------------------------------------------

// Regression guard: with hsMin == 0 and zero momentum the dumping law divides 0/0. The algorithm must
// return zero rather than NaN.
TEST(ThrMomentumManagementEdgeCases, ZeroMomentumWithZeroThresholdIsFinite) {
    ThrMomentumManagementAlgorithm alg{
        ThrMomentumManagementConfig::create(nominalParams(0.0F), makeStandardRwArrayConfig())};

    const auto Lr_B = alg.update(makeWheelSpeeds({0.0F, 0.0F, 0.0F, 0.0F}));

    EXPECT_TRUE(Lr_B.allFinite());
    EXPECT_TRUE(Lr_B.isZero(kAccuracy));
}

// Momentum below the zero tolerance is treated as zero even when the threshold is zero.
TEST(ThrMomentumManagementEdgeCases, NegligibleMomentumIsFinite) {
    ThrMomentumManagementAlgorithm alg{
        ThrMomentumManagementConfig::create(nominalParams(0.0F), makeStandardRwArrayConfig())};

    const auto Lr_B = alg.update(makeWheelSpeeds({1e-12F, -1e-12F, 1e-12F, 0.0F}));

    EXPECT_TRUE(Lr_B.allFinite());
    EXPECT_TRUE(Lr_B.isZero(kAccuracy));
}

// A zero threshold acts on the whole stored momentum.
TEST(ThrMomentumManagementEdgeCases, ZeroThresholdDumpsEverything) {
    const auto rwArrayConfig = makeStandardRwArrayConfig();
    const auto wheelSpeeds = makeWheelSpeeds({10.0F, -25.0F, 50.0F, 100.0F});

    ThrMomentumManagementAlgorithm alg{ThrMomentumManagementConfig::create(nominalParams(0.0F), rwArrayConfig)};
    const auto Lr_B = alg.update(wheelSpeeds);

    const Eigen::Vector3f hs_B = clusterMomentum(rwArrayConfig, wheelSpeeds);

    for (Eigen::Index i = 0; i < 3; ++i) {
        EXPECT_NEAR(Lr_B[i], -kNominalK * hs_B[i], kAccuracy) << "component " << i;
    }
}

// Exactly at the threshold the excess vanishes, so no torque is requested.
TEST(ThrMomentumManagementEdgeCases, MomentumExactlyAtThresholdDoesNotDump) {
    const auto rwArrayConfig = makeRwArrayConfig({{0.0F, 0.0F, 1.0F}}, 0.2F);
    // hs = 0.2 * 50 = 10 Nms exactly.
    ThrMomentumManagementAlgorithm alg{ThrMomentumManagementConfig::create(nominalParams(10.0F), rwArrayConfig)};

    const auto Lr_B = alg.update(makeWheelSpeeds({50.0F}));

    EXPECT_TRUE(Lr_B.isZero(kAccuracy));
}

// With no wheels configured there is no momentum to dump.
TEST(ThrMomentumManagementEdgeCases, NoWheelsProducesZeroRequest) {
    ThrMomentumManagementRwArrayConfiguration rwArrayConfig;  // numRW defaults to zero
    ThrMomentumManagementAlgorithm alg{ThrMomentumManagementConfig::create(nominalParams(0.0F), rwArrayConfig)};

    const auto Lr_B = alg.update(makeWheelSpeeds({10.0F, -25.0F, 50.0F, 100.0F}));

    EXPECT_TRUE(Lr_B.isZero(kAccuracy));
}

// Speeds in slots past numRW belong to wheels that do not exist and must not contribute.
TEST(ThrMomentumManagementEdgeCases, SpeedsBeyondNumRwAreIgnored) {
    const auto rwArrayConfig = makeRwArrayConfig({{0.0F, 0.0F, 1.0F}}, 0.2F);

    ThrMomentumManagementAlgorithm alg1{ThrMomentumManagementConfig::create(nominalParams(1.0F), rwArrayConfig)};
    const auto withExtra = alg1.update(makeWheelSpeeds({50.0F, 999.0F, -999.0F, 12345.0F}));

    ThrMomentumManagementAlgorithm alg2{ThrMomentumManagementConfig::create(nominalParams(1.0F), rwArrayConfig)};
    const auto withoutExtra = alg2.update(makeWheelSpeeds({50.0F}));

    EXPECT_TRUE(withExtra.isApprox(withoutExtra));
}

// ---------------------------------------------------------------------------------------------------
// Properties
// ---------------------------------------------------------------------------------------------------

// The torque magnitude is the gain times the momentum held above the threshold, and never acts on momentum
// the cluster does not hold.
TEST(ThrMomentumManagementProperties, TorqueActsOnExcessMomentumOnly) {
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
            const auto wheelSpeeds = makeWheelSpeeds(speeds);
            ThrMomentumManagementAlgorithm alg{
                ThrMomentumManagementConfig::create(nominalParams(hsMin), rwArrayConfig)};
            const auto Lr_B = alg.update(wheelSpeeds);
            EXPECT_TRUE(Lr_B.allFinite());

            const float hs = clusterMomentum(rwArrayConfig, wheelSpeeds).norm();

            EXPECT_NEAR(Lr_B.norm(), kNominalK * std::max(0.0F, hs - hsMin), kAccuracy) << "hsMin " << hsMin;
            EXPECT_LE(Lr_B.norm(), (kNominalK * hs) + kAccuracy) << "hsMin " << hsMin;
        }
    }
}

// Reversing every wheel speed reverses the requested torque.
TEST(ThrMomentumManagementProperties, IsOddInWheelSpeeds) {
    const auto rwArrayConfig = makeStandardRwArrayConfig();
    const auto wheelSpeeds = makeWheelSpeeds({10.0F, -25.0F, 50.0F, 100.0F});

    ThrMomentumManagementAlgorithm alg1{ThrMomentumManagementConfig::create(nominalParams(), rwArrayConfig)};
    const auto forward = alg1.update(wheelSpeeds);

    ThrMomentumManagementAlgorithm alg2{ThrMomentumManagementConfig::create(nominalParams(), rwArrayConfig)};
    const auto reversed = alg2.update(Eigen::Vector<float, kMaxNumRw>{-wheelSpeeds});

    for (Eigen::Index i = 0; i < 3; ++i) {
        EXPECT_NEAR(reversed[i], -forward[i], kAccuracy) << "component " << i;
    }
}

// Deliberately unphysical speeds, far beyond any real wheel: a pure overflow guard, so it asserts only that
// nothing becomes inf or NaN and makes no accuracy claim.
TEST(ThrMomentumManagementProperties, LargeSpeedsStayFinite) {
    const auto rwArrayConfig = makeStandardRwArrayConfig();
    ThrMomentumManagementAlgorithm alg{ThrMomentumManagementConfig::create(nominalParams(1.0F), rwArrayConfig)};

    const auto Lr_B = alg.update(makeWheelSpeeds({1e6F, -1e6F, 1e6F, -1e6F}));

    EXPECT_TRUE(Lr_B.allFinite());
}
