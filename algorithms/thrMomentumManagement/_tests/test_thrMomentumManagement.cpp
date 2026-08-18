#include "thrMomentumManagementTestHelpers.hpp"

#include <Eigen/Core>
#include <algorithm>
#include <limits>
#include <vector>

namespace {

constexpr float kAccuracy = 1e-5F;

// Nominal dumping threshold for the standard four-wheel pyramid with Js = 0.1 and speeds
// (10, -25, 50, 100) r/s, which puts the cluster momentum well above the threshold.
constexpr float kNominalHsMin = 100.0F / 6000.0F * 100.0F;

}  // namespace

// A threshold above the cluster momentum yields a zero request.
TEST(ThrMomentumManagement, NoDumpWhenBelowThreshold) {
    ThrMomentumManagementAlgorithm alg{
        ThrMomentumManagementConfig::create(1000.0F / 6000.0F * 100.0F, makeStandardRwArrayConfig())};

    const auto deltaH_B = alg.update(makeWheelSpeeds({10.0F, -25.0F, 50.0F, 100.0F}));

    EXPECT_TRUE(deltaH_B.isZero(kAccuracy));
}

TEST(ThrMomentumManagement, MatchesReferenceAcrossCases) {
    const auto rwArrayConfig = makeStandardRwArrayConfig();

    regressionTestThrMomentumManagement(
        rwArrayConfig, makeWheelSpeeds({10.0F, -25.0F, 50.0F, 100.0F}), kNominalHsMin, kAccuracy);
    regressionTestThrMomentumManagement(
        rwArrayConfig, makeWheelSpeeds({0.0F, 0.0F, 0.0F, 0.0F}), kNominalHsMin, kAccuracy);
    regressionTestThrMomentumManagement(
        rwArrayConfig, makeWheelSpeeds({-500.0F, 400.0F, -300.0F, 200.0F}), 1.0F, kAccuracy);
    regressionTestThrMomentumManagement(rwArrayConfig, makeWheelSpeeds({1.0F, 1.0F, 1.0F, 1.0F}), 0.0F, kAccuracy);
}

// A single wheel spinning about a body axis dumps straight back along that axis.
TEST(ThrMomentumManagement, SingleWheelDumpsAlongItsSpinAxis) {
    const auto rwArrayConfig = makeRwArrayConfig({{0.0F, 0.0F, 1.0F}}, 0.2F);
    ThrMomentumManagementAlgorithm alg{ThrMomentumManagementConfig::create(1.0F, rwArrayConfig)};

    // hs = 0.2 * 50 = 10 Nms about +z; dumping to hsMin = 1 removes 9 Nms.
    const auto deltaH_B = alg.update(makeWheelSpeeds({50.0F}));

    EXPECT_NEAR(deltaH_B[0], 0.0F, kAccuracy);
    EXPECT_NEAR(deltaH_B[1], 0.0F, kAccuracy);
    EXPECT_NEAR(deltaH_B[2], -9.0F, kAccuracy);
}

// The requested change is anti-parallel to the momentum and leaves exactly hsMin behind.
TEST(ThrMomentumManagement, DumpLeavesExactlyHsMin) {
    const auto rwArrayConfig = makeStandardRwArrayConfig();
    const auto wheelSpeeds = makeWheelSpeeds({10.0F, -25.0F, 50.0F, 100.0F});

    ThrMomentumManagementAlgorithm alg{ThrMomentumManagementConfig::create(kNominalHsMin, rwArrayConfig)};
    const auto deltaH_B = alg.update(wheelSpeeds);

    Eigen::Vector3f hs_B = Eigen::Vector3f::Zero();
    for (uint32_t i = 0U; i < rwArrayConfig.numRW; ++i) {
        hs_B += rwArrayConfig.JsList[i] * wheelSpeeds[i] * rwArrayConfig.GsMatrix_B.col(i);
    }

    // Post-dump momentum magnitude must equal the threshold, and the change must oppose the momentum.
    EXPECT_NEAR((hs_B + deltaH_B).norm(), kNominalHsMin, kAccuracy);
    EXPECT_LT(deltaH_B.normalized().dot(hs_B.normalized()), -1.0F + kAccuracy);
}

// The request is recomputed on every call rather than latched to the first one: repeated updates with the
// same wheel speeds all report the same non-zero request.
TEST(ThrMomentumManagement, RequestIsRecomputedEveryUpdate) {
    ThrMomentumManagementAlgorithm alg{ThrMomentumManagementConfig::create(kNominalHsMin, makeStandardRwArrayConfig())};
    const auto wheelSpeeds = makeWheelSpeeds({10.0F, -25.0F, 50.0F, 100.0F});

    const Eigen::Vector3f first = alg.update(wheelSpeeds);
    ASSERT_FALSE(first.isZero(kAccuracy));

    for (int cycle = 0; cycle < 4; ++cycle) {
        EXPECT_TRUE(alg.update(wheelSpeeds).isApprox(first)) << "cycle " << cycle;
    }
}

// setConfig replaces the configuration, and the very next update uses the new threshold.
TEST(ThrMomentumManagement, SetConfigTakesEffectOnNextUpdate) {
    const auto rwArrayConfig = makeStandardRwArrayConfig();
    ThrMomentumManagementAlgorithm alg{ThrMomentumManagementConfig::create(kNominalHsMin, rwArrayConfig)};
    const auto wheelSpeeds = makeWheelSpeeds({10.0F, -25.0F, 50.0F, 100.0F});

    EXPECT_FALSE(alg.update(wheelSpeeds).isZero(kAccuracy));

    // A threshold above the cluster momentum switches dumping off from the next update onwards.
    alg.setConfig(ThrMomentumManagementConfig::create(1000.0F / 6000.0F * 100.0F, rwArrayConfig));
    EXPECT_TRUE(alg.update(wheelSpeeds).isZero(kAccuracy));
}

TEST(ThrMomentumManagement, ConfigRoundTrips) {
    testThrMomentumManagementSetup(kNominalHsMin, makeStandardRwArrayConfig(), kAccuracy);
    testThrMomentumManagementSetup(0.0F, makeRwArrayConfig({{0.0F, 0.0F, 1.0F}}, 0.05F), kAccuracy);
    // Non-unit spin axes are normalized on construction rather than rejected outright at this scale.
    testThrMomentumManagementSetup(2.5F, makeRwArrayConfig({{3.0F, 4.0F, 0.0F}}, 0.1F), kAccuracy);
}

// ---------------------------------------------------------------------------------------------------
// Config validation
// ---------------------------------------------------------------------------------------------------

TEST(ThrMomentumManagementConfigValidation, RejectsInvalidHsMin) {
    const auto rwArrayConfig = makeStandardRwArrayConfig();

    EXPECT_FALSE(ThrMomentumManagementConfig::isValidHsMin(-1.0F));
    EXPECT_FALSE(ThrMomentumManagementConfig::isValidHsMin(std::numeric_limits<float>::quiet_NaN()));
    EXPECT_FALSE(ThrMomentumManagementConfig::isValidHsMin(std::numeric_limits<float>::infinity()));

    EXPECT_THROW((void)ThrMomentumManagementConfig::create(-1.0F, rwArrayConfig), fsw::invalid_argument);
    EXPECT_THROW((void)ThrMomentumManagementConfig::create(std::numeric_limits<float>::quiet_NaN(), rwArrayConfig),
                 fsw::invalid_argument);
    EXPECT_THROW((void)ThrMomentumManagementConfig::create(std::numeric_limits<float>::infinity(), rwArrayConfig),
                 fsw::invalid_argument);
}

// A zero threshold means "dump all momentum" and is a legitimate setting.
TEST(ThrMomentumManagementConfigValidation, AcceptsZeroHsMin) {
    EXPECT_TRUE(ThrMomentumManagementConfig::isValidHsMin(0.0F));
    EXPECT_NO_THROW((void)ThrMomentumManagementConfig::create(0.0F, makeStandardRwArrayConfig()));
}

TEST(ThrMomentumManagementConfigValidation, RejectsTooManyWheels) {
    auto rwArrayConfig = makeStandardRwArrayConfig();
    rwArrayConfig.numRW = kMaxNumRw + 1U;

    EXPECT_FALSE(ThrMomentumManagementConfig::isValidRwArrayConfiguration(rwArrayConfig));
    EXPECT_THROW((void)ThrMomentumManagementConfig::create(1.0F, rwArrayConfig), fsw::invalid_argument);
}

TEST(ThrMomentumManagementConfigValidation, AcceptsExactlyMaxWheels) {
    ThrMomentumManagementRwArrayConfiguration rwArrayConfig;
    rwArrayConfig.numRW = kMaxNumRw;
    for (uint32_t i = 0U; i < kMaxNumRw; ++i) {
        rwArrayConfig.GsMatrix_B.col(i) = Eigen::Vector3f::UnitZ();
        rwArrayConfig.JsList[i] = 0.1F;
    }

    EXPECT_TRUE(ThrMomentumManagementConfig::isValidRwArrayConfiguration(rwArrayConfig));
    EXPECT_NO_THROW((void)ThrMomentumManagementConfig::create(1.0F, rwArrayConfig));
}

TEST(ThrMomentumManagementConfigValidation, RejectsNonUnitSpinAxis) {
    auto rwArrayConfig = makeStandardRwArrayConfig();
    rwArrayConfig.GsMatrix_B.col(1) = Eigen::Vector3f{0.0F, 2.0F, 0.0F};

    EXPECT_FALSE(ThrMomentumManagementConfig::isValidRwArrayConfiguration(rwArrayConfig));
    EXPECT_THROW((void)ThrMomentumManagementConfig::create(1.0F, rwArrayConfig), fsw::invalid_argument);
}

// A zero spin axis cannot be normalized and must be rejected rather than producing NaN downstream.
TEST(ThrMomentumManagementConfigValidation, RejectsZeroSpinAxis) {
    auto rwArrayConfig = makeStandardRwArrayConfig();
    rwArrayConfig.GsMatrix_B.col(2) = Eigen::Vector3f::Zero();

    EXPECT_FALSE(ThrMomentumManagementConfig::isValidRwArrayConfiguration(rwArrayConfig));
    EXPECT_THROW((void)ThrMomentumManagementConfig::create(1.0F, rwArrayConfig), fsw::invalid_argument);
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
    EXPECT_NO_THROW((void)ThrMomentumManagementConfig::create(1.0F, rwArrayConfig));
}

// ---------------------------------------------------------------------------------------------------
// Edge cases
// ---------------------------------------------------------------------------------------------------

// Regression guard: with hsMin == 0 and zero momentum the dumping law divides 0/0. The algorithm must
// return zero rather than NaN.
TEST(ThrMomentumManagementEdgeCases, ZeroMomentumWithZeroThresholdIsFinite) {
    ThrMomentumManagementAlgorithm alg{ThrMomentumManagementConfig::create(0.0F, makeStandardRwArrayConfig())};

    const auto deltaH_B = alg.update(makeWheelSpeeds({0.0F, 0.0F, 0.0F, 0.0F}));

    EXPECT_TRUE(deltaH_B.allFinite());
    EXPECT_TRUE(deltaH_B.isZero(kAccuracy));
}

// Momentum below the zero tolerance is treated as zero even when the threshold is zero.
TEST(ThrMomentumManagementEdgeCases, NegligibleMomentumIsFinite) {
    ThrMomentumManagementAlgorithm alg{ThrMomentumManagementConfig::create(0.0F, makeStandardRwArrayConfig())};

    const auto deltaH_B = alg.update(makeWheelSpeeds({1e-12F, -1e-12F, 1e-12F, 0.0F}));

    EXPECT_TRUE(deltaH_B.allFinite());
    EXPECT_TRUE(deltaH_B.isZero(kAccuracy));
}

// A zero threshold with real momentum dumps all of it.
TEST(ThrMomentumManagementEdgeCases, ZeroThresholdDumpsEverything) {
    const auto rwArrayConfig = makeStandardRwArrayConfig();
    const auto wheelSpeeds = makeWheelSpeeds({10.0F, -25.0F, 50.0F, 100.0F});

    ThrMomentumManagementAlgorithm alg{ThrMomentumManagementConfig::create(0.0F, rwArrayConfig)};
    const auto deltaH_B = alg.update(wheelSpeeds);

    Eigen::Vector3f hs_B = Eigen::Vector3f::Zero();
    for (uint32_t i = 0U; i < rwArrayConfig.numRW; ++i) {
        hs_B += rwArrayConfig.JsList[i] * wheelSpeeds[i] * rwArrayConfig.GsMatrix_B.col(i);
    }
    EXPECT_TRUE((hs_B + deltaH_B).isZero(kAccuracy));
}

// Exactly at the threshold the strict comparison yields no dumping.
TEST(ThrMomentumManagementEdgeCases, MomentumExactlyAtThresholdDoesNotDump) {
    const auto rwArrayConfig = makeRwArrayConfig({{0.0F, 0.0F, 1.0F}}, 0.2F);
    // hs = 0.2 * 50 = 10 Nms exactly.
    ThrMomentumManagementAlgorithm alg{ThrMomentumManagementConfig::create(10.0F, rwArrayConfig)};

    const auto deltaH_B = alg.update(makeWheelSpeeds({50.0F}));

    EXPECT_TRUE(deltaH_B.isZero(kAccuracy));
}

// With no wheels configured there is no momentum to dump.
TEST(ThrMomentumManagementEdgeCases, NoWheelsProducesZeroRequest) {
    ThrMomentumManagementRwArrayConfiguration rwArrayConfig;  // numRW defaults to zero
    ThrMomentumManagementAlgorithm alg{ThrMomentumManagementConfig::create(0.0F, rwArrayConfig)};

    const auto deltaH_B = alg.update(makeWheelSpeeds({10.0F, -25.0F, 50.0F, 100.0F}));

    EXPECT_TRUE(deltaH_B.isZero(kAccuracy));
}

// Speeds in slots past numRW belong to wheels that do not exist and must not contribute.
TEST(ThrMomentumManagementEdgeCases, SpeedsBeyondNumRwAreIgnored) {
    const auto rwArrayConfig = makeRwArrayConfig({{0.0F, 0.0F, 1.0F}}, 0.2F);

    ThrMomentumManagementAlgorithm alg1{ThrMomentumManagementConfig::create(1.0F, rwArrayConfig)};
    const auto withExtra = alg1.update(makeWheelSpeeds({50.0F, 999.0F, -999.0F, 12345.0F}));

    ThrMomentumManagementAlgorithm alg2{ThrMomentumManagementConfig::create(1.0F, rwArrayConfig)};
    const auto withoutExtra = alg2.update(makeWheelSpeeds({50.0F}));

    EXPECT_TRUE(withExtra.isApprox(withoutExtra));
}

// ---------------------------------------------------------------------------------------------------
// Properties
// ---------------------------------------------------------------------------------------------------

// Dumping never increases the stored momentum, and never removes more than is there.
TEST(ThrMomentumManagementProperties, DumpNeverIncreasesMomentum) {
    const auto rwArrayConfig = makeStandardRwArrayConfig();
    const std::vector<std::vector<float>> speedCases = {
        {10.0F, -25.0F, 50.0F, 100.0F},
        {-500.0F, 400.0F, -300.0F, 200.0F},
        {0.1F, 0.1F, 0.1F, 0.1F},
        {6000.0F, 6000.0F, 6000.0F, 6000.0F},
    };

    for (float hsMin : {0.0F, 0.5F, 5.0F, 50.0F}) {
        for (const auto& speeds : speedCases) {
            const auto wheelSpeeds = makeWheelSpeeds(speeds);
            ThrMomentumManagementAlgorithm alg{ThrMomentumManagementConfig::create(hsMin, rwArrayConfig)};
            const auto deltaH_B = alg.update(wheelSpeeds);
            EXPECT_TRUE(deltaH_B.allFinite());

            Eigen::Vector3f hs_B = Eigen::Vector3f::Zero();
            for (uint32_t i = 0U; i < rwArrayConfig.numRW; ++i) {
                hs_B += rwArrayConfig.JsList[i] * wheelSpeeds[i] * rwArrayConfig.GsMatrix_B.col(i);
            }

            const float before = hs_B.norm();
            const float after = (hs_B + deltaH_B).norm();
            const float tol = kAccuracy * std::max(1.0F, before);
            EXPECT_LE(after, before + tol) << "hsMin " << hsMin;
            EXPECT_GE(after, -tol) << "hsMin " << hsMin;
        }
    }
}

// Reversing every wheel speed reverses the requested momentum change.
TEST(ThrMomentumManagementProperties, IsOddInWheelSpeeds) {
    const auto rwArrayConfig = makeStandardRwArrayConfig();
    const auto wheelSpeeds = makeWheelSpeeds({10.0F, -25.0F, 50.0F, 100.0F});

    ThrMomentumManagementAlgorithm alg1{ThrMomentumManagementConfig::create(kNominalHsMin, rwArrayConfig)};
    const auto forward = alg1.update(wheelSpeeds);

    ThrMomentumManagementAlgorithm alg2{ThrMomentumManagementConfig::create(kNominalHsMin, rwArrayConfig)};
    const auto reversed = alg2.update(Eigen::Vector<float, kMaxNumRw>{-wheelSpeeds});

    for (Eigen::Index i = 0; i < 3; ++i) {
        EXPECT_NEAR(reversed[i], -forward[i], kAccuracy) << "component " << i;
    }
}

// Large but representable speeds must not overflow to inf or NaN.
TEST(ThrMomentumManagementProperties, LargeSpeedsStayFinite) {
    const auto rwArrayConfig = makeStandardRwArrayConfig();
    ThrMomentumManagementAlgorithm alg{ThrMomentumManagementConfig::create(1.0F, rwArrayConfig)};

    const auto deltaH_B = alg.update(makeWheelSpeeds({1e6F, -1e6F, 1e6F, -1e6F}));

    EXPECT_TRUE(deltaH_B.allFinite());
}
