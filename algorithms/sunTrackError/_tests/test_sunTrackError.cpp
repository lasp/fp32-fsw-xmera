#include "sunTrackErrorTestHelpers.hpp"

#include <Eigen/Core>
#include <cmath>
#include <limits>
#include <numbers>

namespace {
constexpr uint64_t kHalfSecNs = 500000000ULL;                        // 0.5 s update period
constexpr float kManeuverRate = std::numbers::pi_v<float> / 180.0F;  // 1 deg/s feed-forward slew

const Eigen::Vector3f kSigmaBN{0.25F, -0.45F, 0.75F};
const Eigen::Vector3f kSigmaRN{0.35F, -0.25F, 0.15F};
const Eigen::Vector3f kOmegaRNN{0.018F, -0.032F, 0.015F};
const Eigen::Vector3f kDomegaRNN{0.048F, -0.022F, 0.025F};
const Eigen::Vector3d kRBN_N{-30.0, 20.0, -50.0};
const Eigen::Vector3d kRSN_N{1.0, 2.0, 3.0};
const Eigen::Vector3f kSensitiveHat_B{0.0F, -1.0F, 0.0F};
}  // namespace

// ---------------------------------------------------------------------------
// Regression tests: the algorithm's adjusted-reference output matches the independent reference.
// ---------------------------------------------------------------------------

// No optional messages -> no maneuver: the adjusted reference is the input reference unchanged.
TEST(SunTrackErrorTest, RegressionPassThrough) {
    regressionTestSunTrackError(Eigen::Vector3f::Zero(),
                                0.0F,
                                false,
                                kSigmaBN,
                                kSigmaRN,
                                kOmegaRNN,
                                kDomegaRNN,
                                Eigen::Vector3d::Zero(),
                                Eigen::Vector3d::Zero(),
                                kHalfSecNs,
                                12);
}

// Sun-avoidance maneuver actively feeding forward (residual angle > 0 throughout).
TEST(SunTrackErrorTest, RegressionSunAvoidanceFeedingForward) {
    regressionTestSunTrackError(kSensitiveHat_B,
                                kManeuverRate,
                                true,
                                kSigmaBN,
                                kSigmaRN,
                                kOmegaRNN,
                                kDomegaRNN,
                                kRBN_N,
                                kRSN_N,
                                kHalfSecNs,
                                12);
}

// Long run: the residual maneuver angle decays to zero and stays clamped.
TEST(SunTrackErrorTest, RegressionSunAvoidanceDecaysToZero) {
    regressionTestSunTrackError(kSensitiveHat_B,
                                kManeuverRate,
                                true,
                                kSigmaBN,
                                kSigmaRN,
                                kOmegaRNN,
                                kDomegaRNN,
                                kRBN_N,
                                kRSN_N,
                                kHalfSecNs,
                                400);
}

// ---------------------------------------------------------------------------
// Setup tests: Config validators and getters.
// ---------------------------------------------------------------------------

// sensitiveHat_B is validated only when the maneuver is enabled (computeAngleStart == true).
TEST(SunTrackErrorConfigTest, RejectsNonFiniteSensitiveHat) {
    const Eigen::Vector3f bad{std::nanf(""), 0.0F, 0.0F};
    EXPECT_THROW((void)SunTrackErrorConfig::create(bad, 0.0F, true), fsw::invalid_argument);
}

// A grossly non-unit sensitiveHat_B is rejected when the maneuver is enabled (must be within 1e-3 of unit).
TEST(SunTrackErrorConfigTest, RejectsNonUnitSensitiveHat) {
    const Eigen::Vector3f nonUnit{0.0F, -2.0F, 0.0F};
    EXPECT_THROW((void)SunTrackErrorConfig::create(nonUnit, kManeuverRate, true), fsw::invalid_argument);
}

TEST(SunTrackErrorConfigTest, RejectsNonFiniteAngleRate) {
    EXPECT_THROW((void)SunTrackErrorConfig::create(kSensitiveHat_B, std::numeric_limits<float>::infinity(), false),
                 fsw::invalid_argument);
    EXPECT_THROW((void)SunTrackErrorConfig::create(kSensitiveHat_B, std::nanf(""), false), fsw::invalid_argument);
}

TEST(SunTrackErrorConfigTest, AcceptsValidInputs) {
    EXPECT_NO_THROW((void)SunTrackErrorConfig::create(kSensitiveHat_B, kManeuverRate, true));
    // sensitiveHat_B is unused when the maneuver is disabled, so it is not unit-length checked.
    EXPECT_NO_THROW((void)SunTrackErrorConfig::create(Eigen::Vector3f::Zero(), 0.0F, false));
}

// ---------------------------------------------------------------------------
// Property tests.
// ---------------------------------------------------------------------------

TEST(SunTrackErrorTest, PropertyPassThroughEqualsInputRef) {
    propertyPassThroughEqualsInputRef(kSigmaBN, kSigmaRN, kOmegaRNN, kDomegaRNN);
}

TEST(SunTrackErrorTest, PropertyManeuverOutputBoundedAndFinite) {
    propertyManeuverOutputBoundedAndFinite(kSigmaBN, kSigmaRN, kOmegaRNN, kDomegaRNN);
}

TEST(SunTrackErrorTest, PropertyDecayedManeuverEqualsInputRef) {
    propertyDecayedManeuverEqualsInputRef(kSigmaBN, kSigmaRN, kOmegaRNN, kDomegaRNN);
}

TEST(SunTrackErrorTest, PropertyReInitializeRestartsManeuver) {
    propertyReInitializeRestartsManeuver(kSigmaBN, kSigmaRN, kOmegaRNN, kDomegaRNN);
}

// ---------------------------------------------------------------------------
// Edge case tests.
// ---------------------------------------------------------------------------

// Zero maneuver rate: the initial maneuver angle never decays, so the adjusted reference is constant
// across every step.
TEST(SunTrackErrorTest, EdgeZeroAngleRate) {
    const auto config = SunTrackErrorConfig::create(kSensitiveHat_B, 0.0F, true);
    SunTrackErrorAlgorithm alg{config};
    const SunTrackErrorAttRefInputs refIn{kSigmaRN, kOmegaRNN, kDomegaRNN};

    const SunTrackErrorOutput first = alg.update(kSigmaBN, refIn, kRBN_N, kRSN_N, 0);
    constexpr float tol = 1e-6F;
    for (int k = 1; k < 12; ++k) {
        const SunTrackErrorOutput out =
            alg.update(kSigmaBN, refIn, kRBN_N, kRSN_N, static_cast<uint64_t>(k) * kHalfSecNs);
        for (int i = 0; i < 3; ++i) {
            EXPECT_NEAR(out.sigma_RN(i), first.sigma_RN(i), tol);
            EXPECT_NEAR(out.omega_RN_N(i), first.omega_RN_N(i), tol);
            EXPECT_NEAR(out.domega_RN_N(i), first.domega_RN_N(i), tol);
        }
    }
}

// Zero navigation and reference inputs with the maneuver disabled: the output reference is zero.
TEST(SunTrackErrorTest, EdgeZeroInputsPassThrough) {
    propertyPassThroughEqualsInputRef(
        Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero());
}

// Body attitude close to the reference: a small but well-defined maneuver angle; output stays finite
// and bounded.
TEST(SunTrackErrorTest, EdgeSmallManeuverNearAlignment) {
    const Eigen::Vector3f sigmaBN_near = kSigmaRN + Eigen::Vector3f{0.02F, -0.01F, 0.015F};
    propertyManeuverOutputBoundedAndFinite(sigmaBN_near, kSigmaRN, kOmegaRNN, kDomegaRNN);
}

TEST(SunTrackErrorConfigTest, GettersRoundTrip) {
    // sensitiveHat_B is renormalized on storage; a near-unit input (within the 1e-3 tolerance) must come
    // back as the exact unit direction.
    const Eigen::Vector3f rawSensitive{0.0F, -1.0005F, 0.0F};
    const auto config = SunTrackErrorConfig::create(rawSensitive, kManeuverRate, true);

    constexpr float tol = 1e-6F;
    const Eigen::Vector3f expectedSensitive = rawSensitive.normalized();
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(config.getSensitiveHat_B()(i), expectedSensitive(i), tol);
    }
    EXPECT_NEAR(config.getAngleRate(), kManeuverRate, tol);
    EXPECT_TRUE(config.getComputeAngleStart());

    const auto configNoManeuver = SunTrackErrorConfig::create(kSensitiveHat_B, 0.0F, false);
    EXPECT_FALSE(configNoManeuver.getComputeAngleStart());
}
