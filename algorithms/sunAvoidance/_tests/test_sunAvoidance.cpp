#include "sunAvoidanceTestHelpers.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <utility>

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
TEST(SunAvoidanceTest, RegressionPassThrough) {
    regressionTestSunAvoidance(Eigen::Vector3f::Zero(),
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
TEST(SunAvoidanceTest, RegressionSunAvoidanceFeedingForward) {
    regressionTestSunAvoidance(kSensitiveHat_B,
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
TEST(SunAvoidanceTest, RegressionSunAvoidanceDecaysToZero) {
    regressionTestSunAvoidance(kSensitiveHat_B,
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
TEST(SunAvoidanceConfigTest, RejectsNonFiniteSensitiveHat) {
    const Eigen::Vector3f bad{std::nanf(""), 0.0F, 0.0F};
    EXPECT_THROW((void)SunAvoidanceConfig::create(bad, 0.0F, true), fsw::invalid_argument);
}

// A grossly non-unit sensitiveHat_B is rejected when the maneuver is enabled (must be within 1e-3 of unit).
TEST(SunAvoidanceConfigTest, RejectsNonUnitSensitiveHat) {
    const Eigen::Vector3f nonUnit{0.0F, -2.0F, 0.0F};
    EXPECT_THROW((void)SunAvoidanceConfig::create(nonUnit, kManeuverRate, true), fsw::invalid_argument);
}

TEST(SunAvoidanceConfigTest, RejectsNonFiniteAngleRate) {
    EXPECT_THROW((void)SunAvoidanceConfig::create(kSensitiveHat_B, std::numeric_limits<float>::infinity(), false),
                 fsw::invalid_argument);
    EXPECT_THROW((void)SunAvoidanceConfig::create(kSensitiveHat_B, std::nanf(""), false), fsw::invalid_argument);
}

TEST(SunAvoidanceConfigTest, AcceptsValidInputs) {
    EXPECT_NO_THROW((void)SunAvoidanceConfig::create(kSensitiveHat_B, kManeuverRate, true));
    // sensitiveHat_B is unused when the maneuver is disabled, so it is not unit-length checked.
    EXPECT_NO_THROW((void)SunAvoidanceConfig::create(Eigen::Vector3f::Zero(), 0.0F, false));
}

// ---------------------------------------------------------------------------
// Property tests.
// ---------------------------------------------------------------------------

TEST(SunAvoidanceTest, PropertyPassThroughEqualsInputRef) {
    propertyPassThroughEqualsInputRef(kSigmaBN, kSigmaRN, kOmegaRNN, kDomegaRNN);
}

TEST(SunAvoidanceTest, PropertyManeuverOutputBoundedAndFinite) {
    propertyManeuverOutputBoundedAndFinite(kSigmaBN, kSigmaRN, kOmegaRNN, kDomegaRNN);
}

TEST(SunAvoidanceTest, PropertyDecayedManeuverEqualsInputRef) {
    propertyDecayedManeuverEqualsInputRef(kSigmaBN, kSigmaRN, kOmegaRNN, kDomegaRNN);
}

TEST(SunAvoidanceTest, PropertyReInitializeRestartsManeuver) {
    propertyReInitializeRestartsManeuver(kSigmaBN, kSigmaRN, kOmegaRNN, kDomegaRNN);
}

// ---------------------------------------------------------------------------
// Edge case tests.
// ---------------------------------------------------------------------------

// Zero maneuver rate: the initial maneuver angle never decays, so the adjusted reference is constant
// across every step.
TEST(SunAvoidanceTest, EdgeZeroAngleRate) {
    const auto config = SunAvoidanceConfig::create(kSensitiveHat_B, 0.0F, true);
    SunAvoidanceAlgorithm alg{config};
    const SunAvoidanceAttRefInputs refIn{kSigmaRN, kOmegaRNN, kDomegaRNN};

    const SunAvoidanceOutput first = alg.update(kSigmaBN, refIn, kRBN_N, kRSN_N, 0);
    constexpr float tol = 1e-6F;
    for (int k = 1; k < 12; ++k) {
        const SunAvoidanceOutput out =
            alg.update(kSigmaBN, refIn, kRBN_N, kRSN_N, static_cast<uint64_t>(k) * kHalfSecNs);
        for (int i = 0; i < 3; ++i) {
            EXPECT_NEAR(out.sigma_RN(i), first.sigma_RN(i), tol);
            EXPECT_NEAR(out.omega_RN_N(i), first.omega_RN_N(i), tol);
            EXPECT_NEAR(out.domega_RN_N(i), first.domega_RN_N(i), tol);
        }
    }
}

// Zero navigation and reference inputs with the maneuver disabled: the output reference is zero.
TEST(SunAvoidanceTest, EdgeZeroInputsPassThrough) {
    propertyPassThroughEqualsInputRef(
        Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero());
}

// Body attitude close to the reference: a small but well-defined maneuver angle; output stays finite
// and bounded.
TEST(SunAvoidanceTest, EdgeSmallManeuverNearAlignment) {
    const Eigen::Vector3f sigmaBN_near = kSigmaRN + Eigen::Vector3f{0.02F, -0.01F, 0.015F};
    propertyManeuverOutputBoundedAndFinite(sigmaBN_near, kSigmaRN, kOmegaRNN, kDomegaRNN);
}

// No usable Sun information with the maneuver enabled: a Sun position coincident with the spacecraft
// (undefined direction) or a zero Sun position (no ephemeris). Both skip the maneuver, so the adjusted
// reference passes through.
TEST(SunAvoidanceTest, EdgeNoSunInformationPassThrough) {
    const auto config = SunAvoidanceConfig::create(kSensitiveHat_B, kManeuverRate, true);
    const SunAvoidanceAttRefInputs refIn{kSigmaRN, kOmegaRNN, kDomegaRNN};
    const Eigen::Matrix3f dcm_RN_in = mrpToDcm(kSigmaRN);
    constexpr float tol = 1e-5F;

    const std::array<std::pair<Eigen::Vector3d, Eigen::Vector3d>, 2> degenerateGeometry{{
        {Eigen::Vector3d{10.0, -20.0, 30.0}, Eigen::Vector3d{10.0, -20.0, 30.0}},  // r_SN_N == r_BN_N
        {Eigen::Vector3d{10.0, -20.0, 30.0}, Eigen::Vector3d::Zero()},             // r_SN_N == 0
    }};

    for (const auto& [r_BN_N, r_SN_N] : degenerateGeometry) {
        SunAvoidanceAlgorithm alg{config};
        for (int k = 0; k < 5; ++k) {
            const SunAvoidanceOutput out =
                alg.update(kSigmaBN, refIn, r_BN_N, r_SN_N, static_cast<uint64_t>(k) * kHalfSecNs);
            EXPECT_TRUE(out.sigma_RN.allFinite());
            const Eigen::Matrix3f dcm_RN_out = mrpToDcm(out.sigma_RN);
            for (int r = 0; r < 3; ++r) {
                for (int c = 0; c < 3; ++c) {
                    EXPECT_NEAR(dcm_RN_out(r, c), dcm_RN_in(r, c), tol);
                }
            }
            for (int i = 0; i < 3; ++i) {
                EXPECT_NEAR(out.omega_RN_N(i), kOmegaRNN(i), tol);
                EXPECT_NEAR(out.domega_RN_N(i), kDomegaRNN(i), tol);
            }
        }
    }
}

// Body attitude exactly equal to the reference: the principal rotation is zero and the sensitive axes
// are parallel, so no maneuver is needed and the adjusted reference passes through (and stays finite).
TEST(SunAvoidanceTest, EdgeBodyAtReferencePassThrough) {
    const auto config = SunAvoidanceConfig::create(kSensitiveHat_B, kManeuverRate, true);
    SunAvoidanceAlgorithm alg{config};
    const SunAvoidanceAttRefInputs refIn{kSigmaRN, kOmegaRNN, kDomegaRNN};
    const Eigen::Matrix3f dcm_RN_in = mrpToDcm(kSigmaRN);

    constexpr float tol = 1e-5F;
    for (int k = 0; k < 5; ++k) {
        // sigma_BN == sigma_RN with the maneuver enabled and valid Sun geometry.
        const SunAvoidanceOutput out =
            alg.update(kSigmaRN, refIn, kRBN_N, kRSN_N, static_cast<uint64_t>(k) * kHalfSecNs);
        EXPECT_TRUE(out.sigma_RN.allFinite());
        const Eigen::Matrix3f dcm_RN_out = mrpToDcm(out.sigma_RN);
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                EXPECT_NEAR(dcm_RN_out(r, c), dcm_RN_in(r, c), tol);
            }
        }
    }
}

// Degenerate avoidance geometry: the Sun aligned with the initial sensitive axis, or perpendicular to
// the sweep plane. The maneuver stays finite and bounded (falls through to the short way).
TEST(SunAvoidanceTest, EdgeDegenerateAvoidanceGeometryBoundedAndFinite) {
    const Eigen::Vector3f sensitiveInitial_N = mrpToDcm(kSigmaBN).transpose() * kSensitiveHat_B;
    const Eigen::Vector3f sensitiveFinal_N = mrpToDcm(kSigmaRN).transpose() * kSensitiveHat_B;
    const Eigen::Vector3f sweepAxis_N = sensitiveInitial_N.cross(sensitiveFinal_N).normalized();

    // r_BN_N = 0 makes the Sun direction equal to the (unit) Sun position.
    const std::array<Eigen::Vector3d, 2> sunPositions{{
        sensitiveInitial_N.cast<double>(),  // Sun along the initial sensitive axis
        sweepAxis_N.cast<double>(),         // Sun perpendicular to the sweep plane
    }};

    const auto config = SunAvoidanceConfig::create(kSensitiveHat_B, kManeuverRate, true);
    const SunAvoidanceAttRefInputs refIn{kSigmaRN, kOmegaRNN, kDomegaRNN};
    constexpr float normBound = 1.0F + 1e-5F;
    for (const auto& r_SN_N : sunPositions) {
        SunAvoidanceAlgorithm alg{config};
        for (int k = 0; k < 10; ++k) {
            const SunAvoidanceOutput out =
                alg.update(kSigmaBN, refIn, Eigen::Vector3d::Zero(), r_SN_N, static_cast<uint64_t>(k) * kHalfSecNs);
            EXPECT_TRUE(out.sigma_RN.allFinite());
            EXPECT_TRUE(out.omega_RN_N.allFinite());
            EXPECT_LE(out.sigma_RN.norm(), normBound);
        }
    }
}

// Initial and final sensitive axes exactly anti-parallel (a 180-degree flip of the sensitive axis): the
// sweep axis is undefined (cross product of anti-parallel vectors is zero), so the avoidance test is
// skipped and the short-way maneuver stays finite and bounded.
TEST(SunAvoidanceTest, EdgeAntiParallelSensitiveAxes) {
    const Eigen::Vector3f sigmaBN = Eigen::Vector3f::Zero();  // identity attitude
    const Eigen::Vector3f sigmaRN{1.0F, 0.0F, 0.0F};          // 180 deg about X: flips the y sensitive axis
    const auto config = SunAvoidanceConfig::create(kSensitiveHat_B, kManeuverRate, true);
    SunAvoidanceAlgorithm alg{config};
    const SunAvoidanceAttRefInputs refIn{sigmaRN, kOmegaRNN, kDomegaRNN};

    constexpr float normBound = 1.0F + 1e-5F;
    for (int k = 0; k < 10; ++k) {
        const SunAvoidanceOutput out =
            alg.update(sigmaBN, refIn, kRBN_N, kRSN_N, static_cast<uint64_t>(k) * kHalfSecNs);
        EXPECT_TRUE(out.sigma_RN.allFinite());
        EXPECT_TRUE(out.omega_RN_N.allFinite());
        EXPECT_LE(out.sigma_RN.norm(), normBound);
    }
}

TEST(SunAvoidanceConfigTest, GettersRoundTrip) {
    // sensitiveHat_B is renormalized on storage; a near-unit input (within the 1e-3 tolerance) must come
    // back as the exact unit direction.
    const Eigen::Vector3f rawSensitive{0.0F, -1.0005F, 0.0F};
    const auto config = SunAvoidanceConfig::create(rawSensitive, kManeuverRate, true);

    constexpr float tol = 1e-6F;
    const Eigen::Vector3f expectedSensitive = rawSensitive.normalized();
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(config.getSensitiveHat_B()(i), expectedSensitive(i), tol);
    }
    EXPECT_NEAR(config.getAngleRate(), kManeuverRate, tol);
    EXPECT_TRUE(config.getComputeAngleStart());

    const auto configNoManeuver = SunAvoidanceConfig::create(kSensitiveHat_B, 0.0F, false);
    EXPECT_FALSE(configNoManeuver.getComputeAngleStart());
}
