#include "flybyPointTestHelpers.hpp"
#include "utilities/testUtilities/eigenFuzzDomains.hpp"
#include <fuzztest/fuzztest.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <optional>
#include <utility>
#include <vector>

namespace {
/*! One filter sample per control period: {r_x, r_y, r_z, v_x, v_y, v_z}. */
using SampleSequence = std::vector<std::vector<float>>;

/*! Per-sample filter noise: {position [m], velocity [m/s]}. */
using StateNoise = std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>>;

/*! Random configuration; every parameter is valid except possibly the ground state. */
auto configDomain() {
    return fuzztest::StructOf<ConfigParams>(
        fuzztest::InRange(1e-3F, 1.0F),               // controlPeriod [s]
        fuzztest::InRange<uint32_t>(1U, 200U),        // filterReadPeriods [-]
        fuzztest::InRange(1e-6F, 0.5F),               // toleranceForCollinearity [-]
        fuzztest::ElementOf<int>({-1, 1}),            // signOfOrbitNormalFrameVector
        fuzztest::InRange(1e-6F, 1e6F),               // maximumRateThreshold [deg/s]
        fuzztest::InRange(1e-6F, 1e6F),               // maximumAccelerationThreshold [deg/s^2]
        fuzztest::InRange(0.1F, 10.0F),               // numberOfSigmas [-]
        xmera::fuzz::Vector3fInRange(-1e9F, 1e9F),    // groundPosition_BN_N [m]
        xmera::fuzz::Vector3fInRange(-1e9F, 1e9F),    // groundVelocity_BN_N [m/s]
        xmera::fuzz::Vector3fInRange(1e-6F, 1e12F),   // groundPositionCovarianceDiag_N [m^2]
        xmera::fuzz::Vector3fInRange(1e-6F, 1e12F));  // groundVelocityCovarianceDiag_N [(m/s)^2]
}

/*! Any configuration parameters, valid or not: non-finite, zero, negative and out-of-range values included. */
auto anyConfigDomain() {
    const auto component = [] {
        return fuzztest::OneOf(fuzztest::InRange(-1e6F, 1e6F), fuzztest::Arbitrary<float>(), fuzztest::Just(0.0F));
    };
    return fuzztest::StructOf<ConfigParams>(
        fuzztest::Arbitrary<float>(),                        // controlPeriod
        fuzztest::Arbitrary<uint32_t>(),                     // filterReadPeriods
        fuzztest::Arbitrary<float>(),                        // toleranceForCollinearity
        fuzztest::ElementOf<int>({-2, -1, 0, 1, 2}),         // signOfOrbitNormalFrameVector
        fuzztest::Arbitrary<float>(),                        // maximumRateThreshold
        fuzztest::Arbitrary<float>(),                        // maximumAccelerationThreshold
        fuzztest::Arbitrary<float>(),                        // numberOfSigmas
        xmera::fuzz::EigenVectorOf<float, 3>(component()),   // groundPosition_BN_N
        xmera::fuzz::EigenVectorOf<float, 3>(component()),   // groundVelocity_BN_N
        xmera::fuzz::EigenVectorOf<float, 3>(component()),   // position variances
        xmera::fuzz::EigenVectorOf<float, 3>(component()));  // velocity variances
}

/*! Sample component: realistic, any finite, any float (NaN, inf), or zero. */
auto sampleComponentDomain() {
    return fuzztest::OneOf(fuzztest::InRange(-1e14F, 1e14F),
                           fuzztest::Finite<float>(),
                           fuzztest::Arbitrary<float>(),
                           fuzztest::Just(0.0F));
}

auto sampleSequenceDomain() {
    return fuzztest::VectorOf(fuzztest::VectorOf(sampleComponentDomain()).WithSize(6)).WithMinSize(1).WithMaxSize(150);
}

/*! Values spread evenly over the orders of magnitude from lo to hi. */
auto logRange(double lo, double hi) {
    return fuzztest::Map([](double exponent) { return std::pow(10.0, exponent); },
                         fuzztest::InRange(std::log10(lo), std::log10(hi)));
}

/*! The configuration, or std::nullopt after expecting create() to throw for an invalid ground state. */
std::optional<FlybyPointConfig> makeConfigOrExpectThrow(const ConfigParams& p) {
    if (!FlybyPointConfig::isValidGroundState(
            p.groundPosition_BN_N, p.groundVelocity_BN_N, p.toleranceForCollinearity)) {
        EXPECT_ANY_THROW((void)makeConfig(p));
        return std::nullopt;
    }
    return makeConfig(p);
}
}  // namespace

// create() throws exactly when a validator rejects a parameter, and otherwise stores every parameter.
static void fuzzConfigCreateMatchesValidators(const ConfigParams& p) {
    const bool valid = FlybyPointConfig::isValidControlPeriod(p.controlPeriod) &&
                       FlybyPointConfig::isValidFilterReadPeriods(p.filterReadPeriods) &&
                       FlybyPointConfig::isValidToleranceForCollinearity(p.toleranceForCollinearity) &&
                       FlybyPointConfig::isValidSignOfOrbitNormalFrameVector(p.signOfOrbitNormalFrameVector) &&
                       FlybyPointConfig::isValidMaximumRateThreshold(p.maximumRateThreshold) &&
                       FlybyPointConfig::isValidMaximumAccelerationThreshold(p.maximumAccelerationThreshold) &&
                       FlybyPointConfig::isValidNumberOfSigmas(p.numberOfSigmas) &&
                       FlybyPointConfig::isValidGroundState(
                           p.groundPosition_BN_N, p.groundVelocity_BN_N, p.toleranceForCollinearity) &&
                       FlybyPointConfig::isValidGroundPositionCovarianceDiag(p.groundPositionCovarianceDiag_N) &&
                       FlybyPointConfig::isValidGroundVelocityCovarianceDiag(p.groundVelocityCovarianceDiag_N);
    if (!valid) {
        EXPECT_ANY_THROW((void)makeConfig(p));
        return;
    }

    std::optional<FlybyPointConfig> cfg;
    EXPECT_NO_THROW(cfg.emplace(makeConfig(p)));
    if (!cfg) {
        return;
    }
    EXPECT_EQ(cfg->getControlPeriod(), p.controlPeriod);
    EXPECT_EQ(cfg->getFilterReadPeriods(), p.filterReadPeriods);
    EXPECT_EQ(cfg->getToleranceForCollinearity(), p.toleranceForCollinearity);
    EXPECT_EQ(cfg->getSignOfOrbitNormalFrameVector(), p.signOfOrbitNormalFrameVector);
    EXPECT_EQ(cfg->getMaximumRateThreshold(), p.maximumRateThreshold);
    EXPECT_EQ(cfg->getMaximumAccelerationThreshold(), p.maximumAccelerationThreshold);
    EXPECT_EQ(cfg->getNumberOfSigmas(), p.numberOfSigmas);
    EXPECT_EQ(cfg->getGroundPosition_BN_N(), p.groundPosition_BN_N);
    EXPECT_EQ(cfg->getGroundVelocity_BN_N(), p.groundVelocity_BN_N);
    EXPECT_EQ(cfg->getGroundPositionCovarianceDiag_N(), p.groundPositionCovarianceDiag_N);
    EXPECT_EQ(cfg->getGroundVelocityCovarianceDiag_N(), p.groundVelocityCovarianceDiag_N);
}

FUZZ_TEST(FlybyPointConfigFuzz, fuzzConfigCreateMatchesValidators).WithDomains(anyConfigDomain());

// The algorithm matches the reference model on a noisy straight line with a ground state error.
static void fuzzMatchesReferenceModel(ConfigParams p,
                                      const Eigen::Vector3f& r0_BN_N,
                                      const Eigen::Vector3f& v_BN_N,
                                      const Eigen::Vector3f& groundPositionError_N,
                                      const Eigen::Vector3f& groundVelocityError_N,
                                      const StateNoise& noise) {
    p.groundPosition_BN_N = r0_BN_N + groundPositionError_N;
    p.groundVelocity_BN_N = v_BN_N + groundVelocityError_N;
    const std::optional<FlybyPointConfig> cfg = makeConfigOrExpectThrow(p);
    if (!cfg) {
        return;
    }
    std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> samples;
    for (size_t k = 0; k < noise.size(); ++k) {
        const float t = static_cast<float>(k) * cfg->getControlPeriod();
        const auto& [positionNoise_N, velocityNoise_N] = noise[k];
        samples.emplace_back(r0_BN_N + t * v_BN_N + positionNoise_N, v_BN_N + velocityNoise_N);
    }
    expectMatchesReferenceModel(*cfg, samples);
}

FUZZ_TEST(FlybyPointAlgorithmFuzz, fuzzMatchesReferenceModel)
    .WithDomains(configDomain(),
                 xmera::fuzz::Vector3fInRange(-1e14F, 1e14F),  // r0_BN_N [m]
                 xmera::fuzz::Vector3fInRange(-1e14F, 1e14F),  // v_BN_N [m/s]
                 xmera::fuzz::Vector3fInRange(-1e4F, 1e4F),    // groundPositionError_N [m]
                 xmera::fuzz::Vector3fInRange(-10.0F, 10.0F),  // groundVelocityError_N [m/s]
                 fuzztest::VectorOf(fuzztest::PairOf(xmera::fuzz::Vector3fInRange(-1e3F, 1e3F),     // [m]
                                                     xmera::fuzz::Vector3fInRange(-10.0F, 10.0F)))  // [m/s]
                     .WithMinSize(1)
                     .WithMaxSize(120));

// The output matches the exact straight-line flyby, and the live state check accepts every re-read. The tolerances
// are a float floor plus a conditioning term for the run's most head-on end: the frame error grows as eps / s and the
// rate error as eps / s^2, with s the sine of the r-v angle.
static void fuzzFlybyPassMatchesStraightLine(double closestApproachDistance,
                                             double speed,
                                             double timeBeforeClosestApproach,
                                             double timeAfterClosestApproach,
                                             double controlPeriod,
                                             uint32_t filterReadPeriods,
                                             const Eigen::Vector3d& velocityDirection_N,
                                             const Eigen::Vector3d& offsetDirection_N,
                                             int signOfOrbitNormalFrameVector) {
    // Skip directions too short to define the closest-approach point.
    if (velocityDirection_N.stableNorm() < 0.1) {
        return;
    }
    const Eigen::Vector3d uv_N = velocityDirection_N.stableNormalized();
    const Eigen::Vector3d offset_N = offsetDirection_N - offsetDirection_N.dot(uv_N) * uv_N;
    if (offset_N.stableNorm() < 0.1) {
        return;
    }
    const Eigen::Vector3d rClosest_BN_N = closestApproachDistance * offset_N.stableNormalized();
    const Eigen::Vector3d v_BN_N = speed * uv_N;

    // Skip runs too long for the fuzzer.
    constexpr double kMaxPeriods = 1e5;
    const double numPeriods = std::floor((timeBeforeClosestApproach + timeAfterClosestApproach) / controlPeriod);
    if (numPeriods > kMaxPeriods) {
        return;
    }
    const auto timeAt = [&](uint32_t k) { return static_cast<double>(k) * controlPeriod - timeBeforeClosestApproach; };
    const auto lastPeriod = static_cast<uint32_t>(numPeriods);

    // Skip runs that start or end almost head-on (collinear in float).
    constexpr float kCollinearityTolerance = 1e-9F;
    const Eigen::Vector3d rStart_BN_N = rClosest_BN_N + timeAt(0) * v_BN_N;
    const Eigen::Vector3d rEnd_BN_N = rClosest_BN_N + timeAt(lastPeriod) * v_BN_N;
    const Eigen::Vector3f v_BN_N_f = v_BN_N.cast<float>();
    if (referenceIsCollinear(rStart_BN_N.cast<float>(), v_BN_N_f, kCollinearityTolerance) ||
        referenceIsCollinear(rEnd_BN_N.cast<float>(), v_BN_N_f, kCollinearityTolerance)) {
        return;
    }

    // Position sigma relative to |r|, well above the float position resolution (~1e-7 |r|).
    constexpr float kNoLimit = std::numeric_limits<float>::max();
    constexpr float kNumberOfSigmas = 3.0F;
    const auto positionSigma = static_cast<float>(1e-5 * rStart_BN_N.stableNorm());
    const FlybyPointConfig cfg = FlybyPointConfig::create(static_cast<float>(controlPeriod),
                                                          filterReadPeriods,
                                                          kCollinearityTolerance,
                                                          signOfOrbitNormalFrameVector,
                                                          kNoLimit,
                                                          kNoLimit,
                                                          kNumberOfSigmas,
                                                          rStart_BN_N.cast<float>(),
                                                          v_BN_N_f,
                                                          Eigen::Vector3f::Constant(positionSigma * positionSigma),
                                                          Eigen::Vector3f::Constant(1.0F));

    const double sineMin =
        std::min(closestApproachDistance / rStart_BN_N.stableNorm(), closestApproachDistance / rEnd_BN_N.stableNorm());
    const double dcmTolScale = 3.0 + 10.0 / sineMin;  // ~170 eps / s; worst measured ~37 eps / s at s ~ 8e-5
    const double rateTolScale = std::max(1.0, 0.05 / (sineMin * sineMin));
    const double speedOverDistance = speed / closestApproachDistance;
    const double peakAcceleration = 3.0 * std::numbers::sqrt3 / 8.0 * speedOverDistance * speedOverDistance;
    FlybyPointAlgorithm alg(cfg);
    for (uint32_t k = 0; k <= lastPeriod; ++k) {
        SCOPED_TRACE("period " + std::to_string(k));
        const double t = timeAt(k);
        const AttGuideOutput out = alg.updateState((rClosest_BN_N + t * v_BN_N).cast<float>(), v_BN_N_f);
        expectFlags(out);
        expectReference(out,
                        exactStraightLineReference(rClosest_BN_N, v_BN_N, t, signOfOrbitNormalFrameVector),
                        peakAcceleration,
                        dcmTolScale,
                        rateTolScale);
    }
}

FUZZ_TEST(FlybyPointAlgorithmFuzz, fuzzFlybyPassMatchesStraightLine)
    .WithDomains(logRange(1.0, 1e9),                       // closestApproachDistance [m]
                 logRange(1.0, 1e6),                       // speed [m/s]
                 logRange(600.0, 3600.0),                  // timeBeforeClosestApproach [s]: 10 min to 2 h
                 logRange(600.0, 3600.0),                  // timeAfterClosestApproach [s]
                 logRange(0.01, 1.0),                      // controlPeriod [s]
                 fuzztest::InRange<uint32_t>(1U, 200U),    // filterReadPeriods [-]
                 xmera::fuzz::Vector3dInRange(-1.0, 1.0),  // velocityDirection_N
                 xmera::fuzz::Vector3dInRange(-1.0, 1.0),  // offsetDirection_N
                 fuzztest::ElementOf<int>({-1, 1}));       // signOfOrbitNormalFrameVector

// Any inputs: no throw, finite reference with |sigma| <= 1, and flags and counts only at window ends.
static void fuzzOutputProperties(const ConfigParams& p, const SampleSequence& samples) {
    const std::optional<FlybyPointConfig> cfg = makeConfigOrExpectThrow(p);
    if (!cfg) {
        return;
    }
    FlybyPointAlgorithm alg(*cfg);
    const uint64_t windowLength = cfg->getFilterReadPeriods();
    uint32_t unusableInWindow = 0U;
    for (size_t k = 0; k < samples.size(); ++k) {
        SCOPED_TRACE("period " + std::to_string(k));
        const Eigen::Vector3f r{samples[k][0], samples[k][1], samples[k][2]};
        const Eigen::Vector3f v{samples[k][3], samples[k][4], samples[k][5]};
        AttGuideOutput out{};
        EXPECT_NO_THROW(out = alg.updateState(r, v));

        EXPECT_TRUE(out.sigma_RN.allFinite() && out.omega_RN_N.allFinite() && out.domega_RN_N.allFinite());
        EXPECT_LE(out.sigma_RN.stableNorm(), 1.0F + 1e-6F);
        const bool usable = referenceIsUsableSample(r, v);
        EXPECT_EQ(out.inputSampleRejected, !usable);
        if (!usable) {
            ++unusableInWindow;
        }

        if ((static_cast<uint64_t>(k) + 1U) % windowLength == 0U) {
            EXPECT_EQ(out.rejectedSamplesInWindow, unusableInWindow);
            unusableInWindow = 0U;
        } else {
            expectFlags(out, {.inputSampleRejected = !usable});
        }
    }
}

FUZZ_TEST(FlybyPointPropertyFuzz, fuzzOutputProperties).WithDomains(configDomain(), sampleSequenceDomain());
