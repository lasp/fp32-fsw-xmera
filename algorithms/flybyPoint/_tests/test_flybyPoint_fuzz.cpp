#include "flybyPointTestHelpers.hpp"
#include "utilities/testUtilities/eigenFuzzDomains.hpp"
#include <fuzztest/fuzztest.h>
#include <cmath>
#include <limits>
#include <numbers>
#include <utility>
#include <vector>

namespace {
/*! One filter sample per control period: {r_x, r_y, r_z, v_x, v_y, v_z}. */
using SampleSequence = std::vector<std::vector<double>>;

/*! Random valid configuration. */
auto configDomain() {
    return fuzztest::Map(
        [](double controlPeriod,
           uint32_t filterReadPeriods,
           float toleranceForCollinearity,
           int signOfOrbitNormalFrameVector,
           float maximumRateThreshold,
           float maximumAccelerationThreshold,
           float positionKnowledgeSigma) {
            return FlybyPointConfig::create(controlPeriod,
                                            filterReadPeriods,
                                            toleranceForCollinearity,
                                            signOfOrbitNormalFrameVector,
                                            maximumRateThreshold,
                                            maximumAccelerationThreshold,
                                            positionKnowledgeSigma);
        },
        fuzztest::InRange(1e-3, 1.0),           // controlPeriod [s]
        fuzztest::InRange<uint32_t>(1U, 200U),  // filterReadPeriods [-]
        fuzztest::InRange(1e-6F, 1e6F),         // toleranceForCollinearity [-]
        fuzztest::ElementOf<int>({-1, 1}),      // signOfOrbitNormalFrameVector
        fuzztest::InRange(1e-6F, 1e6F),         // maximumRateThreshold [deg/s]
        fuzztest::InRange(1e-6F, 1e6F),         // maximumAccelerationThreshold [deg/s^2]
        fuzztest::InRange(1e-6F, 1e6F));        // positionKnowledgeSigma [m]
}

/*! Each component is drawn from a realistic range, the full finite range (overflow), any double including NaN and
 inf, or exact zero. */
auto sampleComponentDomain() {
    return fuzztest::OneOf(
        fuzztest::InRange(-1e14, 1e14), fuzztest::Finite<double>(), fuzztest::Arbitrary<double>(), fuzztest::Just(0.0));
}

/*! Values spread evenly over the orders of magnitude from lo to hi. */
auto logRange(double lo, double hi) {
    return fuzztest::Map([](double exponent) { return std::pow(10.0, exponent); },
                         fuzztest::InRange(std::log10(lo), std::log10(hi)));
}

auto sampleSequenceDomain() {
    return fuzztest::VectorOf(fuzztest::VectorOf(sampleComponentDomain()).WithSize(6)).WithMinSize(1).WithMaxSize(150);
}
}  // namespace

// For any configuration and a noisy straight-line trajectory, the algorithm matches the test's double-precision copy of
// it on every period, through accepted and rejected re-reads. A (nearly) collinear first sample is skipped: whether it
// is refused can depend on rounding, and the property test below covers it.
static void fuzzMatchesReferenceModel(const FlybyPointConfig& cfg,
                                      const Eigen::Vector3d& r0_BN_N,
                                      const Eigen::Vector3d& v_BN_N,
                                      const std::vector<Eigen::Vector3d>& noise_N) {
    if (r0_BN_N.stableNormalized().cross(v_BN_N.stableNormalized()).stableNorm() < 1e-6) {
        return;
    }
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> samples{{r0_BN_N, v_BN_N}};
    for (size_t k = 1; k <= noise_N.size(); ++k) {
        const double t = static_cast<double>(k) * cfg.getControlPeriod();
        samples.emplace_back(r0_BN_N + t * v_BN_N + noise_N[k - 1], v_BN_N);
    }
    expectMatchesReferenceModel(cfg, samples);
}

FUZZ_TEST(FlybyPointAlgorithmFuzz, fuzzMatchesReferenceModel)
    .WithDomains(configDomain(),
                 xmera::fuzz::Vector3dInRange(-1e14, 1e14),  // r0_BN_N [m]
                 xmera::fuzz::Vector3dInRange(-1e14, 1e14),  // v_BN_N [m/s]
                 fuzztest::VectorOf(xmera::fuzz::Vector3dInRange(-1e3, 1e3)).WithMinSize(1).WithMaxSize(120));  // [m]

// Every case turns the algorithm on some time before closest approach of a straight-line flyby and runs it at its
// control period until some time after, with re-reads along the way. On every period the reference matches the exact
// one from the geometry. No check rejects under these limits, so the first sample seeds and every re-read is accepted.
static void fuzzFlybyPassMatchesStraightLine(double closestApproachDistance,
                                             double speed,
                                             double timeBeforeClosestApproach,
                                             double timeAfterClosestApproach,
                                             double controlPeriod,
                                             uint32_t filterReadPeriods,
                                             const Eigen::Vector3d& velocityDirection_N,
                                             const Eigen::Vector3d& offsetDirection_N,
                                             int signOfOrbitNormalFrameVector) {
    // The closest-approach point is perpendicular to the velocity; skip directions too short to define it.
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

    // Skip runs too long to keep the fuzzer fast.
    constexpr double kMaxPeriods = 1e5;
    const double numPeriods = std::floor((timeBeforeClosestApproach + timeAfterClosestApproach) / controlPeriod);
    if (numPeriods > kMaxPeriods) {
        return;
    }
    const auto timeAt = [&](uint32_t k) { return static_cast<double>(k) * controlPeriod - timeBeforeClosestApproach; };
    const auto lastPeriod = static_cast<uint32_t>(numPeriods);

    constexpr float kCollinearityTolerance = 1e-9F;
    constexpr float kNoLimit = std::numeric_limits<float>::max();
    const FlybyPointConfig cfg = FlybyPointConfig::create(controlPeriod,
                                                          filterReadPeriods,
                                                          kCollinearityTolerance,
                                                          signOfOrbitNormalFrameVector,
                                                          kNoLimit,
                                                          kNoLimit,
                                                          kNoLimit);

    // Skip runs that start or end almost head-on, where the collinearity check refuses the sample. The run is closest
    // to head-on at its two ends, so every sample in between is accepted.
    if (referenceIsCollinear(rClosest_BN_N + timeAt(0) * v_BN_N, v_BN_N, cfg) ||
        referenceIsCollinear(rClosest_BN_N + timeAt(lastPeriod) * v_BN_N, v_BN_N, cfg)) {
        return;
    }

    const double speedOverDistance = speed / closestApproachDistance;
    const double peakAcceleration = 3.0 * std::numbers::sqrt3 / 8.0 * speedOverDistance * speedOverDistance;
    FlybyPointAlgorithm alg(cfg);
    for (uint32_t k = 0; k <= lastPeriod; ++k) {
        SCOPED_TRACE("period " + std::to_string(k));
        const double t = timeAt(k);
        const AttGuideOutput out = alg.updateState(rClosest_BN_N + t * v_BN_N, v_BN_N);
        expectFlags(out);
        expectReference(
            out, straightLineReference(rClosest_BN_N, v_BN_N, t, signOfOrbitNormalFrameVector), peakAcceleration);
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

// For any configuration and any samples (realistic, extreme, non-finite or zero), every period: nothing throws or
// aborts, the reference is finite with |sigma| <= 1, it is exactly zero until a usable, non-collinear sample seeds,
// unusable samples are flagged, and no more samples are reported unusable than the window holds.
static void fuzzOutputProperties(const FlybyPointConfig& cfg, const SampleSequence& samples) {
    FlybyPointAlgorithm alg(cfg);
    bool seeded = false;
    for (const std::vector<double>& sample : samples) {
        const Eigen::Vector3d r{sample[0], sample[1], sample[2]};
        const Eigen::Vector3d v{sample[3], sample[4], sample[5]};
        AttGuideOutput out{};
        EXPECT_NO_THROW(out = alg.updateState(r, v));

        EXPECT_TRUE(out.sigma_RN.allFinite() && out.omega_RN_N.allFinite() && out.domega_RN_N.allFinite());
        EXPECT_LE(out.sigma_RN.stableNorm(), 1.0F + 1e-6F);
        seeded = seeded || (referenceIsUsableSample(r, v) && !referenceIsCollinear(r, v, cfg));
        if (!seeded) {
            expectZeroGuidance(out);
        }
        EXPECT_EQ(out.inputSampleRejected, !referenceIsUsableSample(r, v));
        EXPECT_LE(out.rejectedSamplesInWindow, cfg.getFilterReadPeriods());
    }
}

FUZZ_TEST(FlybyPointPropertyFuzz, fuzzOutputProperties).WithDomains(configDomain(), sampleSequenceDomain());
