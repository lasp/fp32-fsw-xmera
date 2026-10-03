#ifndef TEST_FLYBY_POINT_HELPERS_H
#define TEST_FLYBY_POINT_HELPERS_H

#include "flybyPointAlgorithm.h"
#include "utilities/fsw/rigidBodyKinematics.hpp"
#include "utilities/fsw/safeMath.h"

#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <numbers>
#include <string>
#include <utility>
#include <vector>

// ---------------------------------------------------------------------------
// Test scenario: a body passing the spacecraft on a straight line, and test configurations
// ---------------------------------------------------------------------------
const Eigen::Vector3d kR0{-5e7, 7.5e6, 5e5};       // [m] position at the first sample
const Eigen::Vector3d kV{2e4, 0, 0};               // [m/s] constant velocity
const Eigen::Vector3d kShift{0, 0, 1e4};           // [m] shift that moves the frame visibly but passes every check
const Eigen::Vector3d kPositionOffset{0, 0, 2e5};  // [m] offset that fails the position check of the tests using it

/*! Position on the test trajectory t seconds after the first sample. */
inline Eigen::Vector3d truthAt(double t) { return kR0 + t * kV; }

/*! FlybyPointConfig::create() arguments with defaults under which the test trajectory passes every check. */
struct ConfigParams {
    double controlPeriod = 1.0;
    uint32_t filterReadPeriods = 1U;
    float toleranceForCollinearity = 1e-3F;
    int signOfOrbitNormalFrameVector = 1;
    float maximumRateThreshold = 10.0F;
    float maximumAccelerationThreshold = 1.0F;
    float positionKnowledgeSigma = 1e9F;
};

inline FlybyPointConfig makeConfig(const ConfigParams& p = {}) {
    return FlybyPointConfig::create(p.controlPeriod,
                                    p.filterReadPeriods,
                                    p.toleranceForCollinearity,
                                    p.signOfOrbitNormalFrameVector,
                                    p.maximumRateThreshold,
                                    p.maximumAccelerationThreshold,
                                    p.positionKnowledgeSigma);
}

// ---------------------------------------------------------------------------
// Expected outputs and comparisons
// ---------------------------------------------------------------------------
/*! Reference frame (as an inertial-to-reference DCM), rate and acceleration, in double precision. */
struct ReferenceFlybyOutput {
    Eigen::Matrix3d RN;
    Eigen::Vector3d omega_RN_N;
    Eigen::Vector3d domega_RN_N;
};

/*! The zero reference output when no guidance solution is available (zero MRP = identity DCM). */
inline ReferenceFlybyOutput zeroReference() {
    return {Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
}

/*! Exact reference t seconds after a body at r0 moving with constant velocity v, from the geometry alone: the frame
 axes point along r, along the in-plane direction ahead of r, and along the orbit normal h = r x v (the last two
 negated for sign -1). It turns about h at |h| / |r|^2, and the rate changes at -2 (r.v) |h| / |r|^4. h is constant on
 a straight line, so it is taken at r0, where it is best conditioned.
 @return the expected reference
 @param r0 [m] position at time 0
 @param v [m/s] constant velocity
 @param t [s] time since r0
 @param sign signOfOrbitNormalFrameVector
 */
inline ReferenceFlybyOutput straightLineReference(const Eigen::Vector3d& r0,
                                                  const Eigen::Vector3d& v,
                                                  double t = 0.0,
                                                  int sign = 1) {
    const Eigen::Vector3d h = r0.cross(v);
    const Eigen::Vector3d r = r0 + t * v;
    const Eigen::Vector3d ur = r.stableNormalized();
    const Eigen::Vector3d uh = h.stableNormalized();
    const double s = sign;
    Eigen::Matrix3d RN;
    RN.row(0) = ur;
    RN.row(1) = s * uh.cross(ur);
    RN.row(2) = s * uh;
    const double r2 = r.squaredNorm();
    const double thetaDot = h.stableNorm() / r2;
    const double thetaDDot = -2.0 * r.dot(v) * h.stableNorm() / (r2 * r2);
    return {RN, thetaDot * uh, thetaDDot * uh};
}

/*! The output matches the expected reference.
 The DCMs are compared, so either MRP of the same attitude is accepted. The float32 output (and the float32 DCM the
 algorithm stores at each read) carries about 1e-7 relative error: the DCM elements must agree within kDcmTol, and each
 rate and acceleration component within kRelTol of the expected vector's norm, which keeps a 20x margin. kTinyFloor
 only matters for values in the float32 subnormal range.
 @param domegaScale [rad/s^2] typical acceleration size of the scenario. The acceleration passes through zero at closest
 approach, where a tolerance relative to the expected value alone would fail on rounding; the larger of the two is used
 */
inline void expectReference(const AttGuideOutput& out, const ReferenceFlybyOutput& expected, double domegaScale = 0.0) {
    static constexpr double kDcmTol = 1e-6;
    static constexpr double kRelTol = 1e-5;
    static constexpr double kTinyFloor = 1e-30;

    const Eigen::Matrix3d RN = mrpToDcm(Eigen::Vector3d(out.sigma_RN.cast<double>()));
    EXPECT_LT((RN - expected.RN).cwiseAbs().maxCoeff(), kDcmTol) << "RN\n" << RN << "\nexpected\n" << expected.RN;
    const Eigen::Vector3d omegaError = out.omega_RN_N.cast<double>() - expected.omega_RN_N;
    EXPECT_LE(omegaError.cwiseAbs().maxCoeff(), kRelTol * expected.omega_RN_N.stableNorm() + kTinyFloor)
        << "omega " << out.omega_RN_N.transpose() << ", expected " << expected.omega_RN_N.transpose();
    const Eigen::Vector3d domegaError = out.domega_RN_N.cast<double>() - expected.domega_RN_N;
    EXPECT_LE(domegaError.cwiseAbs().maxCoeff(),
              kRelTol * std::max(expected.domega_RN_N.stableNorm(), domegaScale) + kTinyFloor)
        << "domega " << out.domega_RN_N.transpose() << ", expected " << expected.domega_RN_N.transpose();
}

/*! No guidance solution: attitude, rate and acceleration all exactly zero. */
inline void expectZeroGuidance(const AttGuideOutput& out) {
    EXPECT_TRUE(out.sigma_RN.isZero(0.0F)) << out.sigma_RN.transpose();
    EXPECT_TRUE(out.omega_RN_N.isZero(0.0F)) << out.omega_RN_N.transpose();
    EXPECT_TRUE(out.domega_RN_N.isZero(0.0F)) << out.domega_RN_N.transpose();
}

/*! Expected diagnostic flags of one period; the defaults are a clean period. */
struct ExpectedFlags {
    bool collinearity = false;
    bool maxRate = false;
    bool maxAcceleration = false;
    bool positionKnowledge = false;
    bool inputSampleRejected = false;
    uint32_t rejectedSamplesInWindow = 0U;
};

inline void expectFlags(const AttGuideOutput& out, const ExpectedFlags& expected = {}) {
    EXPECT_EQ(out.collinearityTrigger, expected.collinearity);
    EXPECT_EQ(out.maxRateTrigger, expected.maxRate);
    EXPECT_EQ(out.maxAccelerationTrigger, expected.maxAcceleration);
    EXPECT_EQ(out.positionKnowledgeExceedTrigger, expected.positionKnowledge);
    EXPECT_EQ(out.inputSampleRejected, expected.inputSampleRejected);
    EXPECT_EQ(out.rejectedSamplesInWindow, expected.rejectedSamplesInWindow);
}

// ---------------------------------------------------------------------------
// Reference model: a double-precision copy of the algorithm, for regression tests. It deliberately uses the same
// formulas as the algorithm; the formulas themselves are checked against straightLineReference().
// ---------------------------------------------------------------------------
struct ReferenceFlybyState {
    bool seeded = false;
    uint64_t periodsSinceRead = 0;
    uint32_t windowPeriods = 0;
    uint32_t windowSamples = 0;
    Eigen::Vector3d rSumAtWindowEnd = Eigen::Vector3d::Zero();
    Eigen::Vector3d vSum = Eigen::Vector3d::Zero();
    Eigen::Vector3d readPosition = Eigen::Vector3d::Zero();  // last accepted read
    Eigen::Vector3d readVelocity = Eigen::Vector3d::Zero();
    double f0 = 0.0;
    double gamma0 = 0.0;
    Eigen::Matrix3d R0N = Eigen::Matrix3d::Identity();
};

/*! Same test as isUsableSample() in flybyPointAlgorithm.cpp. */
inline bool referenceIsUsableSample(const Eigen::Vector3d& r, const Eigen::Vector3d& v) {
    const double rNorm = r.stableNorm();
    const double vNorm = v.stableNorm();
    return r.allFinite() && v.allFinite() && rNorm >= 1e-3 && vNorm >= 1e-3 && std::isfinite(rNorm * vNorm) &&
           std::isfinite((vNorm / rNorm) * (vNorm / rNorm));
}

/*! Same test as isCollinear() in flybyPointAlgorithm.cpp. */
inline bool referenceIsCollinear(const Eigen::Vector3d& r, const Eigen::Vector3d& v, const FlybyPointConfig& config) {
    const Eigen::Vector3d ur = r.stableNormalized();
    const Eigen::Vector3d uv = v.stableNormalized();
    return 1.0 - std::fabs(ur.dot(uv)) < config.getToleranceForCollinearity() ||
           ur.cross(uv).stableNorm() < FlybyPointAlgorithm::kMinOrbitNormalNorm;
}

/*! Same checks as checkValidity() in flybyPointAlgorithm.cpp.
 @return true if the candidate (r, v) passes every check
 */
inline bool referencePassesChecks(const ReferenceFlybyState& s,
                                  const Eigen::Vector3d& r,
                                  const Eigen::Vector3d& v,
                                  const FlybyPointConfig& config) {
    static constexpr double kRad2Deg = 180.0 / std::numbers::pi;
    static constexpr double kMaxAccelCoeff = 3.0 * std::numbers::sqrt3 / 8.0;
    const double distanceClosestApproach = r.cross(v).stableNorm() / v.stableNorm();
    if (referenceIsCollinear(r, v, config) || !(distanceClosestApproach > 0.0)) {
        return false;
    }
    const double speedOverDistance = v.stableNorm() / distanceClosestApproach;
    const double deltaT = static_cast<double>(s.periodsSinceRead) * config.getControlPeriod();
    const Eigen::Vector3d rPredicted = s.readPosition + deltaT * s.readVelocity;
    return speedOverDistance * kRad2Deg <= config.getMaximumRateThreshold() &&
           kMaxAccelCoeff * speedOverDistance * speedOverDistance * kRad2Deg <=
               config.getMaximumAccelerationThreshold() &&
           (r - rPredicted).stableNorm() <= config.getPositionKnowledgeSigma();
}

/*! Same profile as seedProfile() in flybyPointAlgorithm.cpp, with the DCM kept in double. */
inline void referenceSeed(ReferenceFlybyState& s, const Eigen::Vector3d& r, const Eigen::Vector3d& v) {
    const Eigen::Vector3d ur = r.stableNormalized();
    const Eigen::Vector3d uv = v.stableNormalized();
    const Eigen::Vector3d uh = ur.cross(uv).stableNormalized();
    const Eigen::Vector3d ut = uh.cross(ur).stableNormalized();
    s.R0N.row(0) = ur;
    s.R0N.row(1) = ut;
    s.R0N.row(2) = uh;
    s.f0 = v.stableNorm() / r.stableNorm();
    s.gamma0 = safeAtan2(v.dot(ur), v.dot(ut));
    s.readPosition = r;
    s.readVelocity = v;
    s.periodsSinceRead = 0;
    s.seeded = true;
}

/*! Same solution as computeGuidanceReference() in flybyPointAlgorithm.cpp, including its zero fallbacks. */
inline ReferenceFlybyOutput referenceGuidanceSolution(const ReferenceFlybyState& s, const FlybyPointConfig& config) {
    const double f0 = s.f0;
    const double gamma0 = s.gamma0;
    const double dt = static_cast<double>(s.periodsSinceRead) * config.getControlPeriod();
    const double theta = safeAtan(safeTan(gamma0) + f0 / safeCos(gamma0) * dt) - gamma0;
    const double den = f0 * f0 * dt * dt + 2.0 * f0 * safeSin(gamma0) * dt + 1.0;
    const double thetaDot = f0 * safeCos(gamma0) / den;
    const double thetaDDot = -2.0 * f0 * f0 * safeCos(gamma0) * (f0 * dt + safeSin(gamma0)) / (den * den);
    if (!std::isfinite(f0) || !std::isfinite(gamma0) || !std::isfinite(theta) || !std::isfinite(thetaDot) ||
        !std::isfinite(thetaDDot)) {
        return zeroReference();
    }

    const Eigen::Matrix3d RtN = prvToDcm(Eigen::Vector3d{0, 0, theta}) * s.R0N;
    const Eigen::Vector3d omega = RtN.transpose() * Eigen::Vector3d{0, 0, thetaDot};
    const Eigen::Vector3d domega = RtN.transpose() * Eigen::Vector3d{0, 0, thetaDDot};
    if (!omega.cast<float>().allFinite() || !domega.cast<float>().allFinite()) {
        return zeroReference();
    }
    Eigen::Matrix3d RN = RtN;
    if (config.getSignOfOrbitNormalFrameVector() == -1) {
        RN.bottomRows<2>() *= -1.0;  // 180 deg turn about the first axis
    }
    return {RN, omega, domega};
}

/*! Advances the reference model by one control period, like FlybyPointAlgorithm::updateState(). */
inline ReferenceFlybyOutput referenceUpdateState(ReferenceFlybyState& s,
                                                 const Eigen::Vector3d& r,
                                                 const Eigen::Vector3d& v,
                                                 const FlybyPointConfig& config) {
    const uint32_t windowLength = config.getFilterReadPeriods();
    const bool usable = referenceIsUsableSample(r, v);
    if (!s.seeded) {
        if (!usable || referenceIsCollinear(r, v, config)) {
            return zeroReference();
        }
        referenceSeed(s, r, v);
        return referenceGuidanceSolution(s, config);
    }

    ++s.periodsSinceRead;
    ++s.windowPeriods;
    if (usable) {
        const double timeToWindowEnd = static_cast<double>(windowLength - s.windowPeriods) * config.getControlPeriod();
        s.rSumAtWindowEnd += r + timeToWindowEnd * v;
        s.vSum += v;
        ++s.windowSamples;
    }
    if (s.windowPeriods >= windowLength) {
        if (s.windowSamples > 0U) {
            const double n = static_cast<double>(s.windowSamples);
            const Eigen::Vector3d rAverage = s.rSumAtWindowEnd / n;
            const Eigen::Vector3d vAverage = s.vSum / n;
            if (referenceIsUsableSample(rAverage, vAverage) && referencePassesChecks(s, rAverage, vAverage, config)) {
                referenceSeed(s, rAverage, vAverage);
            }
        }
        s.windowPeriods = 0;
        s.windowSamples = 0;
        s.rSumAtWindowEnd.setZero();
        s.vSum.setZero();
    }
    return referenceGuidanceSolution(s, config);
}

/*! The algorithm's reference matches the reference model on every period of the sample sequence.
 @param config Algorithm configuration
 @param samples One (r [m], v [m/s]) filter sample per control period, starting with the first
 */
inline void expectMatchesReferenceModel(const FlybyPointConfig& config,
                                        const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& samples) {
    FlybyPointAlgorithm alg(config);
    ReferenceFlybyState model{};
    for (size_t k = 0; k < samples.size(); ++k) {
        SCOPED_TRACE("period " + std::to_string(k));
        const auto& [r, v] = samples[k];
        expectReference(alg.updateState(r, v), referenceUpdateState(model, r, v, config));
    }
}

#endif  // TEST_FLYBY_POINT_HELPERS_H
