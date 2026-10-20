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
// Test scenario: straight-line flyby and default configuration
// ---------------------------------------------------------------------------
const Eigen::Vector3f kR0{-5e7F, 7.5e6F, 5e5F};     // [m] position at the first sample, and default ground position
const Eigen::Vector3f kV{2e4F, 0.0F, 0.0F};         // [m/s] constant velocity, and default ground velocity
const Eigen::Vector3f kShift{0.0F, 0.0F, 1e4F};     // [m] off-line shift that passes the checks of the tests using it
const Eigen::Vector3f kPositionOffset{0, 0, 2e5F};  // [m] offset that fails the state check of the tests using it

/*! Position on the test trajectory t seconds after the first sample, computed in double and rounded to float. */
inline Eigen::Vector3f truthAt(double t) { return (kR0.cast<double>() + t * kV.cast<double>()).cast<float>(); }

/*! FlybyPointConfig::create() arguments; the defaults pass every check on the test trajectory. */
struct ConfigParams {
    float controlPeriod = 1.0F;
    uint32_t filterReadPeriods = 1U;
    float toleranceForCollinearity = 1e-3F;
    int signOfOrbitNormalFrameVector = 1;
    float maximumRateThreshold = 10.0F;
    float maximumAccelerationThreshold = 1.0F;
    float numberOfSigmas = 3.0F;
    Eigen::Vector3f groundPosition_BN_N = kR0;
    Eigen::Vector3f groundVelocity_BN_N = kV;
    Eigen::Vector3f groundPositionCovarianceDiag_N = Eigen::Vector3f::Constant(1e18F);
    Eigen::Vector3f groundVelocityCovarianceDiag_N = Eigen::Vector3f::Constant(1e10F);
};

inline FlybyPointConfig makeConfig(const ConfigParams& p = {}) {
    return FlybyPointConfig::create(p.controlPeriod,
                                    p.filterReadPeriods,
                                    p.toleranceForCollinearity,
                                    p.signOfOrbitNormalFrameVector,
                                    p.maximumRateThreshold,
                                    p.maximumAccelerationThreshold,
                                    p.numberOfSigmas,
                                    p.groundPosition_BN_N,
                                    p.groundVelocity_BN_N,
                                    p.groundPositionCovarianceDiag_N,
                                    p.groundVelocityCovarianceDiag_N);
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

/*! Exact reference t seconds after (r0, v) on a straight line, from the geometry alone, in double.
 @return the expected reference
 @param r0 [m] position at time 0
 @param v [m/s] constant velocity
 @param t [s] time since r0
 @param sign signOfOrbitNormalFrameVector
 */
inline ReferenceFlybyOutput exactStraightLineReference(const Eigen::Vector3d& r0,
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

/*! exactStraightLineReference() for the float vectors the algorithm is given. */
inline ReferenceFlybyOutput straightLineReference(const Eigen::Vector3f& r0,
                                                  const Eigen::Vector3f& v,
                                                  double t = 0.0,
                                                  int sign = 1) {
    return exactStraightLineReference(r0.cast<double>(), v.cast<double>(), t, sign);
}

/*! The output matches the expected reference within float32 tolerance; DCMs are compared, so either MRP passes.
 @param domegaScale [rad/s^2] acceleration scale for the tolerance near zero acceleration
 @param dcmTolScale, rateTolScale factors (>= 1) on the DCM and rate tolerances for ill-conditioned geometry
 */
inline void expectReference(const AttGuideOutput& out,
                            const ReferenceFlybyOutput& expected,
                            double domegaScale = 0.0,
                            double dcmTolScale = 1.0,
                            double rateTolScale = 1.0) {
    static constexpr double kDcmTol = 1e-6;
    static constexpr double kRelTol = 1e-5;
    static constexpr double kTinyFloor = 1e-30;
    const double dcmTol = kDcmTol * dcmTolScale;
    const double relTol = kRelTol * rateTolScale;

    const Eigen::Matrix3d RN = mrpToDcm(Eigen::Vector3d(out.sigma_RN.cast<double>()));
    EXPECT_LT((RN - expected.RN).cwiseAbs().maxCoeff(), dcmTol) << "RN\n" << RN << "\nexpected\n" << expected.RN;
    const Eigen::Vector3d omegaError = out.omega_RN_N.cast<double>() - expected.omega_RN_N;
    EXPECT_LE(omegaError.cwiseAbs().maxCoeff(), relTol * expected.omega_RN_N.stableNorm() + kTinyFloor)
        << "omega " << out.omega_RN_N.transpose() << ", expected " << expected.omega_RN_N.transpose();
    const Eigen::Vector3d domegaError = out.domega_RN_N.cast<double>() - expected.domega_RN_N;
    EXPECT_LE(domegaError.cwiseAbs().maxCoeff(),
              relTol * std::max(expected.domega_RN_N.stableNorm(), domegaScale) + kTinyFloor)
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
    bool stateKnowledge = false;
    bool inputSampleRejected = false;
    uint32_t rejectedSamplesInWindow = 0U;
};

inline void expectFlags(const AttGuideOutput& out, const ExpectedFlags& expected = {}) {
    EXPECT_EQ(out.collinearityTrigger, expected.collinearity);
    EXPECT_EQ(out.maxRateTrigger, expected.maxRate);
    EXPECT_EQ(out.maxAccelerationTrigger, expected.maxAcceleration);
    EXPECT_EQ(out.stateKnowledgeExceedTrigger, expected.stateKnowledge);
    EXPECT_EQ(out.inputSampleRejected, expected.inputSampleRejected);
    EXPECT_EQ(out.rejectedSamplesInWindow, expected.rejectedSamplesInWindow);
}

// ---------------------------------------------------------------------------
// Reference model: a single-precision copy of the algorithm, for regression tests
// ---------------------------------------------------------------------------
struct ReferenceFlybyState {
    uint64_t periodsSinceGroundState = 0;
    uint64_t periodsSinceRead = 0;
    uint32_t windowPeriods = 0;
    uint32_t windowSamples = 0;
    Eigen::Vector3f rMeanAtWindowEnd = Eigen::Vector3f::Zero();
    Eigen::Vector3f vMean = Eigen::Vector3f::Zero();
    float f0 = 0.0F;
    float gamma0 = 0.0F;
    Eigen::Matrix3f R0N = Eigen::Matrix3f::Identity();
};

/*! Same test as FlybyPointConfig::isUsableSample(). */
inline bool referenceIsUsableSample(const Eigen::Vector3f& r, const Eigen::Vector3f& v) {
    const float rNorm = r.stableNorm();
    const float vNorm = v.stableNorm();
    return r.allFinite() && v.allFinite() && rNorm >= FlybyPointConfig::kMinPositionNorm &&
           vNorm >= FlybyPointConfig::kMinVelocityNorm && std::isfinite(rNorm * vNorm) &&
           std::isfinite((vNorm / rNorm) * (vNorm / rNorm));
}

/*! Same test as FlybyPointConfig::isCollinear(). */
inline bool referenceIsCollinear(const Eigen::Vector3f& r, const Eigen::Vector3f& v, float toleranceForCollinearity) {
    const Eigen::Vector3f ur = r.stableNormalized();
    const Eigen::Vector3f uv = v.stableNormalized();
    const float sineAngle = ur.cross(uv).stableNorm();
    const float oneMinusAbsCos = sineAngle * sineAngle / (1.0F + std::fabs(ur.dot(uv)));
    return oneMinusAbsCos < toleranceForCollinearity || sineAngle < FlybyPointConfig::kMinOrbitNormalNorm;
}

/*! Same checks as checkValidity() in flybyPointAlgorithm.cpp.
 @return true if the candidate (r, v) passes every check
 */
inline bool referencePassesChecks(const ReferenceFlybyState& s,
                                  const Eigen::Vector3f& r,
                                  const Eigen::Vector3f& v,
                                  const FlybyPointConfig& config) {
    static constexpr float kRad2Deg = 180.0F / std::numbers::pi_v<float>;
    static constexpr float kMaxAccelCoeff = 3.0F * std::numbers::sqrt3_v<float> / 8.0F;
    const float distanceClosestApproach = r.cross(v).stableNorm() / v.stableNorm();
    if (referenceIsCollinear(r, v, config.getToleranceForCollinearity()) || !(distanceClosestApproach > 0.0F)) {
        return false;
    }
    const float speedOverDistance = v.stableNorm() / distanceClosestApproach;
    if (speedOverDistance * kRad2Deg > config.getMaximumRateThreshold() ||
        kMaxAccelCoeff * speedOverDistance * speedOverDistance * kRad2Deg > config.getMaximumAccelerationThreshold()) {
        return false;
    }

    // Ground state and covariance propagated from t0, checked per axis.
    const float t = static_cast<float>(s.periodsSinceGroundState) * config.getControlPeriod();
    const float n = config.getNumberOfSigmas();
    const Eigen::Vector3f rPredicted = config.getGroundPosition_BN_N() + t * config.getGroundVelocity_BN_N();
    const Eigen::Vector3f vPredicted = config.getGroundVelocity_BN_N();
    for (int i = 0; i < 3; ++i) {
        const float positionVariance = config.getGroundPositionCovarianceDiag_N()[i];
        const float velocityVariance = config.getGroundVelocityCovarianceDiag_N()[i];
        if (std::fabs(r[i] - rPredicted[i]) > n * std::sqrt(positionVariance + t * t * velocityVariance) ||
            std::fabs(v[i] - vPredicted[i]) > n * std::sqrt(velocityVariance)) {
            return false;
        }
    }
    return true;
}

/*! Same profile as FlybyPointAlgorithm::makeProfile(). */
inline void referenceSeed(ReferenceFlybyState& s, const Eigen::Vector3f& r, const Eigen::Vector3f& v) {
    const Eigen::Vector3f ur = r.stableNormalized();
    const Eigen::Vector3f uv = v.stableNormalized();
    const Eigen::Vector3f uh = ur.cross(uv).stableNormalized();
    const Eigen::Vector3f ut = uh.cross(ur).stableNormalized();
    s.R0N.row(0) = ur;
    s.R0N.row(1) = ut;
    s.R0N.row(2) = uh;
    s.f0 = v.stableNorm() / r.stableNorm();
    s.gamma0 = safeAtan2f(v.dot(ur), v.dot(ut));
    s.periodsSinceRead = 0;
}

/*! Reference model seeded from the configuration's ground state. */
inline ReferenceFlybyState referenceInit(const FlybyPointConfig& config) {
    ReferenceFlybyState s{};
    referenceSeed(s, config.getGroundPosition_BN_N(), config.getGroundVelocity_BN_N());
    return s;
}

/*! Same solution and float MRP output as computeGuidanceReference(), including its zero fallback. */
inline ReferenceFlybyOutput referenceGuidanceSolution(const ReferenceFlybyState& s, const FlybyPointConfig& config) {
    const float f0 = s.f0;
    const float gamma0 = s.gamma0;
    const float dt = static_cast<float>(s.periodsSinceRead) * config.getControlPeriod();
    const float cosGamma0 = safeCosf(gamma0);
    const float sinGamma0 = safeSinf(gamma0);
    const float theta = safeAtanf(safeTanf(gamma0) + (f0 / cosGamma0 * dt)) - gamma0;
    const float alongTrack = (f0 * dt) + sinGamma0;
    const float den = (alongTrack * alongTrack) + (cosGamma0 * cosGamma0);
    const float thetaDot = f0 * cosGamma0 / den;
    const float thetaDDot = -2.0F * f0 * f0 * cosGamma0 * alongTrack / (den * den);
    if (!std::isfinite(f0) || !std::isfinite(gamma0) || !std::isfinite(theta) || !std::isfinite(thetaDot) ||
        !std::isfinite(thetaDDot)) {
        return zeroReference();
    }

    const Eigen::Matrix3f RtN = prvToDcm(Eigen::Vector3f{0.0F, 0.0F, theta}) * s.R0N;
    const Eigen::Vector3f omega = RtN.transpose() * Eigen::Vector3f{0.0F, 0.0F, thetaDot};
    const Eigen::Vector3f domega = RtN.transpose() * Eigen::Vector3f{0.0F, 0.0F, thetaDDot};
    Eigen::Matrix3f RN = RtN;
    if (config.getSignOfOrbitNormalFrameVector() == -1) {
        RN.bottomRows<2>() *= -1.0F;  // 180 deg turn about the first axis
    }
    const Eigen::Vector3f sigma = dcmToMrp(RN);  // same float output step as the algorithm
    return {mrpToDcm(Eigen::Vector3d(sigma.cast<double>())), omega.cast<double>(), domega.cast<double>()};
}

/*! Advances the reference model by one control period, like FlybyPointAlgorithm::updateState(). */
inline ReferenceFlybyOutput referenceUpdateState(ReferenceFlybyState& s,
                                                 const Eigen::Vector3f& r,
                                                 const Eigen::Vector3f& v,
                                                 const FlybyPointConfig& config) {
    const uint32_t windowLength = config.getFilterReadPeriods();
    if (referenceIsUsableSample(r, v)) {
        const float timeToWindowEnd =
            static_cast<float>(windowLength - 1U - s.windowPeriods) * config.getControlPeriod();
        const Eigen::Vector3f rAtEnd = r + timeToWindowEnd * v;
        ++s.windowSamples;
        const auto n = static_cast<float>(s.windowSamples);
        s.rMeanAtWindowEnd += (rAtEnd - s.rMeanAtWindowEnd) / n;
        s.vMean += (v - s.vMean) / n;
    }
    ++s.windowPeriods;
    if (s.windowPeriods >= windowLength) {
        if (s.windowSamples > 0U) {
            const Eigen::Vector3f rAverage = s.rMeanAtWindowEnd;
            const Eigen::Vector3f vAverage = s.vMean;
            if (referenceIsUsableSample(rAverage, vAverage) && referencePassesChecks(s, rAverage, vAverage, config)) {
                referenceSeed(s, rAverage, vAverage);
            }
        }
        s.windowPeriods = 0;
        s.windowSamples = 0;
        s.rMeanAtWindowEnd.setZero();
        s.vMean.setZero();
    }
    const ReferenceFlybyOutput out = referenceGuidanceSolution(s, config);
    ++s.periodsSinceRead;
    ++s.periodsSinceGroundState;
    return out;
}

/*! The algorithm's reference matches the reference model on every period.
 @param samples one (r [m], v [m/s]) filter sample per period
 */
inline void expectMatchesReferenceModel(const FlybyPointConfig& config,
                                        const std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>>& samples) {
    FlybyPointAlgorithm alg(config);
    ReferenceFlybyState model = referenceInit(config);
    for (size_t k = 0; k < samples.size(); ++k) {
        SCOPED_TRACE("period " + std::to_string(k));
        const auto& [r, v] = samples[k];
        expectReference(alg.updateState(r, v), referenceUpdateState(model, r, v, config));
    }
}

#endif  // TEST_FLYBY_POINT_HELPERS_H
