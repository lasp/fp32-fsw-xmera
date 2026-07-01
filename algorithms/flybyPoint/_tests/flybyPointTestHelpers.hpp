#ifndef TEST_FLYBY_POINT_HELPERS_H
#define TEST_FLYBY_POINT_HELPERS_H

#include "flybyPointAlgorithm.h"
#include "utilities/fsw/rigidBodyKinematics.hpp"
#include "utilities/fsw/safeMath.h"
#include "utilities/fsw/timeConstants.h"

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <cmath>
#include <numbers>

struct ReferenceFlybyOutput {
    Eigen::Vector3d sigma_RN;
    Eigen::Vector3d omega_RN_N;
    Eigen::Vector3d domega_RN_N;
};

// Plain data mirror of FlybyPointAlgorithm's private state. R0N is Matrix3d (double precision)
// so it serves as ground truth against which the float32 production outputs are compared.
struct ReferenceFlybyState {
    bool firstRead = true;
    uint64_t lastFilterReadTime = 0;
    double timeOfFirstRead = 0.0;
    Eigen::Vector3d firstNavPosition = Eigen::Vector3d::Zero();
    Eigen::Vector3d firstNavVelocity = Eigen::Vector3d::Zero();
    double f0 = 0.0;
    double gamma0 = 0.0;
    Eigen::Matrix3d R0N = Eigen::Matrix3d::Identity();
    double dt = 0.0;
};

/*! Resets the reference state to its post-construction values, mirroring FlybyPointAlgorithm::reset().
 @param s The reference state to reset
 */
inline void referenceReset(ReferenceFlybyState& s) {
    s.firstRead = true;
    s.lastFilterReadTime = 0;
}

/*! Computes f0 and gamma0 from r and v and stores them in state, mirroring
 *  FlybyPointAlgorithm::computeFlybyParameters().
 @param s The reference state to update
 @param r The relative position state [m]
 @param v The relative velocity state [m/s]
 */
inline void referenceComputeFlybyParameters(ReferenceFlybyState& s,
                                            const Eigen::Vector3d& r,
                                            const Eigen::Vector3d& v) {
    s.f0 = v.norm() / r.norm();
    const Eigen::Vector3d ur = r.normalized();
    const Eigen::Vector3d uv = v.normalized();
    const Eigen::Vector3d uh = ur.cross(uv).normalized();
    const Eigen::Vector3d ut = uh.cross(ur).normalized();
    s.gamma0 = safeAtan2(v.dot(ur), v.dot(ut));
}

/*! Builds the inertial-to-reference DCM R0N from r and v and stores it in state, mirroring
 *  FlybyPointAlgorithm::computeRN().
 @param s The reference state to update
 @param r The relative position state [m]
 @param v The relative velocity state [m/s]
 */
inline void referenceComputeRN(ReferenceFlybyState& s, const Eigen::Vector3d& r, const Eigen::Vector3d& v) {
    const Eigen::Vector3d ur = r.normalized();
    const Eigen::Vector3d uv = v.normalized();
    const Eigen::Vector3d uh = ur.cross(uv).normalized();
    const Eigen::Vector3d ut = uh.cross(ur).normalized();
    s.R0N.row(0) = ur;
    s.R0N.row(1) = ut;
    s.R0N.row(2) = uh;
}

/*! Mirrors FlybyPointAlgorithm::checkValidity(). Uses the stored gamma0 from the previous seed,
 *  exactly as the production code does.
 @return true if all validity checks pass and a re-seed should proceed
 @param s The current reference state (read-only)
 @param currentSimNanos The current simulation time [ns]
 @param r The new relative position state [m]
 @param v The new relative velocity state [m/s]
 @param config Algorithm configuration
 */
inline bool referenceCheckValidity(const ReferenceFlybyState& s,
                                   uint64_t currentSimNanos,
                                   const Eigen::Vector3d& r,
                                   const Eigen::Vector3d& v,
                                   const FlybyPointConfig& config) {
    static constexpr double kRad2Deg = 180.0 / std::numbers::pi;
    static constexpr double kMaxAccelCoeff = 3.0 * std::numbers::sqrt3 / 8.0;

    const Eigen::Vector3d ur = r.normalized();
    const Eigen::Vector3d uv = v.normalized();
    if (std::fabs(1.0 - ur.dot(uv)) < config.getToleranceForCollinearity()) return false;

    const double dca = -r.norm() * safeSin(s.gamma0);
    const double maxRate = v.norm() / dca * kRad2Deg;
    if (maxRate > config.getMaximumRateThreshold() && config.getMaximumRateThreshold() > 0) return false;

    const double maxAccel = kMaxAccelCoeff * std::pow(v.norm() / dca, 2) * kRad2Deg;
    if (maxAccel > config.getMaximumAccelerationThreshold() && config.getMaximumAccelerationThreshold() > 0)
        return false;

    const double deltaT = static_cast<double>(currentSimNanos) * kNano2Sec - s.timeOfFirstRead;
    const double deltaPosNorm = (r - (s.firstNavPosition + deltaT * s.firstNavVelocity)).norm();
    if (deltaPosNorm > config.getPositionKnowledgeSigma() && config.getPositionKnowledgeSigma() > 0) return false;

    return true;
}

/*! Computes the guidance frame from the current reference state, mirroring
 *  FlybyPointAlgorithm::computeGuidanceSolution().
 @return ReferenceFlybyOutput containing sigma_RN, omega_RN_N, and domega_RN_N
 @param s The current reference state (read-only)
 @param signOfOrbitNormal Sign of the orbit-normal reference vector (+1 or -1)
 */
inline ReferenceFlybyOutput referenceGuidanceSolution(const ReferenceFlybyState& s, int signOfOrbitNormal) {
    const double theta = safeAtan(safeTan(s.gamma0) + s.f0 / safeCos(s.gamma0) * s.dt) - s.gamma0;
    const Eigen::Matrix3d RtR0 = prvToDcm(Eigen::Vector3d{0, 0, theta});
    const Eigen::Matrix3d RtN = RtR0 * s.R0N;

    const double den = s.f0 * s.f0 * s.dt * s.dt + 2.0 * s.f0 * safeSin(s.gamma0) * s.dt + 1.0;
    const double thetaDot = s.f0 * safeCos(s.gamma0) / den;
    const double thetaDDot = -2.0 * s.f0 * s.f0 * safeCos(s.gamma0) * (s.f0 * s.dt + safeSin(s.gamma0)) / (den * den);

    Eigen::Vector3d sigma_RN = dcmToMrp(RtN);
    if (signOfOrbitNormal == -1) {
        Eigen::Vector3d const halfRotX{1, 0, 0};
        sigma_RN = addMrp(sigma_RN, halfRotX);
    }
    return {sigma_RN,
            RtN.transpose() * Eigen::Vector3d{0, 0, thetaDot},
            RtN.transpose() * Eigen::Vector3d{0, 0, thetaDDot}};
}

/*! Advances the reference state by one call, mirroring FlybyPointAlgorithm::updateState().
 *  Handles the full state machine: first-read seeding, validity-gated re-seeding, and extrapolation.
 @return ReferenceFlybyOutput containing sigma_RN, omega_RN_N, and domega_RN_N
 @param s The reference state to update
 @param currentSimNanos The current simulation time [ns]
 @param r The relative position state [m]
 @param v The relative velocity state [m/s]
 @param config Algorithm configuration
 */
inline ReferenceFlybyOutput referenceUpdateState(ReferenceFlybyState& s,
                                                 uint64_t currentSimNanos,
                                                 const Eigen::Vector3d& r,
                                                 const Eigen::Vector3d& v,
                                                 const FlybyPointConfig& config) {
    s.dt = static_cast<double>(currentSimNanos - s.lastFilterReadTime) * kNano2Sec;
    if ((s.dt >= config.getTimeBetweenFilterData()) || s.firstRead) {
        if (s.firstRead) {
            s.timeOfFirstRead = static_cast<double>(currentSimNanos) * kNano2Sec;
            s.firstNavPosition = r;
            s.firstNavVelocity = v;
            referenceComputeFlybyParameters(s, r, v);
            referenceComputeRN(s, r, v);
            s.firstRead = false;
        } else if (referenceCheckValidity(s, currentSimNanos, r, v, config)) {
            referenceComputeFlybyParameters(s, r, v);
            referenceComputeRN(s, r, v);
            s.lastFilterReadTime = currentSimNanos;
            s.dt = 0;
        }
    }
    return referenceGuidanceSolution(s, config.getSignOfOrbitNormalFrameVector());
}

/*! Drives the algorithm and the reference through numSteps calls with the same (r, v) inputs
 *  and compares sigma_RN, omega_RN_N, and domega_RN_N at each step.
 *  Both are seeded together at t=0 and then stepped in lock-step. The reference handles all
 *  branches of the state machine (extrapolation and re-seeding) so there is no constraint on
 *  stepNanos or numSteps relative to timeBetweenFilterData.
 *
 *  Tolerance strategy (per PRECISION_GUIDELINES.md §7.5):
 *  - sigma_RN    : absolute kSigmaTol. MRP magnitude is bounded by 1 after shadow-set mapping,
 *                  so a fixed absolute floor is appropriate.
 *  - omega_RN_N  : relative kRelTol * |ref| + kAbsFloor. thetaDot = f0*cos(gamma0)/den scales
 *                  with f0 = v/r and is unbounded for large v or small r, so a fixed absolute
 *                  tolerance would fail for high-rate trajectories.
 *  - domega_RN_N : same relative+floor formula as omega_RN_N. thetaDDot has the same scaling.
 *
 *  Derivation of kRelTol (PRECISION_GUIDELINES.md §7.2, §7.5):
 *  The dominant error source is computeRN() storing unit vectors as float32
 *  (unit_vector.cast<float>()), introducing ~1 ULP ~ 1.19e-7 relative error per DCM row.
 *  computeGuidanceSolution() casts R0N back to double and multiplies by a 3-vector (~3 ops),
 *  giving ~4 * 1.19e-7 ~ 4.8e-7 relative error in omega and domega. A 20x safety margin
 *  yields kRelTol = 1e-5F.
 @param config Algorithm configuration
 @param r_BN_N The relative position state passed at every call [m]
 @param v_BN_N The relative velocity state passed at every call [m/s]
 @param stepNanos Time between successive updateState calls [ns]
 @param numSteps Number of steps to run after the initial seed at t=0
 */
inline void regressionTestFlybyPoint(const FlybyPointConfig& config,
                                     const Eigen::Vector3d& r_BN_N,
                                     const Eigen::Vector3d& v_BN_N,
                                     uint64_t stepNanos,
                                     int numSteps) {
    static constexpr float kSigmaTol = 1e-6F;  // absolute: sigma bounded by |sigma| <= 1
    static constexpr float kRelTol = 1e-5F;    // relative: ~4 ops * float_eps * 20x margin
    static constexpr float kAbsFloor = 1e-5F;  // floor for near-zero omega/domega values

    FlybyPointAlgorithm alg(config);
    alg.reset();

    ReferenceFlybyState refState{};
    referenceReset(refState);

    alg.updateState(0U, r_BN_N, v_BN_N);
    referenceUpdateState(refState, 0U, r_BN_N, v_BN_N, config);

    for (int k = 1; k <= numSteps; ++k) {
        const uint64_t simNanos = static_cast<uint64_t>(k) * stepNanos;

        const AttGuideOutput out = alg.updateState(simNanos, r_BN_N, v_BN_N);
        const ReferenceFlybyOutput ref = referenceUpdateState(refState, simNanos, r_BN_N, v_BN_N, config);

        if (out.validOutput) {
            // Compare MRPs using nominal and shadow representations
            Eigen::Vector3d sigma_out = out.sigma_RN.cast<double>();
            Eigen::Vector3d sigma_ref = ref.sigma_RN;
            Eigen::Vector3d sigma_ref_shadow = sigma_ref;

            if (sigma_ref.squaredNorm() > 1e-12) {
                sigma_ref_shadow = -sigma_ref / sigma_ref.squaredNorm();
            }

            double error_norm = (sigma_out - sigma_ref).norm();
            double error_shadow = (sigma_out - sigma_ref_shadow).norm();

            EXPECT_TRUE(error_norm < 1e-6 || error_shadow < 1e-6);

            Eigen::Vector3d sigma_compared = sigma_ref;
            if (error_shadow < error_norm) {
                sigma_compared = sigma_ref_shadow;
            }

            for (int i = 0; i < 3; ++i) {
                const float refOmega = static_cast<float>(ref.omega_RN_N[i]);
                const float refDomega = static_cast<float>(ref.domega_RN_N[i]);
                EXPECT_NEAR(sigma_out[i], sigma_compared[i], kSigmaTol);
                EXPECT_NEAR(out.omega_RN_N[i], refOmega, kRelTol * std::abs(refOmega) + kAbsFloor);
                EXPECT_NEAR(out.domega_RN_N[i], refDomega, kRelTol * std::abs(refDomega) + kAbsFloor);
                EXPECT_TRUE(std::isfinite(out.sigma_RN[i]));
                EXPECT_TRUE(std::isfinite(out.omega_RN_N[i]));
                EXPECT_TRUE(std::isfinite(out.domega_RN_N[i]));
            }
        }
    }
}

// Expected output if the algorithm keeps extrapolating the (seedR, seedV) solution for dt > timeBetweenFilterData
// (the reseed-attempt time used by the trigger tests below) instead of re-seeding -- i.e. what
// a rejected reseed (checkValidity == false) should produce.
inline ReferenceFlybyOutput expectedExtrapolatedOutput(const double timeBetweenFilterData,
                                                       const Eigen::Vector3d& seedR,
                                                       const Eigen::Vector3d& seedV,
                                                       int signOfOrbitNormal) {
    ReferenceFlybyState s{};
    referenceComputeFlybyParameters(s, seedR, seedV);
    referenceComputeRN(s, seedR, seedV);
    s.dt = timeBetweenFilterData + 0.1;
    return referenceGuidanceSolution(s, signOfOrbitNormal);
}

inline void expectMatchesExtrapolation(const AttGuideOutput& out, const ReferenceFlybyOutput& ref) {
    static constexpr float kSigmaTol = 1e-6F;
    static constexpr float kRelTol = 1e-5F;
    static constexpr float kAbsFloor = 1e-5F;

    const Eigen::Vector3d sigmaOut = out.sigma_RN.cast<double>();
    Eigen::Vector3d sigmaRefShadow = ref.sigma_RN;
    if (ref.sigma_RN.squaredNorm() > 1e-12) {
        sigmaRefShadow = -ref.sigma_RN / ref.sigma_RN.squaredNorm();
    }
    const double errorNorm = (sigmaOut - ref.sigma_RN).norm();
    const double errorShadow = (sigmaOut - sigmaRefShadow).norm();
    ASSERT_TRUE(errorNorm < 1e-6 || errorShadow < 1e-6);
    const Eigen::Vector3d& sigmaCompared = (errorShadow < errorNorm) ? sigmaRefShadow : ref.sigma_RN;

    for (int i = 0; i < 3; ++i) {
        const float refOmega = static_cast<float>(ref.omega_RN_N[i]);
        const float refDomega = static_cast<float>(ref.domega_RN_N[i]);
        EXPECT_NEAR(sigmaOut[i], sigmaCompared[i], kSigmaTol);
        EXPECT_NEAR(out.omega_RN_N[i], refOmega, kRelTol * std::abs(refOmega) + kAbsFloor);
        EXPECT_NEAR(out.domega_RN_N[i], refDomega, kRelTol * std::abs(refDomega) + kAbsFloor);
    }
}

#endif  // TEST_FLYBY_POINT_HELPERS_H
