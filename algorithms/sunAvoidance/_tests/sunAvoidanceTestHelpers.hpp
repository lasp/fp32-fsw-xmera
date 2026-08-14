#ifndef F32XMERA_SUN_AVOIDANCE_TEST_HELPERS_H
#define F32XMERA_SUN_AVOIDANCE_TEST_HELPERS_H

#include "sunAvoidanceAlgorithm.h"
#include "utilities/fsw/rigidBodyKinematics.hpp"
#include "utilities/fsw/safeMath.h"
#include "utilities/fsw/timeConstants.h"

#include <gtest/gtest.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <cstdint>
#include <numbers>

// ---------------------------------------------------------------------------
// Independent reference implementation of the sunAvoidance algorithm's output: the Sun-avoidance
// maneuver-adjusted reference frame (sigma_RN, omega_RN_N, domega_RN_N).
// ---------------------------------------------------------------------------
class SunAvoidanceReference {
   public:
    SunAvoidanceReference(const Eigen::Vector3f& sensitiveHat_B, float slewRate, bool computeAngleStart)
        : sensitiveHat_B(sensitiveHat_B.normalized()), slewRate(slewRate), computeAngleStart(computeAngleStart) {}

    SunAvoidanceOutput update(const Eigen::Vector3f& sigma_BN,
                              const Eigen::Vector3f& sigma_RN,
                              const Eigen::Vector3f& omega_RN_N,
                              const Eigen::Vector3f& domega_RN_N,
                              const Eigen::Vector3d& r_BN_N,
                              const Eigen::Vector3d& r_SN_N,
                              uint64_t callTime) {
        if (!this->maneuverInitialized) {
            if (this->computeAngleStart) {
                const Eigen::Matrix3f dcm_BN = mrpToDcm(sigma_BN);
                const Eigen::Vector3f sHat_N = (r_SN_N - r_BN_N).normalized().cast<float>();
                const Eigen::Vector3f sensInitial_N = dcm_BN.transpose() * this->sensitiveHat_B;
                const Eigen::Matrix3f dcm_RN = mrpToDcm(sigma_RN);
                const Eigen::Vector3f sensFinal_N = dcm_RN.transpose() * this->sensitiveHat_B;

                const Eigen::Vector3f sensAxis_N = sensInitial_N.cross(sensFinal_N).normalized();
                const Eigen::Vector3f pHat_N = (sHat_N - (sensAxis_N.dot(sHat_N)) * sensAxis_N).normalized();

                const float initMnvrAngle = safeAcosf(sensInitial_N.dot(sensFinal_N));
                const float initCelAngle = safeAcosf(pHat_N.dot(sensInitial_N));

                const Eigen::Matrix3f dcm_BR = dcm_BN * dcm_RN.transpose();
                const Eigen::Vector3f prv_BR = dcmToPrv(dcm_BR);
                this->angleStart = prv_BR.norm();
                this->mnvrAxis_B = prv_BR.normalized();

                const Eigen::Vector3f sensToSunAxis_N = sensInitial_N.cross(sHat_N).normalized();
                const Eigen::Vector3f mnvrAxis_N = dcm_BN.transpose() * this->mnvrAxis_B;
                if (sensToSunAxis_N.dot(mnvrAxis_N) < 0.0F && initCelAngle < initMnvrAngle) {
                    this->angleStart = (2.0F * std::numbers::pi_v<float>)-this->angleStart;
                    this->mnvrAxis_B = -this->mnvrAxis_B;
                }
            } else {
                this->angleStart = 0.0F;
            }
            this->mnvrStartTime = callTime;
            this->maneuverInitialized = true;
        }

        const Eigen::Matrix3f dcm_BN = mrpToDcm(sigma_BN);
        const Eigen::Matrix3f dcm_RN = mrpToDcm(sigma_RN);

        const float dtSeconds = static_cast<float>(callTime - this->mnvrStartTime) * kNano2SecF;
        float relativeAngleCurr = this->angleStart - (this->slewRate * dtSeconds);
        relativeAngleCurr = relativeAngleCurr < 0.0F ? 0.0F : relativeAngleCurr;

        SunAvoidanceOutput out{};
        const Eigen::Vector3f prv_cmd = relativeAngleCurr * this->mnvrAxis_B;
        const Eigen::Matrix3f dcmCmd = prvToDcm(prv_cmd);
        const Eigen::Matrix3f dcm_RcN = dcmCmd * dcm_RN;
        out.sigma_RN = dcmToMrp(dcm_RcN);

        Eigen::Vector3f omega_RcN_N = omega_RN_N;
        if (relativeAngleCurr > 0.0F) {
            omega_RcN_N += -this->slewRate * (dcm_BN.transpose() * this->mnvrAxis_B);
        }
        out.omega_RN_N = omega_RcN_N;
        out.domega_RN_N = domega_RN_N;
        return out;
    }

   private:
    Eigen::Vector3f sensitiveHat_B;
    float slewRate;
    bool computeAngleStart;

    bool maneuverInitialized = false;
    float angleStart = 0.0F;
    Eigen::Vector3f mnvrAxis_B = Eigen::Vector3f::Zero();
    uint64_t mnvrStartTime = 0;
};

// Whether the maneuver-path geometry is near a degeneracy or near the discrete short/long-way decision
// boundary. Near these, an independent fp32 reference can select the opposite (equally valid) maneuver, so
// the regression helper skips such inputs rather than compare against an ambiguous branch. Mirrors the
// quantities the algorithm's short/long-way decision turns on.
inline bool nearManeuverDecisionBoundary(const Eigen::Vector3f& sensitiveHat_B,
                                         const Eigen::Vector3f& sigma_BN,
                                         const Eigen::Vector3f& sigma_RN,
                                         const Eigen::Vector3d& r_BN_N,
                                         const Eigen::Vector3d& r_SN_N) {
    constexpr float kMargin = 1e-2F;  // comfortably above fp32 noise; excludes only thin shells

    const Eigen::Vector3f sHat_N = (r_SN_N - r_BN_N).stableNormalized().cast<float>();
    if (r_SN_N.stableNorm() <= 0.0 || sHat_N.stableNorm() <= 0.0F) {
        return true;  // no usable Sun information (the algorithm passes through)
    }

    const Eigen::Vector3f sensitive_B = sensitiveHat_B.normalized();
    const Eigen::Matrix3f dcm_BN = mrpToDcm(sigma_BN);
    const Eigen::Matrix3f dcm_RN = mrpToDcm(sigma_RN);
    const Eigen::Vector3f sensitiveInitial_N = dcm_BN.transpose() * sensitive_B;
    const Eigen::Vector3f sensitiveFinal_N = dcm_RN.transpose() * sensitive_B;

    // Degeneracies: each raw magnitude is a sin(angle) that stableNormalized() would collapse to zero.
    const Eigen::Vector3f sweepAxisRaw = sensitiveInitial_N.cross(sensitiveFinal_N);
    if (sweepAxisRaw.norm() < kMargin) {
        return true;  // initial and final sensitive axes near parallel / anti-parallel
    }
    const Eigen::Vector3f sweepAxis_N = sweepAxisRaw.normalized();
    const Eigen::Vector3f sunInPlaneRaw = sHat_N - (sweepAxis_N.dot(sHat_N)) * sweepAxis_N;
    if (sunInPlaneRaw.norm() < kMargin) {
        return true;  // Sun near parallel to the sweep axis
    }
    const Eigen::Vector3f initialToSunAxisRaw = sensitiveInitial_N.cross(sHat_N);
    if (initialToSunAxisRaw.norm() < kMargin) {
        return true;  // Sun near parallel to the initial sensitive axis
    }

    // Decision ties: the two comparisons the short/long-way branch turns on.
    const float sensitiveSweepAngle = safeAcosf(sensitiveInitial_N.dot(sensitiveFinal_N));
    const float initialToSunAngle = safeAcosf(sensitiveInitial_N.dot(sunInPlaneRaw.normalized()));
    if (std::fabs(initialToSunAngle - sensitiveSweepAngle) < kMargin) {
        return true;  // Sun near the edge of the swept arc
    }
    const Eigen::Matrix3f dcm_BR = dcm_BN * dcm_RN.transpose();
    const Eigen::Vector3f maneuverAxis_B = dcmToPrv(dcm_BR).stableNormalized();
    const Eigen::Vector3f initialToReferenceAxis_N = -(dcm_BN.transpose() * maneuverAxis_B);
    if (std::fabs(initialToSunAxisRaw.normalized().dot(initialToReferenceAxis_N)) < kMargin) {
        return true;  // maneuver near perpendicular to the Sun (toward / away tie)
    }

    return false;
}

// ---------------------------------------------------------------------------
// Regression helper: drive the algorithm and the independent reference through a time sequence with
// the given configuration, navigation attitude, reference frame and Sun geometry, and assert the
// adjusted-reference output agrees at every step. On the maneuver path, inputs near a degeneracy or the
// discrete short/long-way decision boundary are skipped (see nearManeuverDecisionBoundary).
// ---------------------------------------------------------------------------
inline void regressionTestSunAvoidance(const Eigen::Vector3f& sensitiveHat_B,
                                       float slewRate,
                                       bool computeAngleStart,
                                       const Eigen::Vector3f& sigma_BN,
                                       const Eigen::Vector3f& sigma_RN,
                                       const Eigen::Vector3f& omega_RN_N,
                                       const Eigen::Vector3f& domega_RN_N,
                                       const Eigen::Vector3d& r_BN_N,
                                       const Eigen::Vector3d& r_SN_N,
                                       uint64_t stepNs,
                                       int numSteps) {
    if (computeAngleStart && nearManeuverDecisionBoundary(sensitiveHat_B, sigma_BN, sigma_RN, r_BN_N, r_SN_N)) {
        return;  // ambiguous short/long-way branch: skip
    }

    const auto config = SunAvoidanceConfig::create(sensitiveHat_B, slewRate, computeAngleStart);
    SunAvoidanceAlgorithm alg{config};
    SunAvoidanceReference ref{sensitiveHat_B, slewRate, computeAngleStart};

    const SunAvoidanceAttRefInputs refIn{sigma_RN, omega_RN_N, domega_RN_N};

    constexpr float tol = 1e-5F;
    for (int k = 0; k < numSteps; ++k) {
        const uint64_t callTime = static_cast<uint64_t>(k) * stepNs;
        const SunAvoidanceOutput algOut = alg.update(sigma_BN, refIn, r_BN_N, r_SN_N, callTime);
        const SunAvoidanceOutput refOut =
            ref.update(sigma_BN, sigma_RN, omega_RN_N, domega_RN_N, r_BN_N, r_SN_N, callTime);

        EXPECT_TRUE(algOut.sigma_RN.allFinite()) << "sigma_RN not finite at step " << k;
        EXPECT_TRUE(algOut.omega_RN_N.allFinite()) << "omega_RN_N not finite at step " << k;
        EXPECT_TRUE(algOut.domega_RN_N.allFinite()) << "domega_RN_N not finite at step " << k;

        // Compare the adjusted-reference attitude via its DCM: at a ~180-deg attitude the two
        // independent dcmToMrp evaluations can land on opposite (equivalent) MRP shadow-set
        // representatives. The rate vectors are unambiguous.
        const Eigen::Matrix3f dcm_alg = mrpToDcm(algOut.sigma_RN);
        const Eigen::Matrix3f dcm_ref = mrpToDcm(refOut.sigma_RN);
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                EXPECT_NEAR(dcm_alg(r, c), dcm_ref(r, c), tol) << "sigma_RN (dcm) mismatch at step " << k;
            }
        }
        for (int i = 0; i < 3; ++i) {
            EXPECT_NEAR(algOut.omega_RN_N(i), refOut.omega_RN_N(i), tol) << "omega_RN_N mismatch at step " << k;
            EXPECT_NEAR(algOut.domega_RN_N(i), refOut.domega_RN_N(i), tol) << "domega_RN_N mismatch at step " << k;
        }
    }
}

// ---------------------------------------------------------------------------
// Property helpers
// ---------------------------------------------------------------------------

namespace detail {
constexpr uint64_t kStepNs = 500000000ULL;                           // 0.5 s
constexpr float kManeuverRate = std::numbers::pi_v<float> / 180.0F;  // 1 deg/s
inline Eigen::Vector3f sensitiveHat_B() { return Eigen::Vector3f{0.0F, -1.0F, 0.0F}; }
inline Eigen::Vector3d rBN_N() { return Eigen::Vector3d{-30.0, 20.0, -50.0}; }
inline Eigen::Vector3d rSN_N() { return Eigen::Vector3d{1.0, 2.0, 3.0}; }
}  // namespace detail

// With the maneuver disabled (computeAngleStart == false), the adjusted reference equals the input
// reference. The attitude is compared via its DCM so the check is independent of which MRP shadow-set
// representative dcmToMrp returns for a non-principal input. slewRate must still be a valid (positive)
// rate, but it has nothing to feed forward: the maneuver angle starts at zero.
inline void propertyPassThroughEqualsInputRef(const Eigen::Vector3f& sigma_BN,
                                              const Eigen::Vector3f& sigma_RN,
                                              const Eigen::Vector3f& omega_RN_N,
                                              const Eigen::Vector3f& domega_RN_N) {
    const auto config = SunAvoidanceConfig::create(Eigen::Vector3f::Zero(), detail::kManeuverRate, false);
    SunAvoidanceAlgorithm alg{config};
    const SunAvoidanceAttRefInputs refIn{sigma_RN, omega_RN_N, domega_RN_N};
    const Eigen::Matrix3f dcm_RN_in = mrpToDcm(sigma_RN);

    constexpr float tol = 1e-5F;
    for (int k = 0; k < 3; ++k) {
        const SunAvoidanceOutput out = alg.update(sigma_BN,
                                                  refIn,
                                                  Eigen::Vector3d::Zero(),
                                                  Eigen::Vector3d::Zero(),
                                                  static_cast<uint64_t>(k) * detail::kStepNs);
        const Eigen::Matrix3f dcm_RN_out = mrpToDcm(out.sigma_RN);
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                EXPECT_NEAR(dcm_RN_out(r, c), dcm_RN_in(r, c), tol);
            }
        }
        for (int i = 0; i < 3; ++i) {
            EXPECT_NEAR(out.omega_RN_N(i), omega_RN_N(i), tol);
            EXPECT_NEAR(out.domega_RN_N(i), domega_RN_N(i), tol);
        }
    }
}

// With the maneuver engaged and well-separated (non-degenerate) Sun geometry, the adjusted reference
// is always finite and its MRP stays within the principal set (|sigma| <= 1). Not fuzzed: the caller
// must supply non-degenerate attitudes.
inline void propertyManeuverOutputBoundedAndFinite(const Eigen::Vector3f& sigma_BN,
                                                   const Eigen::Vector3f& sigma_RN,
                                                   const Eigen::Vector3f& omega_RN_N,
                                                   const Eigen::Vector3f& domega_RN_N) {
    const auto config = SunAvoidanceConfig::create(detail::sensitiveHat_B(), detail::kManeuverRate, true);
    SunAvoidanceAlgorithm alg{config};
    const SunAvoidanceAttRefInputs refIn{sigma_RN, omega_RN_N, domega_RN_N};

    constexpr float normBound = 1.0F + 1e-5F;
    for (int k = 0; k < 20; ++k) {
        const SunAvoidanceOutput out =
            alg.update(sigma_BN, refIn, detail::rBN_N(), detail::rSN_N(), static_cast<uint64_t>(k) * detail::kStepNs);
        EXPECT_TRUE(out.sigma_RN.allFinite());
        EXPECT_TRUE(out.omega_RN_N.allFinite());
        EXPECT_TRUE(out.domega_RN_N.allFinite());
        EXPECT_LE(out.sigma_RN.norm(), normBound);
    }
}

// Once the maneuver angle has decayed to zero the adjusted reference again equals the input reference.
inline void propertyDecayedManeuverEqualsInputRef(const Eigen::Vector3f& sigma_BN,
                                                  const Eigen::Vector3f& sigma_RN,
                                                  const Eigen::Vector3f& omega_RN_N,
                                                  const Eigen::Vector3f& domega_RN_N) {
    const auto config = SunAvoidanceConfig::create(detail::sensitiveHat_B(), detail::kManeuverRate, true);
    SunAvoidanceAlgorithm alg{config};
    const SunAvoidanceAttRefInputs refIn{sigma_RN, omega_RN_N, domega_RN_N};
    const Eigen::Matrix3f dcm_RN_in = mrpToDcm(sigma_RN);

    // A full 2*pi maneuver at 1 deg/s decays in <= 360 s; 800 half-second steps guarantees completion.
    SunAvoidanceOutput out{};
    for (int k = 0; k < 800; ++k) {
        out = alg.update(sigma_BN, refIn, detail::rBN_N(), detail::rSN_N(), static_cast<uint64_t>(k) * detail::kStepNs);
    }

    constexpr float tol = 1e-5F;
    const Eigen::Matrix3f dcm_RN_out = mrpToDcm(out.sigma_RN);
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            EXPECT_NEAR(dcm_RN_out(r, c), dcm_RN_in(r, c), tol);
        }
    }
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(out.omega_RN_N(i), omega_RN_N(i), tol);
        EXPECT_NEAR(out.domega_RN_N(i), domega_RN_N(i), tol);
    }
}

// reInitialize() restarts the maneuver: the first update after reInitialize reproduces the very first
// update's output (the maneuver is recomputed from scratch at the same call time).
inline void propertyReInitializeRestartsManeuver(const Eigen::Vector3f& sigma_BN,
                                                 const Eigen::Vector3f& sigma_RN,
                                                 const Eigen::Vector3f& omega_RN_N,
                                                 const Eigen::Vector3f& domega_RN_N) {
    const auto config = SunAvoidanceConfig::create(detail::sensitiveHat_B(), detail::kManeuverRate, true);
    SunAvoidanceAlgorithm alg{config};
    const SunAvoidanceAttRefInputs refIn{sigma_RN, omega_RN_N, domega_RN_N};

    const SunAvoidanceOutput first = alg.update(sigma_BN, refIn, detail::rBN_N(), detail::rSN_N(), 0);
    for (int k = 1; k < 5; ++k) {
        (void)alg.update(sigma_BN, refIn, detail::rBN_N(), detail::rSN_N(), static_cast<uint64_t>(k) * detail::kStepNs);
    }
    alg.reInitialize();
    const SunAvoidanceOutput afterReinit = alg.update(sigma_BN, refIn, detail::rBN_N(), detail::rSN_N(), 0);

    constexpr float tol = 1e-6F;
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(afterReinit.sigma_RN(i), first.sigma_RN(i), tol);
        EXPECT_NEAR(afterReinit.omega_RN_N(i), first.omega_RN_N(i), tol);
        EXPECT_NEAR(afterReinit.domega_RN_N(i), first.domega_RN_N(i), tol);
    }
}

// Fuzz entry point: exercise the shared regressionTestSunAvoidance on the maneuver path (computeAngleStart
// = true) for arbitrary attitudes and realistic Sun geometry. The helper skips inputs near a degeneracy or
// near the discrete short/long-way decision boundary (see nearManeuverDecisionBoundary), where an
// independent fp32 reference can select the opposite (equally valid) maneuver; away from those the algorithm
// and reference agree, and the output is checked finite at every step.
inline void fuzzRegressionSunAvoidance(const Eigen::Vector3f& sigma_BN,
                                       const Eigen::Vector3f& sigma_RN,
                                       const Eigen::Vector3f& omega_RN_N,
                                       const Eigen::Vector3f& domega_RN_N,
                                       const Eigen::Vector3d& r_BN_N,
                                       const Eigen::Vector3d& r_SN_N) {
    regressionTestSunAvoidance(detail::sensitiveHat_B(),
                               detail::kManeuverRate,
                               true,
                               sigma_BN,
                               sigma_RN,
                               omega_RN_N,
                               domega_RN_N,
                               r_BN_N,
                               r_SN_N,
                               detail::kStepNs,
                               12);
}

#endif  // F32XMERA_SUN_AVOIDANCE_TEST_HELPERS_H
