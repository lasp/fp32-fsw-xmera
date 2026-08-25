#ifndef TEST_SUNAVOIDANCE_H
#define TEST_SUNAVOIDANCE_H

#include "attTrackingError/attTrackingErrorAlgorithm.h"
#include "sunAvoidanceAlgorithm.h"
#include "utilities/fsw/rigidBodyKinematics.hpp"
#include "utilities/fsw/safeMath.h"
#include "utilities/fsw/timeConstants.h"

#include <gtest/gtest.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <numbers>

// ---------------------------------------------------------------------------
// Independent reference implementation of the COMBINED sunAvoidance behavior
// (Sun-avoidance maneuver initialization + attitude tracking error). It is written
// only in terms of rigidBodyKinematics primitives and mirrors the current, monolithic
// sunAvoidance algorithm -- which today performs both the Sun-avoidance reference
// generation and the attitude-tracking-error computation in a single module.
//
// sunAvoidance is slated to be split so the tracking-error stage is delegated to the
// separate attTrackingError module. This reference pins the combined input->output
// behavior so the eventual pipeline (sunAvoidance -> attTrackingError) can be verified
// to reproduce it. The reference is stateful: the maneuver is initialized on the first
// update() and fed forward at the configured rate on subsequent calls.
// ---------------------------------------------------------------------------
struct SunAvoidanceReferenceOutput {
    Eigen::Vector3f sigma_BR = Eigen::Vector3f::Zero();
    Eigen::Vector3f omega_BR_B = Eigen::Vector3f::Zero();
    Eigen::Vector3f omega_RN_B = Eigen::Vector3f::Zero();
    Eigen::Vector3f domega_RN_B = Eigen::Vector3f::Zero();
};

class SunAvoidanceReference {
   public:
    SunAvoidanceReference(const Eigen::Vector3f& sigma_R0R,
                          const Eigen::Vector3f& sensitiveHat_B,
                          float angleRate,
                          bool computeAngleStart)
        : sigma_R0R(sigma_R0R),
          sensitiveHat_B(sensitiveHat_B.normalized()),
          angleRate(angleRate),
          computeAngleStart(computeAngleStart) {}

    SunAvoidanceReferenceOutput update(const Eigen::Vector3f& sigma_BN,
                                       const Eigen::Vector3f& omega_BN_B,
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

                const Eigen::Matrix3f dcm_R0N = mrpToDcm(sigma_RN);
                const Eigen::Matrix3f dcm_R0R = mrpToDcm(this->sigma_R0R);
                const Eigen::Matrix3f dcm_BNFinal = (dcm_R0N.transpose() * dcm_R0R).transpose();
                const Eigen::Vector3f sensFinal_N = dcm_BNFinal.transpose() * this->sensitiveHat_B;

                const Eigen::Vector3f sensAxis_N = sensInitial_N.cross(sensFinal_N).normalized();
                const Eigen::Vector3f pHat_N = (sHat_N - (sensAxis_N.dot(sHat_N)) * sensAxis_N).normalized();

                const float initMnvrAngle = safeAcosf(sensInitial_N.dot(sensFinal_N));
                const float initCelAngle = safeAcosf(pHat_N.dot(sensInitial_N));

                const Eigen::Matrix3f dcm_BR = dcm_BN * dcm_BNFinal.transpose();
                const Eigen::Vector3f prv_BR = dcmToPrv(dcm_BR);
                this->angleStart = prv_BR.norm();
                this->mnvrAxis_B = prv_BR.normalized();

                const Eigen::Vector3f sensToSunAxis_N = sensInitial_N.cross(sHat_N).normalized();
                const Eigen::Vector3f mnvrAxis_N = dcm_BN.transpose() * this->mnvrAxis_B;
                const float finalCelAngle = sensToSunAxis_N.dot(mnvrAxis_N);

                if (finalCelAngle < 0.0F && initCelAngle < initMnvrAngle) {
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
        const Eigen::Matrix3f dcm_R0N = mrpToDcm(sigma_RN);
        const Eigen::Matrix3f dcm_R0R = mrpToDcm(this->sigma_R0R);
        const Eigen::Matrix3f dcm_RN = (dcm_R0N.transpose() * dcm_R0R).transpose();

        const float dtSeconds = static_cast<float>(callTime - this->mnvrStartTime) * kNano2SecF;
        float relativeAngleCurr = this->angleStart - (this->angleRate * dtSeconds);
        relativeAngleCurr = relativeAngleCurr < 0.0F ? 0.0F : relativeAngleCurr;

        SunAvoidanceReferenceOutput out{};
        const Eigen::Vector3f prv_BR = relativeAngleCurr * this->mnvrAxis_B;
        const Eigen::Matrix3f dcmCmd_BR = prvToDcm(prv_BR);
        const Eigen::Matrix3f dcm_BR = dcm_BN * (dcmCmd_BR * dcm_RN).transpose();
        out.sigma_BR = dcmToMrp(dcm_BR);

        Eigen::Vector3f omega_RN_B = dcm_BN * omega_RN_N;
        if (relativeAngleCurr > 0.0F) {
            omega_RN_B += -this->angleRate * this->mnvrAxis_B;
        }
        out.omega_RN_B = omega_RN_B;
        out.omega_BR_B = omega_BN_B - omega_RN_B;
        out.domega_RN_B = dcm_BN * domega_RN_N;
        return out;
    }

   private:
    Eigen::Vector3f sigma_R0R;
    Eigen::Vector3f sensitiveHat_B;
    float angleRate;
    bool computeAngleStart;

    bool maneuverInitialized = false;
    float angleStart = 0.0F;
    Eigen::Vector3f mnvrAxis_B = Eigen::Vector3f::Zero();
    uint64_t mnvrStartTime = 0;
};

// ---------------------------------------------------------------------------
// Integrated regression helper: drive the sunAvoidance algorithm and the independent
// reference through a time sequence with fixed, representative navigation/reference
// inputs, and assert agreement at every step. The optional-message-derived Sun geometry
// (r_BN_N, r_SN_N) and the computeAngleStart flag are varied by the caller.
// ---------------------------------------------------------------------------
inline void integratedRegression(const Eigen::Vector3f& sensitiveHat_B,
                                 float angleRate,
                                 bool computeAngleStart,
                                 const Eigen::Vector3d& r_BN_N,
                                 const Eigen::Vector3d& r_SN_N,
                                 uint64_t stepNs,
                                 int numSteps) {
    const Eigen::Vector3f sigma_BN{0.25F, -0.45F, 0.75F};
    const Eigen::Vector3f omega_BN_B{-0.015F, -0.012F, 0.005F};
    const Eigen::Vector3f sigma_RN{0.35F, -0.25F, 0.15F};
    const Eigen::Vector3f omega_RN_N{0.018F, -0.032F, 0.015F};
    const Eigen::Vector3f domega_RN_N{0.048F, -0.022F, 0.025F};

    const auto config = SunAvoidanceConfig::create(sensitiveHat_B, angleRate, computeAngleStart);
    SunAvoidanceAlgorithm alg{config};
    AttTrackingErrorAlgorithm attError{};
    // The reference model computes the COMBINED behavior. sunAvoidance no longer applies a
    // corrected-reference offset, so the reference frame is the input reference directly (sigma_R0R == 0).
    SunAvoidanceReference ref{Eigen::Vector3f::Zero(), sensitiveHat_B, angleRate, computeAngleStart};

    const SunAvoidanceAttRefInputs refIn{sigma_RN, omega_RN_N, domega_RN_N};

    constexpr float tol = 1e-5F;
    for (int k = 0; k < numSteps; ++k) {
        const uint64_t callTime = static_cast<uint64_t>(k) * stepNs;

        // sunAvoidance produces the maneuver-adjusted reference frame ...
        const SunAvoidanceOutput adjustedRef = alg.update(sigma_BN, refIn, r_BN_N, r_SN_N, callTime);
        // ... and attTrackingError forms the attitude tracking error from it and the navigation attitude.
        const AttGuidOutput algOut =
            attError.update(AttNavInput{sigma_BN, omega_BN_B},
                            AttRefInput{adjustedRef.sigma_RN, adjustedRef.omega_RN_N, adjustedRef.domega_RN_N});

        const SunAvoidanceReferenceOutput refOut =
            ref.update(sigma_BN, omega_BN_B, sigma_RN, omega_RN_N, domega_RN_N, r_BN_N, r_SN_N, callTime);

        // attTrackingError forms sigma_BR via subMrp while the reference uses dcmToMrp; at large errors
        // these can pick different (physically identical) MRP shadow-set representatives, so compare the
        // attitude through its DCM, which is shadow-set invariant. The rate vectors are unambiguous.
        const Eigen::Matrix3f dcmBR_alg = mrpToDcm(algOut.sigma_BR);
        const Eigen::Matrix3f dcmBR_ref = mrpToDcm(refOut.sigma_BR);
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                EXPECT_NEAR(dcmBR_alg(r, c), dcmBR_ref(r, c), tol) << "sigma_BR (dcm) mismatch at step " << k;
            }
        }
        for (int i = 0; i < 3; ++i) {
            EXPECT_NEAR(algOut.omega_BR_B(i), refOut.omega_BR_B(i), tol) << "omega_BR_B mismatch at step " << k;
            EXPECT_NEAR(algOut.omega_RN_B(i), refOut.omega_RN_B(i), tol) << "omega_RN_B mismatch at step " << k;
            EXPECT_NEAR(algOut.domega_RN_B(i), refOut.domega_RN_B(i), tol) << "domega_RN_B mismatch at step " << k;
        }
    }
}

#endif  // TEST_SUNAVOIDANCE_H
