#ifndef F32XMERA_SUN_TRACK_ERROR_TEST_HELPERS_H
#define F32XMERA_SUN_TRACK_ERROR_TEST_HELPERS_H

#include "sunTrackErrorAlgorithm.h"
#include "utilities/fsw/rigidBodyKinematics.hpp"
#include "utilities/fsw/safeMath.h"
#include "utilities/fsw/timeConstants.h"

#include <gtest/gtest.h>
#include <Eigen/Core>
#include <cstdint>
#include <numbers>

// ---------------------------------------------------------------------------
// Independent reference implementation of the sunTrackError algorithm's output: the Sun-avoidance
// maneuver-adjusted reference frame (sigma_RN, omega_RN_N, domega_RN_N).
// ---------------------------------------------------------------------------
class SunTrackErrorReference {
   public:
    SunTrackErrorReference(const Eigen::Vector3f& sensitiveHat_B, float angleRate, bool computeAngleStart)
        : sensitiveHat_B(sensitiveHat_B.normalized()), angleRate(angleRate), computeAngleStart(computeAngleStart) {}

    SunTrackErrorOutput update(const Eigen::Vector3f& sigma_BN,
                               const Eigen::Vector3f& sigma_RN,
                               const Eigen::Vector3f& omega_RN_N,
                               const Eigen::Vector3f& domega_RN_N,
                               const Eigen::Vector3f& r_BN_N,
                               const Eigen::Vector3f& r_SN_N,
                               uint64_t callTime) {
        if (!this->maneuverInitialized) {
            if (this->computeAngleStart) {
                const Eigen::Matrix3f dcm_BN = mrpToDcm(sigma_BN);
                const Eigen::Vector3f sHat_N = (r_SN_N - r_BN_N).normalized();
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
        float relativeAngleCurr = this->angleStart - (this->angleRate * dtSeconds);
        relativeAngleCurr = relativeAngleCurr < 0.0F ? 0.0F : relativeAngleCurr;

        SunTrackErrorOutput out{};
        const Eigen::Vector3f prv_cmd = relativeAngleCurr * this->mnvrAxis_B;
        const Eigen::Matrix3f dcmCmd = prvToDcm(prv_cmd);
        const Eigen::Matrix3f dcm_RcN = dcmCmd * dcm_RN;
        out.sigma_RN = dcmToMrp(dcm_RcN);

        Eigen::Vector3f omega_RcN_N = omega_RN_N;
        if (relativeAngleCurr > 0.0F) {
            omega_RcN_N += -this->angleRate * (dcm_BN.transpose() * this->mnvrAxis_B);
        }
        out.omega_RN_N = omega_RcN_N;
        out.domega_RN_N = domega_RN_N;
        return out;
    }

   private:
    Eigen::Vector3f sensitiveHat_B;
    float angleRate;
    bool computeAngleStart;

    bool maneuverInitialized = false;
    float angleStart = 0.0F;
    Eigen::Vector3f mnvrAxis_B = Eigen::Vector3f::Zero();
    uint64_t mnvrStartTime = 0;
};

// ---------------------------------------------------------------------------
// Regression helper: drive the algorithm and the independent reference through a time sequence with
// the given configuration, navigation attitude, reference frame and Sun geometry, and assert the
// adjusted-reference output agrees at every step.
// ---------------------------------------------------------------------------
inline void regressionTestSunTrackError(const Eigen::Vector3f& sensitiveHat_B,
                                        float angleRate,
                                        bool computeAngleStart,
                                        const Eigen::Vector3f& sigma_BN,
                                        const Eigen::Vector3f& sigma_RN,
                                        const Eigen::Vector3f& omega_RN_N,
                                        const Eigen::Vector3f& domega_RN_N,
                                        const Eigen::Vector3f& r_BN_N,
                                        const Eigen::Vector3f& r_SN_N,
                                        uint64_t stepNs,
                                        int numSteps) {
    const auto config = SunTrackErrorConfig::create(sensitiveHat_B, angleRate, computeAngleStart);
    SunTrackErrorAlgorithm alg{config};
    SunTrackErrorReference ref{sensitiveHat_B, angleRate, computeAngleStart};

    const SunTrackErrorAttRefInputs refIn{sigma_RN, omega_RN_N, domega_RN_N};

    constexpr float tol = 1e-5F;
    for (int k = 0; k < numSteps; ++k) {
        const uint64_t callTime = static_cast<uint64_t>(k) * stepNs;
        const SunTrackErrorOutput algOut = alg.update(sigma_BN, refIn, r_BN_N, r_SN_N, callTime);
        const SunTrackErrorOutput refOut =
            ref.update(sigma_BN, sigma_RN, omega_RN_N, domega_RN_N, r_BN_N, r_SN_N, callTime);

        for (int i = 0; i < 3; ++i) {
            EXPECT_NEAR(algOut.sigma_RN(i), refOut.sigma_RN(i), tol) << "sigma_RN mismatch at step " << k;
            EXPECT_NEAR(algOut.omega_RN_N(i), refOut.omega_RN_N(i), tol) << "omega_RN_N mismatch at step " << k;
            EXPECT_NEAR(algOut.domega_RN_N(i), refOut.domega_RN_N(i), tol) << "domega_RN_N mismatch at step " << k;
        }
    }
}

#endif  // F32XMERA_SUN_TRACK_ERROR_TEST_HELPERS_H
