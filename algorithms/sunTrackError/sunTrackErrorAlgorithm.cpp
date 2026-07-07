#include "sunTrackErrorAlgorithm.h"
#include "utilities/fsw/eigenMRP.h"
#include "utilities/fsw/rigidBodyKinematics.hpp"
#include "utilities/fsw/safeMath.h"
#include "utilities/fsw/timeConstants.h"
#include <numbers>

/*! Construct the algorithm with a validated configuration. Stores the configuration and clears the
 runtime maneuver state via reInitialize() so the maneuver is initialized on the first update.
 @param config Validated configuration (sigma_R0R, sensitiveHat_B, angleRate, computeAngleStart)
 */
SunTrackErrorAlgorithm::SunTrackErrorAlgorithm(const SunTrackErrorConfig& config) : cfg(config) {
    setConfig(config);
    reInitialize();
}

/*! Replace the algorithm's stored configuration at runtime. The runtime maneuver state is
 preserved; call reInitialize() to restart the maneuver from the configured values.
 @param config New validated configuration to apply
 */
void SunTrackErrorAlgorithm::setConfig(const SunTrackErrorConfig& config) { this->cfg = config; }

/*! Re-seed the runtime maneuver state so the next update recomputes the maneuver from the current
 configuration.
 @return void
 */
void SunTrackErrorAlgorithm::reInitialize() { this->maneuverInitialized = false; }

/*! This method computes the attitude tracking error for sun avoidance
 @return SunTrackErrorOutput
 @param nav attitude navigation inputs
 @param ref attitude reference inputs
 @param r_BN_N spacecraft inertial position
 @param r_SN_N sun inertial position
 @param callTime The clock time at which the function was called (nanoseconds)
 */
SunTrackErrorOutput SunTrackErrorAlgorithm::update(const SunTrackErrorNavAttInputs& nav,
                                                   const SunTrackErrorAttRefInputs& ref,
                                                   const Eigen::Vector3f& r_BN_N,
                                                   const Eigen::Vector3f& r_SN_N,
                                                   const uint64_t callTime) {
    if (!this->maneuverInitialized) {
        if (this->cfg.getComputeAngleStart()) {
            const Eigen::Vector3f sensitiveHat_B = this->cfg.getSensitiveHat_B();
            const Eigen::MRPf sigma_BN(nav.sigma_BN);
            const Eigen::MRPf sigma_R0N(ref.sigma_RN);
            const Eigen::MRPf sigmaLocal_R0R(this->cfg.getSigma_R0R());

            const Eigen::Vector3f sHat_N = (r_SN_N - r_BN_N).normalized();  //!< inertial sun direction

            const Eigen::Matrix3f dcm_BN = sigma_BN.toRotationMatrix().transpose();
            // Define initial sensitive sun direction
            const Eigen::Vector3f senstiveInitial_N = dcm_BN.transpose() * sensitiveHat_B;

            const Eigen::Matrix3f dcm_R0N = sigma_R0N.toRotationMatrix().transpose();
            const Eigen::Matrix3f dcm_R0R = sigmaLocal_R0R.toRotationMatrix().transpose();
            const Eigen::Matrix3f dcm_BNFinal = (dcm_R0N.transpose() * dcm_R0R).transpose();
            // Define final sensitive sun direction
            const Eigen::Vector3f senstiveFinal_N = dcm_BNFinal.transpose() * sensitiveHat_B;

            // Define axis of rotation for sensitive sun direction
            const Eigen::Vector3f senstiveAxis_N = (senstiveInitial_N.cross(senstiveFinal_N)).normalized();
            // Perform a Gram-Schmidt process to get a unit vector in the direction of sHat with no senstiveAxis_N comp
            const Eigen::Vector3f pHat_N = (sHat_N - (senstiveAxis_N.dot(sHat_N)) * senstiveAxis_N).normalized();

            // Define total angle between initial and final directions of senstive surface
            const float initMnvrAngle = safeAcosf(senstiveInitial_N.dot(senstiveFinal_N));
            // Define the angle between the sHatDirection not in the rotation axis and initial sensitive direction
            const float initCelAngle = safeAcosf(pHat_N.dot(senstiveInitial_N));

            const Eigen::Matrix3f dcm_BR = dcm_BN * dcm_BNFinal.transpose();
            const Eigen::Vector3f prv_BR = dcmToPrv(dcm_BR);
            this->angleStart = prv_BR.norm();        //!< Find the principal rotation angle
            this->mnvrAxis_B = prv_BR.normalized();  //!< Find the principal rotation axis

            const Eigen::Vector3f sensToSunAxis_N = (senstiveInitial_N.cross(sHat_N)).normalized();
            const Eigen::Vector3f mnvrAxis_N = dcm_BN.transpose() * this->mnvrAxis_B;
            // Define dot product between the angle between how close the sun could move to the sensitive surface
            const float finalCelAngle = sensToSunAxis_N.dot(mnvrAxis_N);

            // Logic to go the short or long rotation depending on sun avoidance
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

    return computeSunTrackError(nav, ref, callTime);
}

/*! This method computes the sun tracking error
 @return SunTrackErrorOutput
 @param nav attitude navigation inputs
 @param ref attitude reference inputs
 @param callTime The clock time at which the function was called (nanoseconds)
 */
SunTrackErrorOutput SunTrackErrorAlgorithm::computeSunTrackError(const SunTrackErrorNavAttInputs& nav,
                                                                 const SunTrackErrorAttRefInputs& ref,
                                                                 const uint64_t callTime) const {
    const Eigen::MRPf sigmaLocal_BN(nav.sigma_BN);
    const Eigen::Vector3f omegaLocal_BN_B = nav.omega_BN_B;
    const Eigen::MRPf sigmaLocal_R0N(ref.sigma_RN);
    const Eigen::Vector3f omegaLocal_RN_N = ref.omega_RN_N;
    const Eigen::Vector3f domegaLocal_RN_N = ref.domega_RN_N;
    const Eigen::MRPf sigmaLocal_R0R(this->cfg.getSigma_R0R());

    // Convert mrps to dcms
    const Eigen::Matrix3f dcm_BN = sigmaLocal_BN.toRotationMatrix().transpose();
    const Eigen::Matrix3f dcm_R0N = sigmaLocal_R0N.toRotationMatrix().transpose();
    const Eigen::Matrix3f dcm_R0R = sigmaLocal_R0R.toRotationMatrix().transpose();

    // This calculation can be seen in attitude tracking documentation
    const Eigen::Matrix3f dcm_RN = (dcm_R0N.transpose() * dcm_R0R).transpose();

    const float dtSeconds = static_cast<float>(callTime - this->mnvrStartTime) * kNano2SecF;

    // Integrate the angle to provide a feed forward rate
    float relativeAngleCurr = this->angleStart - (this->cfg.getAngleRate() * dtSeconds);

    relativeAngleCurr = relativeAngleCurr < 0.0F ? 0.0F : relativeAngleCurr;

    SunTrackErrorOutput out{};

    // This calculation can be seen in attitude tracking documentation
    const Eigen::Vector3f prv_BR = relativeAngleCurr * this->mnvrAxis_B;
    const Eigen::Matrix3f dcmCmd_BR = prvToDcm(prv_BR);
    const Eigen::Matrix3f dcm_BR = dcm_BN * (dcmCmd_BR * dcm_RN).transpose();
    out.sigma_BR = dcmToMrp(dcm_BR);

    // This calculation can be seen in attitude tracking documentation
    Eigen::Vector3f omegaLocal_RN_B = dcm_BN * omegaLocal_RN_N;

    const Eigen::Vector3f omegaCatchup_BN_B = -this->cfg.getAngleRate() * this->mnvrAxis_B;
    // Logic to provide the feedforward rate
    if (relativeAngleCurr > 0.0F) {
        omegaLocal_RN_B += omegaCatchup_BN_B;
    }
    out.omega_RN_B = omegaLocal_RN_B;

    // Perform remaining attitude tracking calculations
    out.omega_BR_B = omegaLocal_BN_B - omegaLocal_RN_B;

    out.domega_RN_B = dcm_BN * domegaLocal_RN_N;  //!< reference d(omega)/dt in body frame components

    return out;
}
