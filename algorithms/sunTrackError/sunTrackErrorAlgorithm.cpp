#include "sunTrackErrorAlgorithm.h"
#include "utilities/fsw/rigidBodyKinematics.hpp"
#include "utilities/fsw/safeMath.h"
#include "utilities/fsw/timeConstants.h"
#include <numbers>

/*! Construct the algorithm with a validated configuration. Stores the configuration and clears the
 runtime maneuver state via reInitialize() so the maneuver is initialized on the first update.
 @param config Validated configuration (sensitiveHat_B, angleRate, computeAngleStart)
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

/*! This method computes the Sun-avoidance maneuver-adjusted reference frame. On the first call the
 maneuver angle and axis are initialized from the Sun geometry; the residual angle is then fed forward
 at the configured rate on subsequent calls.
 @return SunTrackErrorOutput the maneuver-adjusted reference frame (sigma_RN, omega_RN_N, domega_RN_N)
 @param sigma_BN measured MRP attitude of the body wrt inertial N
 @param ref attitude reference inputs
 @param r_BN_N spacecraft inertial position
 @param r_SN_N sun inertial position
 @param callTime The clock time at which the function was called (nanoseconds)
 */
SunTrackErrorOutput SunTrackErrorAlgorithm::update(const Eigen::Vector3f& sigma_BN,
                                                   const SunTrackErrorAttRefInputs& ref,
                                                   const Eigen::Vector3f& r_BN_N,
                                                   const Eigen::Vector3f& r_SN_N,
                                                   const uint64_t callTime) {
    if (!this->maneuverInitialized) {
        if (this->cfg.getComputeAngleStart()) {
            initializeManeuver(sigma_BN, ref, r_BN_N, r_SN_N);
        } else {
            this->angleStart = 0.0F;
        }
        this->mnvrStartTime = callTime;

        this->maneuverInitialized = true;
    }

    return computeAdjustedReference(sigma_BN, ref, callTime);
}

/*! Initialize the Sun-avoidance maneuver from the current geometry. */
void SunTrackErrorAlgorithm::initializeManeuver(const Eigen::Vector3f& sigma_BN,
                                                const SunTrackErrorAttRefInputs& ref,
                                                const Eigen::Vector3f& r_BN_N,
                                                const Eigen::Vector3f& r_SN_N) {
    const Eigen::Vector3f sensitiveHat_B = this->cfg.getSensitiveHat_B();

    const Eigen::Vector3f sHat_N = (r_SN_N - r_BN_N).normalized();  //!< inertial sun direction

    const Eigen::Matrix3f dcm_BN = mrpToDcm(sigma_BN);
    // Define initial sensitive sun direction
    const Eigen::Vector3f senstiveInitial_N = dcm_BN.transpose() * sensitiveHat_B;

    // The final body attitude aligns with the reference frame
    const Eigen::Matrix3f dcm_BNFinal = mrpToDcm(ref.sigma_RN);
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
}

/*! This method superimposes the current maneuver rotation on the input reference frame, producing the
 maneuver-adjusted reference frame. Downstream, attTrackingError forms the attitude tracking error from
 this adjusted reference and the navigation attitude.
 @return SunTrackErrorOutput the maneuver-adjusted reference frame (sigma_RN, omega_RN_N, domega_RN_N)
 @param sigma_BN measured MRP attitude of the body wrt inertial N
 @param ref attitude reference inputs
 @param callTime The clock time at which the function was called (nanoseconds)
 */
SunTrackErrorOutput SunTrackErrorAlgorithm::computeAdjustedReference(const Eigen::Vector3f& sigma_BN,
                                                                     const SunTrackErrorAttRefInputs& ref,
                                                                     const uint64_t callTime) const {
    // Convert mrps to dcms
    const Eigen::Matrix3f dcm_BN = mrpToDcm(sigma_BN);
    const Eigen::Matrix3f dcm_RN = mrpToDcm(ref.sigma_RN);

    const float dtSeconds = static_cast<float>(callTime - this->mnvrStartTime) * kNano2SecF;

    // Integrate the angle to provide a feed forward rate
    float relativeAngleCurr = this->angleStart - (this->cfg.getAngleRate() * dtSeconds);

    relativeAngleCurr = relativeAngleCurr < 0.0F ? 0.0F : relativeAngleCurr;

    SunTrackErrorOutput out{};

    // Rotate the input reference frame by the current maneuver rotation to get the adjusted reference
    const Eigen::Vector3f prv_BR = relativeAngleCurr * this->mnvrAxis_B;
    const Eigen::Matrix3f dcmCmd_BR = prvToDcm(prv_BR);
    const Eigen::Matrix3f dcm_RcN = dcmCmd_BR * dcm_RN;
    out.sigma_RN = dcmToMrp(dcm_RcN);

    // Feed the maneuver rate forward as an extra reference angular velocity (expressed in N frame)
    Eigen::Vector3f omega_RcN_N = ref.omega_RN_N;
    if (relativeAngleCurr > 0.0F) {
        omega_RcN_N += -this->cfg.getAngleRate() * (dcm_BN.transpose() * this->mnvrAxis_B);
    }
    out.omega_RN_N = omega_RcN_N;

    // The constant-rate maneuver adds no angular acceleration, so the reference acceleration is unchanged
    out.domega_RN_N = ref.domega_RN_N;

    return out;
}
