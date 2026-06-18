#include "sunTrackErrorAlgorithm.h"
#include "utilities/fsw/eigenMRP.h"
#include "utilities/fsw/eigenSupport.h"
#include "utilities/fsw/rigidBodyKinematics.hpp"
#include "utilities/fsw/safeMath.h"
#include "utilities/fsw/timeConstants.h"
#include <numbers>

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param computeStartAngle indicator whether angleStart should be computed
 */
void SunTrackErrorAlgorithm::reset(const bool computeStartAngle) {
    this->maneuverInitialized = false;
    this->computeAngleStart = computeStartAngle;
}

/*! This method computes the attitude tracking error for sun avoidance
 @return AttGuidMsgF32Payload
 @param ref attitude reference message
 @param nav attitude navigation message
 @param navTrans translational navigation message
 @param celState ephemeris message
 @param callTime The clock time at which the function was called (nanoseconds)
 */
AttGuidMsgF32Payload SunTrackErrorAlgorithm::update(AttRefMsgF32Payload& ref,
                                                    NavAttMsgF32Payload& nav,
                                                    NavTransMsgF32Payload& navTrans,
                                                    EphemerisMsgF32Payload& celState,
                                                    const uint64_t callTime) {
    if (!this->maneuverInitialized) {
        if (this->computeAngleStart) {
            const Eigen::MRPf sigma_BN = cArrayToEigenMrp(nav.sigma_BN);
            const Eigen::MRPf sigma_R0N = cArrayToEigenMrp(ref.sigma_RN);
            const Eigen::MRPf sigmaLocal_R0R(this->sigma_R0R);

            const Eigen::Vector3f sHat_N =
                (cArrayToEigenVector(celState.r_BdyZero_N) - cArrayToEigenVector(navTrans.r_BN_N))
                    .normalized()
                    .cast<float>();  //!< inertial sun direction

            const Eigen::Matrix3f dcm_BN = sigma_BN.toRotationMatrix().transpose();
            // Define initial sensitive sun direction
            const Eigen::Vector3f senstiveInitial_N = dcm_BN.transpose() * this->sensitiveHat_B;

            const Eigen::Matrix3f dcm_R0N = sigma_R0N.toRotationMatrix().transpose();
            const Eigen::Matrix3f dcm_R0R = sigmaLocal_R0R.toRotationMatrix().transpose();
            const Eigen::Matrix3f dcm_BNFinal = (dcm_R0N.transpose() * dcm_R0R).transpose();
            // Define final sensitive sun direction
            const Eigen::Vector3f senstiveFinal_N = dcm_BNFinal.transpose() * this->sensitiveHat_B;

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

    const AttGuidMsgF32Payload attGuid = computeSunTrackError(nav, ref, callTime);

    return attGuid;
}

/*! This method computes the sun tracking error
 @return AttGuidMsgF32Payload
 @param ref attitude reference message
 @param nav attitude navigation message
 @param callTime The clock time at which the function was called (nanoseconds)
 */
AttGuidMsgF32Payload SunTrackErrorAlgorithm::computeSunTrackError(NavAttMsgF32Payload& nav,
                                                                  AttRefMsgF32Payload& ref,
                                                                  const uint64_t callTime) const {
    // Convert inputs to Eigen
    const Eigen::MRPf sigmaLocal_BN = cArrayToEigenMrp(nav.sigma_BN);
    const Eigen::Vector3f omegaLocal_BN_B = cArrayToEigenVector3(nav.omega_BN_B);
    const Eigen::MRPf sigmaLocal_R0N = cArrayToEigenMrp(ref.sigma_RN);
    const Eigen::Vector3f omegaLocal_RN_N = cArrayToEigenVector3(ref.omega_RN_N);
    const Eigen::Vector3f domegaLocal_RN_N = cArrayToEigenVector3(ref.domega_RN_N);
    const Eigen::MRPf sigmaLocal_R0R(this->sigma_R0R);

    // Convert mrps to dcms
    const Eigen::Matrix3f dcm_BN = sigmaLocal_BN.toRotationMatrix().transpose();
    const Eigen::Matrix3f dcm_R0N = sigmaLocal_R0N.toRotationMatrix().transpose();
    const Eigen::Matrix3f dcm_R0R = sigmaLocal_R0R.toRotationMatrix().transpose();

    // This calculation can be seen in attitude tracking documentation
    const Eigen::Matrix3f dcm_RN = (dcm_R0N.transpose() * dcm_R0R).transpose();

    const float dtSeconds = static_cast<float>(callTime - this->mnvrStartTime) * kNano2SecF;

    // Integrate the angle to provide a feed forward rate
    float relativeAngleCurr = this->angleStart - (this->angleRate * dtSeconds);

    relativeAngleCurr = relativeAngleCurr < 0.0F ? 0.0F : relativeAngleCurr;

    AttGuidMsgF32Payload attGuidOut{};

    // This calculation can be seen in attitude tracking documentation
    const Eigen::Vector3f prv_BR = relativeAngleCurr * this->mnvrAxis_B;
    const Eigen::Matrix3f dcmCmd_BR = prvToDcm(prv_BR);
    const Eigen::Matrix3f dcm_BR = dcm_BN * (dcmCmd_BR * dcm_RN).transpose();
    const Eigen::Vector3f sigmaLocal_BR = dcmToMrp(dcm_BR);
    eigenVectorToCArray(sigmaLocal_BR, attGuidOut.sigma_BR);

    // This calculation can be seen in attitude tracking documentation
    Eigen::Vector3f omegaLocal_RN_B = dcm_BN * omegaLocal_RN_N;

    const Eigen::Vector3f omegaCatchup_BN_B = -this->angleRate * this->mnvrAxis_B;
    // Logic to provide the feedforward rate
    if (relativeAngleCurr > 0.0F) {
        omegaLocal_RN_B += omegaCatchup_BN_B;
    }
    eigenVectorToCArray(omegaLocal_RN_B, attGuidOut.omega_RN_B);

    // Perform remaining attitude tracking calculations
    const Eigen::Vector3f omegaLocal_BR_B = omegaLocal_BN_B - omegaLocal_RN_B;
    eigenVectorToCArray(omegaLocal_BR_B, attGuidOut.omega_BR_B);

    const Eigen::Vector3f domegaLocal_RN_B = dcm_BN * domegaLocal_RN_N;
    eigenVectorToCArray(domegaLocal_RN_B,
                        attGuidOut.domega_RN_B);  //!< compute reference d(omega)/dt in body frame components

    return attGuidOut;
}

/*! Set the MRP from corrected reference frame to original frame R0.
 @return void
 @param sigma [-] The MRP from corrected reference frame to original frame R0
*/
void SunTrackErrorAlgorithm::setSigma_R0R(const Eigen::Vector3f& sigma) { this->sigma_R0R = sigma; }

/*! Get the MRP from corrected reference frame to original frame R0.
 @return const Eigen::Vector3f
*/
Eigen::Vector3f SunTrackErrorAlgorithm::getSigma_R0R() const { return this->sigma_R0R; }

/*! Set the direction to exclude from the Sun in body frame components.
 @return void
 @param sensitiveDirection [-] The direction to exclude from the Sun in body frame components
*/
void SunTrackErrorAlgorithm::setSensitiveHat_B(const Eigen::Vector3f& sensitiveDirection) {
    this->sensitiveHat_B = sensitiveDirection.normalized();
}

/*! Get the direction to exclude from the Sun in body frame components.
 @return const Eigen::Vector3f
*/
Eigen::Vector3f SunTrackErrorAlgorithm::getSensitiveHat_B() const { return this->sensitiveHat_B; }

/*! Set the rate at which we maneuver to Sun point.
 @return void
 @param rate [rad/s] The rate at which we maneuver to Sun point
*/
void SunTrackErrorAlgorithm::setAngleRate(const float rate) { this->angleRate = rate; }

/*! Get the rate at which we maneuver to Sun point.
 @return const float
*/
float SunTrackErrorAlgorithm::getAngleRate() const { return this->angleRate; }
