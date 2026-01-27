/*
    Attitude Tracking Error Module for Sun Avoidance
 */

#include "sunTrackError.h"

#include "architecture/utilities/eigenMRP.h"
#include "architecture/utilities/eigenSupport.h"
#include "architecture/utilities/macroDefinitions.h"
#include "architecture/utilities/rigidBodyKinematics.hpp"

#include <stdexcept>

// For localized cast using NANO2SEC macro
inline double nsToSec(const long long& ns) { return (double)(ns * NANO2SEC); }

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 */
void SunTrackError::reset(uint64_t callTime) {
    // check if the required input messages are included
    if (!this->attRefInMsg.isLinked()) {
        throw std::invalid_argument("sunTrackError.attRefInMsg wasn't connected.");
    }
    if (!this->attNavInMsg.isLinked()) {
        throw std::invalid_argument("sunTrackError.attNavInMsg wasn't connected.");
    }

    this->maneuverInitialized = 0;
}

/*! Add a description of what this main Update() routine does for this module
 @return void
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void SunTrackError::updateState(uint64_t callTime) {
    AttRefMsgPayload ref = this->attRefInMsg();  //!< reference guidance message
    NavAttMsgPayload nav = this->attNavInMsg();  //!< attitude navigation message

    if (this->maneuverInitialized != 1) {
        if (this->transNavInMsg.isLinked() && this->ephemerisInMsg.isLinked()) {
            Eigen::MRPd sigma_BN((Eigen::Vector3d)nav.sigma_BN);
            Eigen::MRPd sigma_R0N((Eigen::Vector3d)ref.sigma_RN);
            Eigen::MRPd sigmaLocal_R0R((Eigen::MRPd)this->sigma_R0R);

            NavTransMsgPayload navTrans = this->transNavInMsg();    //!< Get access to the spacecraft position
            EphemerisMsgPayload celState = this->ephemerisInMsg();  //!< Get access to the sun position

            Eigen::Vector3d sHat_N = ((Eigen::Vector3d)celState.r_BdyZero_N - (Eigen::Vector3d)navTrans.r_BN_N)
                                         .normalized();  //!< inertial sun direction

            Eigen::Matrix3d dcm_BN = sigma_BN.toRotationMatrix().transpose();
            // Define initial sensitive sun direction
            Eigen::Vector3d senstiveInitial_N = dcm_BN.transpose() * this->sensitiveHat_B;

            Eigen::Matrix3d dcm_R0N = sigma_R0N.toRotationMatrix().transpose();
            Eigen::Matrix3d dcm_R0R = sigmaLocal_R0R.toRotationMatrix().transpose();
            Eigen::Matrix3d dcm_BNFinal = (dcm_R0N.transpose() * dcm_R0R).transpose();
            // Define final sensitive sun direction
            Eigen::Vector3d senstiveFinal_N = dcm_BNFinal.transpose() * this->sensitiveHat_B;

            // Define axis of rotation for sensitive sun direction
            Eigen::Vector3d senstiveAxis_N = (senstiveInitial_N.cross(senstiveFinal_N)).normalized();
            // Perform a Gram-Schmidt process to get a unit vector in the direction of sHat with no senstiveAxis_N comp
            Eigen::Vector3d pHat_N = (sHat_N - (senstiveAxis_N.dot(sHat_N)) * senstiveAxis_N).normalized();

            // Define total angle between initial and final directions of senstive surface
            double initMnvrAngle = acos(senstiveInitial_N.dot(senstiveFinal_N));
            // Define the angle between the sHatDirection not in the rotation axis and initial sensitive direction
            double initCelAngle = acos(pHat_N.dot(senstiveInitial_N));

            Eigen::Matrix3d dcm_BR = dcm_BN * dcm_BNFinal.transpose();
            Eigen::Vector3d prv_BR = dcmToPrv(dcm_BR);
            this->angleStart = prv_BR.norm();        //!< Find the principal rotation angle
            this->mnvrAxis_B = prv_BR.normalized();  //!< Find the principal rotation axis

            Eigen::Vector3d sensToSunAxis_N = senstiveInitial_N.cross(sHat_N);  //!< This should be normalized, correct?
            Eigen::Vector3d mnvrAxis_N = dcm_BN.transpose() * this->mnvrAxis_B;
            // Define dot product between the angle between how close the sun could move to the sensitive surface
            double finalCelAngle = sensToSunAxis_N.dot(mnvrAxis_N);

            // Logic to go the short or long rotation depending on sun avoidance
            if (finalCelAngle < 0.0 && initCelAngle < initMnvrAngle) {
                this->angleStart = 2.0 * M_PI - this->angleStart;
                this->mnvrAxis_B = -this->mnvrAxis_B;
            }

        } else {
            this->angleStart = 0.0;
        }
        this->mnvrStartTime = callTime;

        this->maneuverInitialized = 1;
    }

    computeSunTrackError(nav.sigma_BN,
                         nav.omega_BN_B,
                         ref.sigma_RN,
                         ref.omega_RN_N,
                         ref.domega_RN_N,
                         this->attGuid.sigma_BR,
                         this->attGuid.omega_BR_B,
                         this->attGuid.omega_RN_B,
                         this->attGuid.domega_RN_B,
                         callTime);

    /*! write output message */
    this->attGuidOutMsg.write(&this->attGuid, this->moduleID, callTime);
}

void SunTrackError::computeSunTrackError(double sigma_BN[3],
                                         double omega_BN_B[3],
                                         double sigma_R0N[3],
                                         double omega_RN_N[3],
                                         double domega_RN_N[3],
                                         double sigma_BR[3],
                                         double omega_BR_B[3],
                                         double omega_RN_B[3],
                                         double domega_RN_B[3],
                                         uint64_t callTime) {
    // Convert inputs to Eigen
    Eigen::MRPd sigmaLocal_BN((Eigen::Vector3d)sigma_BN);
    Eigen::Vector3d omegaLocal_BN_B((Eigen::Vector3d)omega_BN_B);
    Eigen::MRPd sigmaLocal_R0N((Eigen::Vector3d)sigma_R0N);
    Eigen::Vector3d omegaLocal_RN_N((Eigen::Vector3d)omega_RN_N);
    Eigen::Vector3d domegaLocal_RN_N((Eigen::Vector3d)domega_RN_N);
    Eigen::MRPd sigmaLocal_R0R((Eigen::MRPd)this->sigma_R0R);

    // Convert mrps to dcms
    Eigen::Matrix3d dcm_BN = sigmaLocal_BN.toRotationMatrix().transpose();
    Eigen::Matrix3d dcm_R0N = sigmaLocal_R0N.toRotationMatrix().transpose();
    Eigen::Matrix3d dcm_R0R = sigmaLocal_R0R.toRotationMatrix().transpose();

    // This calculation can be seen in attitude tracking documentation
    Eigen::Matrix3d dcm_RN = (dcm_R0N.transpose() * dcm_R0R).transpose();

    double dtSeconds = nsToSec(callTime - this->mnvrStartTime);

    // Integrate the angle to provide a feed forward rate
    double relativeAngleCurr = this->angleStart - this->angleRate * dtSeconds;

    relativeAngleCurr = relativeAngleCurr < 0.0 ? 0.0 : relativeAngleCurr;

    // This calculation can be seen in attitude tracking documentation
    Eigen::Vector3d prv_BR = relativeAngleCurr * this->mnvrAxis_B;
    Eigen::Matrix3d dcmCmd_BR = prvToDcm(prv_BR);
    Eigen::Matrix3d dcm_BR = dcm_BN * (dcmCmd_BR * dcm_RN).transpose();
    Eigen::Vector3d sigmaLocal_BR = dcmToMrp(dcm_BR);
    eigenVectorToCArray(sigmaLocal_BR, sigma_BR);

    // This calculation can be seen in attitude tracking documentation
    Eigen::Vector3d omegaLocal_RN_B = dcm_BN * omegaLocal_RN_N;
    eigenVectorToCArray(omegaLocal_RN_B, omega_RN_B);

    Eigen::Vector3d omegaCatchup_BN_B = -this->angleRate * this->mnvrAxis_B;
    // Logic to provide the feedforward rate
    if (relativeAngleCurr > 0.0) {
        omegaLocal_RN_B = omegaLocal_RN_B + omegaCatchup_BN_B;
        eigenVectorToCArray(omegaLocal_RN_B, omega_RN_B);
    }

    // Perform remaining attitude tracking calculations
    Eigen::Vector3d omegaLocal_BR_B = omegaLocal_BN_B - omegaLocal_RN_B;
    eigenVectorToCArray(omegaLocal_BR_B, omega_BR_B);

    Eigen::Vector3d domegaLocal_RN_B = dcm_BN * domegaLocal_RN_N;
    eigenVectorToCArray(domegaLocal_RN_B, domega_RN_B);  //!< compute reference d(omega)/dt in body frame components
}

/*! Set the MRP from corrected reference frame to original frame R0.
 @return void
 @param sigma [-] The MRP from corrected reference frame to original frame R0
*/
void SunTrackError::setSigma_R0R(const Eigen::Vector3d& sigma) {
    this->sigma_R0R = sigma;
}

/*! Get the MRP from corrected reference frame to original frame R0.
 @return const Eigen::Vector3d
*/
Eigen::Vector3d SunTrackError::getSigma_R0R() const { return this->sigma_R0R; }

/*! Set the direction to exclude from the Sun in body frame components.
 @return void
 @param sensitiveDirection [-] The direction to exclude from the Sun in body frame components
*/
void SunTrackError::setSensitiveHat_B(const Eigen::Vector3d& sensitiveDirection) {
    this->sensitiveHat_B = sensitiveDirection;
}

/*! Get the direction to exclude from the Sun in body frame components.
 @return const Eigen::Vector3d
*/
Eigen::Vector3d SunTrackError::getSensitiveHat_B() const { return this->sensitiveHat_B; }

/*! Set the rate at which we maneuver to Sun point.
 @return void
 @param rate [rad/s] The rate at which we maneuver to Sun point
*/
void SunTrackError::setAngleRate(const double rate) {
    this->angleRate = rate;
}

/*! Get the rate at which we maneuver to Sun point.
 @return const double
*/
double SunTrackError::getAngleRate() const { return this->angleRate; }
